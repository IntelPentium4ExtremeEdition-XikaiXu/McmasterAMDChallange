/*
 * TEAM TBJ — INDUSTRIAL CRANE POSITION-TRACKING PROOF OF CONCEPT
 * Authors: Yuchen Tao, Zihao Zhou, Xikai Xu
 * C11 / POSIX / standard library only.  Synthetic engineering research demo.
 *
 * Features: 3D crane reference motion; encoder odometry; UWB-like ABSOLUTE fixes;
 * scalar-per-axis Gaussian Kalman update with joint innovation gate; faults;
 * live read-only SCADA-style API; coil event emulation; CSV logging;
 * deterministic Monte Carlo; extended simulated-time soak; dashboard server.
 *
 * WARNING: No crane control, no physical I/O, not safety-rated or field-validated.
 */
#define _POSIX_C_SOURCE 200809L
#include <arpa/inet.h>
#include <stdatomic.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define DT 0.1
#define RUN_SECONDS 180.0
#define HIST_CAP 512
#define JSON_CAP 262144
#define FIX_PERIOD_TICKS 5
#define GATE_D2 16.27
#define PI 3.14159265358979323846

typedef struct { double x,y,z; } V3;
typedef enum { NOMINAL=0, RANDOM_LOSS=1, OCCLUSION_BIAS=2 } Scenario;
typedef enum { WAREHOUSE=0, REPORT_TRAJECTORY=1 } Profile;
typedef struct { double t; V3 truth,enc,fused; double e_enc,e_fused; int fix; } Sample;
typedef struct { char id[24]; V3 storage; int status; } Coil;
typedef struct {
    uint64_t rng; int have_gauss; double spare;
    Scenario scenario; Profile profile;
    int tick; double time_s; double max_time_s;
    V3 truth, true_velocity, enc, fused, initial_bias;
    V3 covariance, run_bias;
    double last_fix_s; int fix_status; /* 1 accept, -1 rejected, 0 not available/interval */
    long accepted,rejected,missing,fix_attempts;
    double sq_enc,sq_fused,max_enc,max_fused;
    int err_n,err_capacity; double *enc_errors,*fused_errors;
    Sample history[HIST_CAP]; int hist_pos,hist_count;
    Coil coils[2]; char carrying[24]; int completed_moves;
    char latest_event[190];
    FILE *trace,*events;
} Sim;

typedef struct { pthread_mutex_t mu; Sim s; int paused, speed, logging; atomic_int keep_running; } Shared;
static Shared g = { .mu=PTHREAD_MUTEX_INITIALIZER, .paused=0,.speed=5,.keep_running=1,.logging=1};

static V3 v3(double x,double y,double z){ V3 v={x,y,z}; return v; }
static V3 plus(V3 a,V3 b){ return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static V3 minus(V3 a,V3 b){ return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static V3 times(V3 a,double k){return v3(a.x*k,a.y*k,a.z*k);}
static double norm3(V3 a){return sqrt(a.x*a.x+a.y*a.y+a.z*a.z);}
static double clamp(double x,double a,double b){return x<a?a:(x>b?b:x);}
static const char *scenario_name(Scenario c){return c==NOMINAL?"nominal":c==RANDOM_LOSS?"random_loss":"occlusion_and_bias";}
static const char *profile_name(Profile p){return p==WAREHOUSE?"warehouse_route":"report_trajectory";}
static uint64_t random_u64(Sim *s){
    uint64_t x=s->rng;
    x^=x>>12; x^=x<<25; x^=x>>27; s->rng=x;
    return x * UINT64_C(2685821657736338717);
}
static double uniform01(Sim *s){return ((random_u64(s)>>11)+0.5)*(1.0/9007199254740992.0);}
static double normal(Sim *s){
    if(s->have_gauss){s->have_gauss=0; return s->spare;}
    double r=sqrt(-2.0*log(uniform01(s))),t=2.0*PI*uniform01(s);
    s->spare=r*sin(t); s->have_gauss=1; return r*cos(t);
}
static void mkdir_results(void){ (void)mkdir("results",0755); }

/* A synthetic warehouse crane route with two pick-up/drop-off cycles. */
typedef struct {double t; V3 p;} Waypoint;
static const Waypoint route[]={
    {0,{7,7,12}}, {15,{10,10,12}}, {25,{10,10,3}},
    {35,{10,10,12}}, {66,{62,39,12}}, {76,{62,39,3}},
    {86,{62,39,12}}, {100,{31,45,12}}, {110,{31,45,3}},
    {120,{31,45,12}}, {145,{80,16,12}}, {155,{80,16,3}},
    {166,{80,16,12}}, {180,{7,7,12}}
};
static void get_truth(Profile p,double t,V3 *pos,V3 *velocity){
    if(p==REPORT_TRAJECTORY){
        /* Matches the time functions in TBJ's earlier Python technical study. */
        *pos=v3(30+22*sin(2*PI*t/250-.65),
                 12+8*cos(2*PI*t/190+.3),
                 4+2*sin(2*PI*t/130));
        *velocity=v3(22*2*PI/250*cos(2*PI*t/250-.65),
                     -8*2*PI/190*sin(2*PI*t/190+.3),
                     2*2*PI/130*cos(2*PI*t/130));
        return;
    }
    double local=fmod(t,RUN_SECONDS);
    if(t>0 && local<1e-9) local=RUN_SECONDS;
    size_t size=sizeof(route)/sizeof(route[0]);
    for(size_t i=1;i<size;i++){
        if(local<=route[i].t){
            double duration=route[i].t-route[i-1].t;
            double u=clamp((local-route[i-1].t)/duration,0,1);
            double smooth=u*u*(3-2*u), ds=6*u*(1-u)/duration;
            V3 delta=minus(route[i].p,route[i-1].p);
            *pos=plus(route[i-1].p,times(delta,smooth));
            *velocity=times(delta,ds); return;
        }
    }
    *pos=route[size-1].p; *velocity=v3(0,0,0);
}
static void sim_init(Sim *s,Scenario scenario,Profile profile,uint64_t seed,int collect_errors){
    memset(s,0,sizeof(*s));
    s->scenario=scenario;s->profile=profile;s->rng=seed?seed:123456789;
    s->max_time_s=RUN_SECONDS;
    get_truth(profile,0,&s->truth,&s->true_velocity);
    s->run_bias=v3(.006+.003*normal(s),-.004+.003*normal(s),.002+.0015*normal(s));
    V3 init_error=v3(.12*normal(s),.12*normal(s),.08*normal(s));
    s->enc=s->fused=plus(s->truth,init_error);
    s->covariance=v3(.12*.12,.12*.12,.08*.08);
    s->last_fix_s=0;
    s->coils[0]=(Coil){"A1058",{10,10,3},0};
    s->coils[1]=(Coil){"B2062",{31,45,3},0};
    snprintf(s->latest_event,sizeof(s->latest_event),"SYSTEM STARTED — position reference initialized");
    if(collect_errors){
        s->err_capacity=(int)(RUN_SECONDS/DT)+4;
        s->enc_errors=calloc((size_t)s->err_capacity,sizeof(double));
        s->fused_errors=calloc((size_t)s->err_capacity,sizeof(double));
        if(!s->enc_errors||!s->fused_errors){perror("calloc");exit(2);}
    }
}
static void sim_free(Sim *s){
    if(s->trace) fclose(s->trace);
    if(s->events) fclose(s->events);
    free(s->enc_errors);free(s->fused_errors);
    s->trace=NULL;s->events=NULL;s->enc_errors=NULL;s->fused_errors=NULL;
}
static double sim_err_enc(const Sim *s){return norm3(minus(s->enc,s->truth));}
static double sim_err_fused(const Sim *s){return norm3(minus(s->fused,s->truth));}
static void record_event(Sim *s,const char *verb,const char *id,V3 p){
    snprintf(s->latest_event,sizeof(s->latest_event),"t=%.1fs | %s | coil %s | estimated location (%.2f, %.2f, %.2f) m",
             s->time_s,verb,id,p.x,p.y,p.z);
    if(s->events){fprintf(s->events,"%.1f,%s,%s,%.6f,%.6f,%.6f\n",s->time_s,verb,id,p.x,p.y,p.z);fflush(s->events);}
}
static void coil_event(Sim *s){
    if(s->profile!=WAREHOUSE || s->time_s>180.01) return;
    /* Emulated MES/RFID ID and gripper signals; NOT inferred from XYZ alone. */
    if(s->tick==260){
        s->coils[0].status=1; snprintf(s->carrying,sizeof(s->carrying),"A1058");
        record_event(s,"PICK_UP","A1058",s->fused);
    } else if(s->tick==770){
        s->coils[0].status=2;s->coils[0].storage=s->fused;s->carrying[0]=0;
        s->completed_moves++;record_event(s,"DROP_OFF","A1058",s->fused);
    } else if(s->tick==1110){
        s->coils[1].status=1;snprintf(s->carrying,sizeof(s->carrying),"B2062");
        record_event(s,"PICK_UP","B2062",s->fused);
    } else if(s->tick==1560){
        s->coils[1].status=2;s->coils[1].storage=s->fused;s->carrying[0]=0;
        s->completed_moves++;record_event(s,"DROP_OFF","B2062",s->fused);
    }
}
static void sim_record(Sim *s){
    double ee=sim_err_enc(s), ef=sim_err_fused(s);
    Sample sample={s->time_s,s->truth,s->enc,s->fused,ee,ef,s->fix_status};
    s->history[s->hist_pos]=sample;
    s->hist_pos=(s->hist_pos+1)%HIST_CAP;
    if(s->hist_count<HIST_CAP) s->hist_count++;
    if(s->trace){
        fprintf(s->trace,"%.1f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d\n",
                s->time_s,s->truth.x,s->truth.y,s->truth.z,
                s->enc.x,s->enc.y,s->enc.z,s->fused.x,s->fused.y,s->fused.z,ee,ef,s->fix_status);
        fflush(s->trace);
    }
}
static void sim_step(Sim *s){
    if(s->time_s>=s->max_time_s-1e-8) return;
    s->tick++;
    s->time_s=s->tick*DT;
    get_truth(s->profile,s->time_s,&s->truth,&s->true_velocity);
    /* Three independent noisy velocity measurements from encoders. */
    V3 observed=plus(s->true_velocity,s->run_bias);
    observed=plus(observed,v3(.07*normal(s),.07*normal(s),.05*normal(s)));
    if(s->time_s>=65 && s->time_s<87) observed=plus(observed,v3(.035,-.025,.010));
    V3 displacement=times(observed,DT);
    s->enc=plus(s->enc,displacement);
    s->fused=plus(s->fused,displacement);
    s->covariance.x+=pow(.07*DT,2)+.018*.018;
    s->covariance.y+=pow(.07*DT,2)+.018*.018;
    s->covariance.z+=pow(.05*DT,2)+.018*.018;
    s->fix_status=0;
    if(s->tick%FIX_PERIOD_TICKS==0){
        s->fix_attempts++;
        int available=1;
        if(s->scenario==RANDOM_LOSS && uniform01(s)<.30) available=0;
        if(s->scenario==OCCLUSION_BIAS){
            if(s->time_s>=80 && s->time_s<110) available=0;
            else if(uniform01(s)<.15) available=0;
        }
        if(!available){s->missing++;s->fix_status=0;}
        else {
            /* Position fixes are a UWB-like abstraction, not a radio/ranging solver. */
            V3 measurement=plus(s->truth,v3(.15*normal(s),.15*normal(s),.10*normal(s)));
            if(s->scenario==OCCLUSION_BIAS && s->time_s>=110 && s->time_s<137)
                measurement=plus(measurement,v3(.95,-.70,.45));
            V3 residual=minus(measurement,s->fused);
            V3 R=v3(.15*.15,.15*.15,.10*.10);
            double d2=residual.x*residual.x/(s->covariance.x+R.x)
                    +residual.y*residual.y/(s->covariance.y+R.y)
                    +residual.z*residual.z/(s->covariance.z+R.z);
            if(d2>GATE_D2){s->rejected++;s->fix_status=-1;}
            else {
                double kx=s->covariance.x/(s->covariance.x+R.x);
                double ky=s->covariance.y/(s->covariance.y+R.y);
                double kz=s->covariance.z/(s->covariance.z+R.z);
                s->fused=plus(s->fused,v3(kx*residual.x,ky*residual.y,kz*residual.z));
                s->covariance=v3((1-kx)*(1-kx)*s->covariance.x+kx*kx*R.x,
                                 (1-ky)*(1-ky)*s->covariance.y+ky*ky*R.y,
                                 (1-kz)*(1-kz)*s->covariance.z+kz*kz*R.z);
                s->accepted++;s->fix_status=1;s->last_fix_s=s->time_s;
            }
        }
    }
    double ee=sim_err_enc(s),ef=sim_err_fused(s);
    s->sq_enc+=ee*ee;s->sq_fused+=ef*ef;
    if(ee>s->max_enc)s->max_enc=ee;
    if(ef>s->max_fused)s->max_fused=ef;
    if(s->err_n<s->err_capacity){
        s->enc_errors[s->err_n]=ee;
        s->fused_errors[s->err_n]=ef;
        s->err_n++;
    }
    coil_event(s);
    if(s->tick%5==0) sim_record(s);
}
static double rmse(double squares,int count){return count>0?sqrt(squares/count):0;}
static int compare_doubles(const void *a,const void *b){double d=*(const double*)a-*(const double*)b;return (d>0)-(d<0);}
static double p95(const double *values,int n){
    if(!values||n<=0)return 0;
    double *copy=malloc((size_t)n*sizeof(*copy));
    if(!copy)return 0;
    memcpy(copy,values,(size_t)n*sizeof(*copy));
    qsort(copy,(size_t)n,sizeof(*copy),compare_doubles);
    double index=.95*(n-1);int lo=(int)floor(index),hi=(int)ceil(index);
    double answer=copy[lo]+(index-lo)*(copy[hi]-copy[lo]);free(copy);return answer;
}
static void open_logs(Sim *s){
    mkdir_results();
    s->trace=fopen("results/live_trace.csv","w");
    s->events=fopen("results/coil_events.csv","w");
    if(s->trace)fprintf(s->trace,"time_s,true_x_m,true_y_m,true_z_m,encoder_x_m,encoder_y_m,encoder_z_m,fused_x_m,fused_y_m,fused_z_m,encoder_error_3d_m,fused_error_3d_m,fix_status\n");
    if(s->events)fprintf(s->events,"time_s,event,coil_id,est_x_m,est_y_m,est_z_m\n");
    sim_record(s);
}
static void write_design_exports(void){
    mkdir_results();
    FILE *f=fopen("results/technology_comparison.csv","w");
    if(f){
        fputs("technology,strengths,industrial_limitations,recommendation\n"
              "Encoder and PLC feedback,Direct axis motion and mature OT interfaces,Slip homing and access permissions,Preferred baseline\n"
              "UWB positioning,Independent absolute reference,Steel multipath and NLOS,Conditional pilot for drift correction\n"
              "LiDAR laser,Potentially accurate absolute range,Dust occlusion and alignment,Consider only with clear geometry\n"
              "Machine vision,Visual markers and object identification,Lighting and maintenance,Future extension\n"
              "IMU inertial,Fast cheap motion assistance,Integrated position drift,Auxiliary only\n"
              "Hybrid,Combines local continuity and drift correction,Complexity and fault detection,Preferred evaluated architecture\n",f);fclose(f);
    }
    f=fopen("results/cost_estimates_CAD.csv","w");
    if(f){
        fputs("item,quantity,unit_cost_CAD,extended_CAD,scope\n"
              "Axis feedback devices,3,275,825,baseline crane\n"
              "Mechanical coupling mounting,3,180,540,baseline crane\n"
              "Industrial IO interface,1,1500,1500,baseline crane\n"
              "Edge gateway,1,1100,1100,baseline crane\n"
              "Industrial networking,1,600,600,baseline crane\n"
              "Cables enclosures protection,1,1100,1100,baseline crane\n"
              "Installation labor h,24,110,2640,baseline crane\n"
              "Commissioning labor h,16,110,1760,baseline crane\n"
              "Baseline total,1,10065,10065,per crane\n"
              "Optional tag and calibration,1,1860,1860,incremental per crane\n"
              "Shared six anchor infrastructure,1,8900,8900,per one zone\n"
              "Integration engineering allowance,1,4400,4400,per zone\n"
              "One crane hybrid pilot,1,25225,25225,scenario total\n"
              "Five crane one zone hybrid,1,72925,72925,scenario total\n",f);fclose(f);
    }
    f=fopen("results/risks.csv","w");
    if(f){
        fputs("risk,impact,proposed_mitigation,residual_concern\n"
              "Steel NLOS multipath,Absolute fix bias and blackout,Innovation gating and field survey,High\n"
              "Encoder slip and homing,Coordinate drift,Periodic reference checks,Medium\n"
              "Cable and load swing,Coil-drop position offset,Settled-load and geometry correction,High\n"
              "Network or time-sync outage,Delayed or stale telemetry,Timestamp quality status and buffered logs,Medium\n"
              "Electrical noise and vibration,Intermittent sensor faults,Shielding isolation filters and industrial housing,Medium\n"
              "OT cybersecurity,Unauthorized interaction,Read-only segregated reviewed plant network,High\n"
              "Premature inventory update,Wrong coil placement,Independent MES ID and human verification,High\n"
              "Synthetic-to-field gap,Overestimated performance,Instrumented measured pilot before rollout,High\n",f);fclose(f);
    }
    f=fopen("results/requirements_matrix.csv","w");
    if(f){
        fputs("id,AMD_challenge_requirement,demo_evidence,limitation\n"
              "R1,Continuous 3D XYZ position,C estimation at 10 Hz plus on-demand JSON query,Simulated sensors only\n"
              "R2,Robust in steel dust vibration and noise,Injected losses bias drift and outlier gate,Does not model actual RF propagation\n"
              "R3,24x7 high availability and low maintenance,Fast-forward simulated-time soak CLI,Does not prove real hardware 24x7 reliability\n"
              "R4,Updates appropriate for coil location,Positions and simulated pickup dropoff and inventory,Coil identity events are emulated\n"
              "R5,OT information integration,Read-only HTTP JSON API,Real OT protocol and approvals pending\n"
              "R6,SCADA API or TCP IP query,Versioned position endpoint and C client,HTTP on TCP loopback only\n"
              "R7,Visualization and historical route,Local browser dashboard and CSV trace,Not a production HMI\n"
              "R8,Cost scalability installation maintenance,CAD cost table risk register and roadmap,Assumed costs not vendor quotes\n",f);fclose(f);
    }
}
static void write_summary(FILE *dest,const Sim *s){
    fprintf(dest,"scenario=%s profile=%s simulated_time=%.1fs samples=%d\n",
            scenario_name(s->scenario),profile_name(s->profile),s->time_s,s->tick);
    fprintf(dest,"encoder_3D_RMSE=%.4fm hybrid_3D_RMSE=%.4fm\n",rmse(s->sq_enc,s->tick),rmse(s->sq_fused,s->tick));
    fprintf(dest,"encoder_P95=%.4fm hybrid_P95=%.4fm (stored samples=%d)\n",
            p95(s->enc_errors,s->err_n),p95(s->fused_errors,s->err_n),s->err_n);
    fprintf(dest,"accepted=%ld rejected=%ld missing=%ld attempted=%ld availability_of_accepted_fixes=%.1f%%\n",
            s->accepted,s->rejected,s->missing,s->fix_attempts,
            s->fix_attempts?100.0*s->accepted/s->fix_attempts:0.0);
}
/* Dependency-free SVG artifacts for presentations and static report figures. */
static void make_benchmark_svg(double means[3][7],int runs){
    FILE *f=fopen("results/benchmark_comparison.svg","w");if(!f)return;
    fprintf(f,"<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 1000 530\">\n"
      "<rect width=\"1000\" height=\"530\" fill=\"#102031\"/>\n"
      "<text x=\"60\" y=\"62\" fill=\"#eff7ff\" font-size=\"26\" font-family=\"Arial\" font-weight=\"bold\">"
      "Team TBJ | Synthetic Positioning Comparison</text>\n"
      "<text x=\"60\" y=\"91\" fill=\"#a6bfd2\" font-size=\"14\" font-family=\"Arial\">"
      "Mean 3D RMSE across %d simulation trials per scenario (C implementation)</text>\n",runs);
    const double maxy=2.0, bottom=423, scale=275/maxy;
    for(int tick=0;tick<=4;tick++){
        double value=tick*maxy/4, y=bottom-value*scale;
        fprintf(f,"<line x1=\"93\" y1=\"%.1f\" x2=\"928\" y2=\"%.1f\" stroke=\"#2b4960\"/>"
                  "<text x=\"38\" y=\"%.1f\" font-family=\"Arial\" font-size=\"13\" fill=\"#a3b7c9\">%.1f</text>\n",y,y,y+5,value);
    }
    const char *names[3]={"Nominal","30% random loss","Occlusion + bias"};
    for(int k=0;k<3;k++){
        int center=228+k*278;
        for(int j=0;j<2;j++){
            double val=means[k][j],hh=val*scale;
            fprintf(f,"<rect x=\"%d\" y=\"%.2f\" width=\"64\" height=\"%.2f\" fill=\"%s\" rx=\"3\"/>\n",
                 center+(j?10:-66),bottom-hh,hh,j?"#43d0c8":"#f1a262");
            fprintf(f,"<text x=\"%d\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"Arial\" font-size=\"15\" font-weight=\"bold\" fill=\"#e8f2f8\">%.3f m</text>\n",
                    center+(j?42:-34),bottom-hh-10,val);
        }
        fprintf(f,"<text x=\"%d\" y=\"454\" text-anchor=\"middle\" font-family=\"Arial\" font-size=\"15\" fill=\"#cfdeec\">%s</text>\n",center,names[k]);
    }
    fputs("<rect x=\"98\" y=\"480\" width=\"15\" height=\"9\" fill=\"#f1a262\"/>"
          "<text x=\"120\" y=\"490\" fill=\"#c6d7e4\" font-family=\"Arial\" font-size=\"13\">Encoder only</text>"
          "<rect x=\"260\" y=\"480\" width=\"15\" height=\"9\" fill=\"#43d0c8\"/>"
          "<text x=\"282\" y=\"490\" fill=\"#c6d7e4\" font-family=\"Arial\" font-size=\"13\">Hybrid estimator</text>"
          "<text x=\"746\" y=\"490\" fill=\"#8ca9bc\" font-family=\"Arial\" font-size=\"11\">NOT PLANT DATA</text></svg>\n",f);
    fclose(f);
}
static void make_trace_svgs(const Sim *s){
    for(int chart=0;chart<2;chart++){
        FILE *f=fopen(chart?"results/warehouse_trajectory.svg":"results/error_over_time.svg","w");
        if(!f)continue;
        fprintf(f,"<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 1000 480\">"
          "<rect width=\"1000\" height=\"480\" fill=\"#102031\"/>"
          "<text x=\"50\" y=\"48\" fill=\"#ebf5ff\" font-size=\"25\" font-weight=\"bold\" font-family=\"Arial\">%s</text>"
          "<text x=\"50\" y=\"74\" fill=\"#91b0c3\" font-size=\"13\" font-family=\"Arial\">"
          "%s | synthetic simulation, one deterministic run</text>\n",
          chart?"Crane XY Trajectory (Top View)":"3D Position Error vs Time",scenario_name(s->scenario));
        if(!chart && s->scenario==OCCLUSION_BIAS){
            fputs("<rect x=\"450\" y=\"95\" width=\"150\" height=\"290\" fill=\"#af7f43\" opacity=\"0.20\"/>"
                  "<rect x=\"600\" y=\"95\" width=\"135\" height=\"290\" fill=\"#bc5b61\" opacity=\"0.15\"/>\n",f);
        }
        for(int n=0;n<=4;n++){
            double y=385-n*72.5;
            fprintf(f,"<line x1=\"68\" y1=\"%.1f\" x2=\"965\" y2=\"%.1f\" stroke=\"#27465a\"/>\n",y,y);
        }
        for(int axis=0;axis<3;axis++){
            const char *color=axis==0?"#8aa6c4":axis==1?"#f1a262":"#43d0c8";
            fprintf(f,"<polyline fill=\"none\" stroke=\"%s\" stroke-width=\"%.1f\" points=\"",color,axis==2?2.2:1.4);
            for(int i=0;i<s->hist_count;i++){
                int idx=(s->hist_pos-s->hist_count+i+HIST_CAP)%HIST_CAP;
                const Sample *a=&s->history[idx];
                double x,y;
                if(chart){
                    V3 v=axis==0?a->truth:axis==1?a->enc:a->fused;
                    x=68+v.x*8.85;
                    y=385-v.y*5.0;
                }else{
                    double e=axis==0?0:axis==1?a->e_enc:a->e_fused;
                    x=68+a->t*4.95;y=385-fmin(e,3.5)*83;
                }
                fprintf(f,"%.1f,%.1f ",x,y);
            }
            fputs("\"/>\n",f);
        }
        fprintf(f,"<text x=\"65\" y=\"418\" fill=\"#91aec1\" font-family=\"Arial\" font-size=\"13\">%s</text>\n",
                chart?"Warehouse X position (m) vs Y position (m) (scaled axes)":"Time: 0 to 180 seconds | Y axis: 3D position error (m), 0 to 3.5");
        fputs("<rect x=\"75\" y=\"436\" width=\"13\" height=\"7\" fill=\"#8aa6c4\"/>"
              "<text x=\"95\" y=\"446\" fill=\"#c7dae8\" font-family=\"Arial\" font-size=\"12\">Reference / baseline zero error</text>"
              "<rect x=\"350\" y=\"436\" width=\"13\" height=\"7\" fill=\"#f1a262\"/>"
              "<text x=\"370\" y=\"446\" fill=\"#c7dae8\" font-family=\"Arial\" font-size=\"12\">Encoder</text>"
              "<rect x=\"488\" y=\"436\" width=\"13\" height=\"7\" fill=\"#43d0c8\"/>"
              "<text x=\"508\" y=\"446\" fill=\"#c7dae8\" font-family=\"Arial\" font-size=\"12\">Fused</text>"
              "<text x=\"815\" y=\"446\" fill=\"#8ca4b5\" font-family=\"Arial\" font-size=\"11\">NOT PLANT DATA</text></svg>\n",f);
        fclose(f);
    }
}
static int run_benchmark(int runs){
    if(runs<=0||runs>10000){fprintf(stderr,"Runs must be 1..10000\n");return 2;}
    write_design_exports();
    FILE *raw=fopen("results/benchmark_runs.csv","w"),*summary=fopen("results/benchmark_summary.csv","w");
    if(!raw||!summary){perror("benchmark CSV");return 2;}
    fprintf(raw,"scenario,seed,encoder_rmse_m,hybrid_rmse_m,encoder_p95_m,hybrid_p95_m,accepted,rejected,missing\n");
    fprintf(summary,"scenario,runs,encoder_rmse_mean_m,hybrid_rmse_mean_m,encoder_p95_mean_m,hybrid_p95_mean_m,accepted_mean,rejected_mean,missing_mean\n");
    printf("TEAM TBJ: synthetic Monte Carlo, %d seeds per scenario, 180 s / run\n",runs);
    double means[3][7]={{0}};
    for(int m=0;m<3;m++){
        double sums[7]={0};
        for(int k=0;k<runs;k++){
            Sim s;sim_init(&s,(Scenario)m,REPORT_TRAJECTORY,(uint64_t)(2026+k),1);
            for(int j=0;j<1800;j++)sim_step(&s);
            double a[7]={rmse(s.sq_enc,s.tick),rmse(s.sq_fused,s.tick),
                          p95(s.enc_errors,s.err_n),p95(s.fused_errors,s.err_n),
                          (double)s.accepted,(double)s.rejected,(double)s.missing};
            for(int i=0;i<7;i++)sums[i]+=a[i];
            fprintf(raw,"%s,%d,%.7f,%.7f,%.7f,%.7f,%ld,%ld,%ld\n",
                    scenario_name((Scenario)m),2026+k,a[0],a[1],a[2],a[3],s.accepted,s.rejected,s.missing);
            sim_free(&s);
        }
        for(int i=0;i<7;i++){sums[i]/=runs;means[m][i]=sums[i];}
        fprintf(summary,"%s,%d,%.7f,%.7f,%.7f,%.7f,%.3f,%.3f,%.3f\n",
                scenario_name((Scenario)m),runs,sums[0],sums[1],sums[2],sums[3],sums[4],sums[5],sums[6]);
        printf("  %-22s encoder RMSE=%6.3fm | hybrid RMSE=%6.3fm | hybrid P95=%6.3fm\n",
               scenario_name((Scenario)m),sums[0],sums[1],sums[3]);
    }
    fclose(raw);fclose(summary);make_benchmark_svg(means,runs);
    printf("Saved: results/benchmark_runs.csv, results/benchmark_summary.csv, results/benchmark_comparison.svg\n");
    return 0;
}
static int run_headless(Scenario scenario,Profile profile){
    write_design_exports();Sim s;sim_init(&s,scenario,profile,2051,1);open_logs(&s);
    for(int k=0;k<1800;k++)sim_step(&s);
    write_summary(stdout,&s);make_trace_svgs(&s);sim_free(&s);
    printf("Saved: results/live_trace.csv, results/coil_events.csv, results/error_over_time.svg, results/warehouse_trajectory.svg\n");return 0;
}
static int run_soak(double duration){
    if(duration<=0||duration>604800){fprintf(stderr,"Soak seconds must be 0..604800\n");return 2;}
    /* Continuous mathematical trajectory, no server/restart/hardware during the run.
     * Every simulated step executes real C estimation and random fault logic.
     */
    Sim s;sim_init(&s,RANDOM_LOSS,REPORT_TRAJECTORY,3097,0);
    int steps=(int)(duration/DT+.5);
    s.max_time_s=steps*DT;
    struct timespec started,done;clock_gettime(CLOCK_MONOTONIC,&started);
    for(int i=0;i<steps;i++)sim_step(&s);
    clock_gettime(CLOCK_MONOTONIC,&done);
    double wall=(done.tv_sec-started.tv_sec)+(done.tv_nsec-started.tv_nsec)/1e9;
    printf("TBJ uninterrupted C estimator soak: %.0f synthetic seconds in %.2f wall seconds\n",duration,wall);
    printf("samples=%d, encoder RMSE=%.3fm, hybrid RMSE=%.3fm, accepted=%ld, missing=%ld\n",
           steps,sqrt(s.sq_enc/steps),sqrt(s.sq_fused/steps),s.accepted,s.missing);
    printf("NOTE: fast-forward simulation only. No proof of actual hardware uptime, memory growth under live API traffic, or plant availability.\n");
    sim_free(&s);return 0;
}

static void json_v3(char *dst,size_t size,V3 p){snprintf(dst,size,"[%.5f,%.5f,%.5f]",p.x,p.y,p.z);}
static void write_state_json(const Sim *s,char *out,size_t cap){
    char t[96],en[96],fu[96],c0[96],c1[96];
    json_v3(t,sizeof(t),s->truth);json_v3(en,sizeof(en),s->enc);
    json_v3(fu,sizeof(fu),s->fused);
    json_v3(c0,sizeof(c0),s->coils[0].storage);json_v3(c1,sizeof(c1),s->coils[1].storage);
    double since=s->time_s-s->last_fix_s;
    const char *status=(since>1.0+1e-8)?"DEGRADED_STALE":"VALID_SIMULATED";
    double er=sim_err_enc(s),fr=sim_err_fused(s);
    snprintf(out,cap,
        "{\"crane_id\":\"TBJ-01\",\"coordinate_frame\":\"TBJ_WAREHOUSE_LOCAL_METRES\","
        "\"scenario\":\"%s\",\"profile\":\"%s\",\"time_s\":%.1f,\"status\":\"%s\","
        "\"units\":\"m\",\"truth_xyz\":%s,\"encoder_xyz\":%s,\"fused_xyz\":%s,"
        "\"encoder_error_3d_m\":%.5f,\"fused_error_3d_m\":%.5f,\"encoder_rmse_3d_m\":%.5f,"
        "\"hybrid_rmse_3d_m\":%.5f,\"accepted_fixes\":%ld,\"rejected_fixes\":%ld,"
        "\"missing_fixes\":%ld,\"total_fix_opportunities\":%ld,\"time_since_fix_s\":%.2f,"
        "\"fix_status\":%d,\"covariance_trace_m2\":%.6f,"
        "\"carrying\":\"%s\",\"completed_moves\":%d,\"inventory\":["
        "{\"id\":\"%s\",\"status\":%d,\"xyz\":%s},"
        "{\"id\":\"%s\",\"status\":%d,\"xyz\":%s}],"
        "\"latest_event\":\"%s\",\"duration_s\":180}",
        scenario_name(s->scenario),profile_name(s->profile),s->time_s,status,t,en,fu,er,fr,
        rmse(s->sq_enc,s->tick),rmse(s->sq_fused,s->tick),
        s->accepted,s->rejected,s->missing,s->fix_attempts,since,s->fix_status,
        s->covariance.x+s->covariance.y+s->covariance.z,s->carrying,s->completed_moves,
        s->coils[0].id,s->coils[0].status,c0,s->coils[1].id,s->coils[1].status,c1,
        s->latest_event);
}
static void write_metrics_json(const Sim *s,char *out,size_t cap){
    snprintf(out,cap,"{\"time_s\":%.1f,\"scenario\":\"%s\",\"encoder_rmse_3d_m\":%.6f,"
        "\"hybrid_rmse_3d_m\":%.6f,\"encoder_p95_3d_m\":%.6f,\"hybrid_p95_3d_m\":%.6f,"
        "\"encoder_max_error_m\":%.6f,\"hybrid_max_error_m\":%.6f,"
        "\"accepted\":%ld,\"rejected\":%ld,\"missing\":%ld,"
        "\"fix_accept_percent\":%.4f,\"provisional_nominal_gate_rmse_m\":0.25,"
        "\"provisional_nominal_gate_p95_m\":0.50,\"metrics_from\":\"synthetic_simulation\"}",
        s->time_s,scenario_name(s->scenario),rmse(s->sq_enc,s->tick),rmse(s->sq_fused,s->tick),
        p95(s->enc_errors,s->err_n),p95(s->fused_errors,s->err_n),s->max_enc,s->max_fused,
        s->accepted,s->rejected,s->missing,s->fix_attempts?100.0*s->accepted/s->fix_attempts:0.0);
}
static void write_history_json(const Sim *s,char *out,size_t cap){
    size_t offset=0;
    offset+=(size_t)snprintf(out+offset,cap-offset,"{\"samples\":[");
    for(int j=0;j<s->hist_count;j++){
        int i=(s->hist_pos-s->hist_count+j+HIST_CAP)%HIST_CAP;
        const Sample *p=&s->history[i];
        if(offset+210>=cap)break;
        offset+=(size_t)snprintf(out+offset,cap-offset,
            "%s[%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]",
            j?",":"",p->t,p->truth.x,p->truth.y,p->truth.z,
            p->enc.x,p->enc.y,p->enc.z,p->fused.x,p->fused.y,p->fused.z,p->e_enc,p->e_fused);
    }
    snprintf(out+offset,cap-offset,"]}");
}
static void reply_http(int fd,int code,const char *type,const char *data,size_t n){
    char header[350];
    int k=snprintf(header,sizeof(header),
      "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\n"
      "Cache-Control: no-store\r\nAccess-Control-Allow-Origin: http://127.0.0.1\r\n"
      "Connection: close\r\nX-Content-Type-Options: nosniff\r\n\r\n",
      code,code==200?"OK":code==404?"Not Found":code==405?"Method Not Allowed":"Bad Request",type,n);
    (void)send(fd,header,(size_t)k,0);
    size_t sent=0;
    while(sent<n){ssize_t written=send(fd,data+sent,n-sent,0);if(written<=0)break;sent+=(size_t)written;}
}
static void reply_text(int fd,int status,const char *type,const char *message){reply_http(fd,status,type,message,strlen(message));}
static void serve_file(int fd,const char *path,const char *ctype){
    FILE *f=fopen(path,"rb");
    if(!f){reply_text(fd,404,"text/plain; charset=utf-8","Start the program inside its extracted project folder (web/ required).\n");return;}
    fseek(f,0,SEEK_END);long n=ftell(f);rewind(f);
    if(n<0||n>1000000){fclose(f);reply_text(fd,400,"text/plain","Invalid file\n");return;}
    char *data=malloc((size_t)n+1);
    if(!data){fclose(f);reply_text(fd,400,"text/plain","Out of memory\n");return;}
    if(fread(data,1,(size_t)n,f)!=(size_t)n){free(data);fclose(f);reply_text(fd,400,"text/plain","File read error\n");return;}
    fclose(f);reply_http(fd,200,ctype,data,(size_t)n);free(data);
}
static void shared_reset(Scenario scenario,Profile profile){
    Sim fresh;sim_init(&fresh,scenario,profile,2051,1);
    sim_free(&g.s);g.s=fresh;
    if(g.logging)open_logs(&g.s);
    g.paused=0;
}
static void handle_client(int fd){
    char req[4096];ssize_t n=recv(fd,req,sizeof(req)-1,0);
    if(n<=0)return;
    req[n]=0;
    char method[10],uri[2048];
    if(sscanf(req,"%9s %2047s",method,uri)!=2){reply_text(fd,400,"text/plain","Bad request\n");return;}
    if(!strcmp(uri,"/")||!strcmp(uri,"/index.html")){serve_file(fd,"web/index.html","text/html; charset=utf-8");return;}
    if(!strcmp(uri,"/app.js")){serve_file(fd,"web/app.js","text/javascript; charset=utf-8");return;}
    if(!strcmp(uri,"/style.css")){serve_file(fd,"web/style.css","text/css; charset=utf-8");return;}
    char *q=strchr(uri,'?');if(q)*q++=0;
    if(!strcmp(uri,"/api/control")){
        if(strcmp(method,"POST")){reply_text(fd,405,"text/plain","POST only\n");return;}
        pthread_mutex_lock(&g.mu);
        if(q && !strncmp(q,"scenario=",9)){
            int m=atoi(q+9);if(m>=0&&m<=2)shared_reset((Scenario)m,g.s.profile);
        }else if(q && !strncmp(q,"profile=",8)){
            int p=atoi(q+8);if(p>=0&&p<=1)shared_reset(g.s.scenario,(Profile)p);
        }else if(q && !strcmp(q,"restart=1"))shared_reset(g.s.scenario,g.s.profile);
        else if(q && !strcmp(q,"pause=1"))g.paused=1;
        else if(q && !strcmp(q,"pause=0"))g.paused=0;
        else if(q && !strncmp(q,"speed=",6)){
            int speed=atoi(q+6);if(speed>=1&&speed<=20)g.speed=speed;
        }
        pthread_mutex_unlock(&g.mu);
        reply_text(fd,200,"application/json","{\"ok\":true}\n");return;
    }
    if(!strcmp(uri,"/api/state")||!strcmp(uri,"/api/v1/cranes/TBJ-01/position")||
       !strcmp(uri,"/api/metrics")||!strcmp(uri,"/api/history")){
        char *json=malloc(JSON_CAP);
        if(!json){reply_text(fd,400,"text/plain","Out of memory");return;}
        pthread_mutex_lock(&g.mu);
        if(!strcmp(uri,"/api/history"))write_history_json(&g.s,json,JSON_CAP);
        else if(!strcmp(uri,"/api/metrics"))write_metrics_json(&g.s,json,JSON_CAP);
        else write_state_json(&g.s,json,JSON_CAP);
        pthread_mutex_unlock(&g.mu);
        reply_http(fd,200,"application/json; charset=utf-8",json,strlen(json));free(json);return;
    }
    reply_text(fd,404,"text/plain","Not found\n");
}
static void *simulation_worker(void *ignored){
    (void)ignored;
    struct timespec wait_time={0,100000000L};
    while(atomic_load(&g.keep_running)){
        nanosleep(&wait_time,NULL);
        pthread_mutex_lock(&g.mu);
        if(!g.paused){
            for(int i=0;i<g.speed;i++)sim_step(&g.s);
            if(g.s.time_s>=RUN_SECONDS-1e-8)g.paused=1;
        }
        pthread_mutex_unlock(&g.mu);
    }
    return NULL;
}
static volatile sig_atomic_t terminate_requested=0;
static void catch_signal(int sig){(void)sig;terminate_requested=1;}
static int run_server(int port){
    if(port<1024||port>65535){fprintf(stderr,"Port must be 1024..65535\n");return 2;}
    signal(SIGINT,catch_signal);signal(SIGTERM,catch_signal);signal(SIGPIPE,SIG_IGN);
    write_design_exports();
    sim_init(&g.s,NOMINAL,WAREHOUSE,2051,1);
    if(g.logging)open_logs(&g.s);
    int server=socket(AF_INET,SOCK_STREAM,0);if(server<0){perror("socket");return 2;}
    int one=1;setsockopt(server,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
    struct sockaddr_in addr={0};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    addr.sin_port=htons((uint16_t)port);
    if(bind(server,(struct sockaddr*)&addr,sizeof(addr))<0){perror("bind");close(server);return 2;}
    if(listen(server,12)<0){perror("listen");close(server);return 2;}
    pthread_t worker;
    if(pthread_create(&worker,NULL,simulation_worker,NULL)){perror("thread");close(server);return 2;}
    printf("\nTEAM TBJ — C Crane Tracking Demonstration\n");
    printf("Open http://127.0.0.1:%d in your browser\n",port);
    printf("SCADA GET: http://127.0.0.1:%d/api/v1/cranes/TBJ-01/position\n",port);
    printf("Metrics:   http://127.0.0.1:%d/api/metrics\n",port);
    printf("Logs:      results/live_trace.csv, results/coil_events.csv\n");
    printf("Press Ctrl+C to end. ALL motion and measurements are synthetic.\n\n");fflush(stdout);
    while(!terminate_requested){
        fd_set rfds;FD_ZERO(&rfds);FD_SET(server,&rfds);
        struct timeval tv={0,200000};
        int r=select(server+1,&rfds,NULL,NULL,&tv);
        if(r<0){if(errno==EINTR)continue;perror("select");break;}
        if(r==0)continue;
        int client=accept(server,NULL,NULL);
        if(client>=0){handle_client(client);close(client);}
    }
    atomic_store(&g.keep_running,0);pthread_join(worker,NULL);
    close(server);
    pthread_mutex_lock(&g.mu);sim_free(&g.s);pthread_mutex_unlock(&g.mu);
    printf("Demo closed. Logs saved under results/.\n");return 0;
}
static void usage(const char *argv0){
    printf("TEAM TBJ / Synthetic 3D Crane Tracking — C11\n"
      "Usage:\n"
      "  %s --serve [port]        Local live dashboard + SCADA API (default port 8080)\n"
      "  %s --benchmark [runs]    3 scenarios × runs seeds (default 120)\n"
      "  %s --run [mode] [profile] 180 s headless trace and coil events\n"
      "     mode: 0 nominal | 1 30%% dropout | 2 blackout + bias\n"
      "     profile: 0 warehouse route | 1 earlier report trajectory\n"
      "  %s --soak [seconds]     Fast-forward software soak (e.g. 86400)\n"
      "  %s --help\n",argv0,argv0,argv0,argv0,argv0);
}
int main(int argc,char **argv){
    if(argc<2||!strcmp(argv[1],"--help")){usage(argv[0]);return 0;}
    if(!strcmp(argv[1],"--serve"))return run_server(argc>2?atoi(argv[2]):8080);
    if(!strcmp(argv[1],"--benchmark"))return run_benchmark(argc>2?atoi(argv[2]):120);
    if(!strcmp(argv[1],"--run")){
        int m=argc>2?atoi(argv[2]):0,p=argc>3?atoi(argv[3]):0;
        if(m<0||m>2||p<0||p>1){usage(argv[0]);return 2;}
        return run_headless((Scenario)m,(Profile)p);
    }
    if(!strcmp(argv[1],"--soak"))return run_soak(argc>2?atof(argv[2]):86400);
    usage(argv[0]);return 2;
}
