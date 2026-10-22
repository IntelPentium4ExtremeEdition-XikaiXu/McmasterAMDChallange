/*
 * TEAM TBJ — 3D Visualization for Crane Position Tracking PoC
 * Extra file: real-time 3D window (raylib)
 * Ubuntu 24.04 + CPU rendering OK
 *
 * Compile (after installing raylib):
 *   sudo apt update
 *   sudo apt install libraylib-dev
 *
 *   gcc tbj_3d_vis.c -o crane3d -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -O2
 *
 * Run:
 *   ./crane3d
 *   ./crane3d --scenario 2 --speed 8
 *
 * Controls:
 *   SPACE      pause / resume
 *   1 / 2 / 3  switch scenario (nominal / random loss / occlusion+bias)
 *   UP / DOWN  change simulation speed
 *   R          restart
 *   C          toggle camera follow
 *   ESC / Q    quit
 *
 * All motion & measurements are synthetic. NOT plant data.
 */

#define _POSIX_C_SOURCE 200809L
#include "raylib.h"
#include "raymath.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdint.h>

#define DT              0.1
#define RUN_SECONDS     180.0
#define HIST_CAP        600
#define FIX_PERIOD      5
#define GATE_D2         16.27
#define PI              3.14159265358979323846

/* ------------------------------------------------------------------ */
/* Minimal simulation (same math as original tbj_crane.c)             */
/* ------------------------------------------------------------------ */

typedef struct { double x, y, z; } V3;
typedef enum { NOMINAL=0, RANDOM_LOSS=1, OCCLUSION_BIAS=2 } Scenario;

typedef struct {
    double t;
    V3 truth, enc, fused;
    double e_enc, e_fused;
    int fix;
} Sample;

typedef struct {
    char id[24];
    V3 storage;
    int status;          /* 0 free, 1 carried, 2 dropped */
} Coil;

typedef struct {
    uint64_t rng;
    int have_gauss;
    double spare;

    Scenario scenario;
    int tick;
    double time_s;
    double max_time_s;

    V3 truth, true_velocity;
    V3 enc, fused;
    V3 covariance;
    V3 run_bias;

    double last_fix_s;
    int fix_status;      /* 1 accept, -1 reject, 0 none */

    long accepted, rejected, missing, fix_attempts;
    double sq_enc, sq_fused;
    double max_enc, max_fused;

    Sample history[HIST_CAP];
    int hist_pos, hist_count;

    Coil coils[2];
    char carrying[24];
    int completed_moves;
    char latest_event[192];
} Sim;

static V3 v3(double x, double y, double z) { V3 v = {x,y,z}; return v; }
static V3 plus(V3 a, V3 b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static V3 minus(V3 a, V3 b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static V3 times(V3 a, double k) { return v3(a.x*k, a.y*k, a.z*k); }
static double norm3(V3 a) { return sqrt(a.x*a.x + a.y*a.y + a.z*a.z); }
static double clampd(double x, double a, double b) { return x < a ? a : (x > b ? b : x); }

static uint64_t random_u64(Sim *s) {
    uint64_t x = s->rng;
    x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
    s->rng = x;
    return x * UINT64_C(2685821657736338717);
}
static double uniform01(Sim *s) {
    return ((random_u64(s) >> 11) + 0.5) * (1.0 / 9007199254740992.0);
}
static double normal(Sim *s) {
    if (s->have_gauss) { s->have_gauss = 0; return s->spare; }
    double r = sqrt(-2.0 * log(uniform01(s)));
    double t = 2.0 * PI * uniform01(s);
    s->spare = r * sin(t);
    s->have_gauss = 1;
    return r * cos(t);
}

typedef struct { double t; V3 p; } Waypoint;
static const Waypoint route[] = {
    {0,  {7,7,12}}, {15,{10,10,12}}, {25,{10,10,3}},
    {35, {10,10,12}}, {66,{62,39,12}}, {76,{62,39,3}},
    {86, {62,39,12}}, {100,{31,45,12}}, {110,{31,45,3}},
    {120,{31,45,12}}, {145,{80,16,12}}, {155,{80,16,3}},
    {166,{80,16,12}}, {180,{7,7,12}}
};

static void get_truth(double t, V3 *pos, V3 *vel) {
    double local = fmod(t, RUN_SECONDS);
    if (t > 0 && local < 1e-9) local = RUN_SECONDS;
    size_t n = sizeof(route) / sizeof(route[0]);
    for (size_t i = 1; i < n; i++) {
        if (local <= route[i].t) {
            double dur = route[i].t - route[i-1].t;
            double u = clampd((local - route[i-1].t) / dur, 0, 1);
            double smooth = u*u*(3 - 2*u);
            double ds = 6*u*(1-u) / dur;
            V3 delta = minus(route[i].p, route[i-1].p);
            *pos = plus(route[i-1].p, times(delta, smooth));
            *vel = times(delta, ds);
            return;
        }
    }
    *pos = route[n-1].p;
    *vel = v3(0,0,0);
}

static void sim_init(Sim *s, Scenario sc, uint64_t seed) {
    memset(s, 0, sizeof(*s));
    s->scenario = sc;
    s->rng = seed ? seed : 2051;
    s->max_time_s = RUN_SECONDS;
    get_truth(0, &s->truth, &s->true_velocity);
    s->run_bias = v3(0.006 + 0.003*normal(s),
                     -0.004 + 0.003*normal(s),
                      0.002 + 0.0015*normal(s));
    V3 init_err = v3(0.12*normal(s), 0.12*normal(s), 0.08*normal(s));
    s->enc = s->fused = plus(s->truth, init_err);
    s->covariance = v3(0.12*0.12, 0.12*0.12, 0.08*0.08);
    s->coils[0] = (Coil){"A1058", {10,10,3}, 0};
    s->coils[1] = (Coil){"B2062", {31,45,3}, 0};
    snprintf(s->latest_event, sizeof(s->latest_event),
             "SYSTEM STARTED — position reference initialized");
}

static double sim_err_enc(const Sim *s) { return norm3(minus(s->enc, s->truth)); }
static double sim_err_fused(const Sim *s) { return norm3(minus(s->fused, s->truth)); }

static void record_event(Sim *s, const char *verb, const char *id, V3 p) {
    snprintf(s->latest_event, sizeof(s->latest_event),
             "t=%.1fs | %s | coil %s | (%.2f, %.2f, %.2f)",
             s->time_s, verb, id, p.x, p.y, p.z);
}

static void coil_event(Sim *s) {
    if (s->time_s > 180.01) return;
    if (s->tick == 260) {
        s->coils[0].status = 1;
        snprintf(s->carrying, sizeof(s->carrying), "A1058");
        record_event(s, "PICK_UP", "A1058", s->fused);
    } else if (s->tick == 770) {
        s->coils[0].status = 2;
        s->coils[0].storage = s->fused;
        s->carrying[0] = 0;
        s->completed_moves++;
        record_event(s, "DROP_OFF", "A1058", s->fused);
    } else if (s->tick == 1110) {
        s->coils[1].status = 1;
        snprintf(s->carrying, sizeof(s->carrying), "B2062");
        record_event(s, "PICK_UP", "B2062", s->fused);
    } else if (s->tick == 1560) {
        s->coils[1].status = 2;
        s->coils[1].storage = s->fused;
        s->carrying[0] = 0;
        s->completed_moves++;
        record_event(s, "DROP_OFF", "B2062", s->fused);
    }
}

static void sim_record(Sim *s) {
    Sample sm = {
        s->time_s, s->truth, s->enc, s->fused,
        sim_err_enc(s), sim_err_fused(s), s->fix_status
    };
    s->history[s->hist_pos] = sm;
    s->hist_pos = (s->hist_pos + 1) % HIST_CAP;
    if (s->hist_count < HIST_CAP) s->hist_count++;
}

static void sim_step(Sim *s) {
    if (s->time_s >= s->max_time_s - 1e-8) return;
    s->tick++;
    s->time_s = s->tick * DT;
    get_truth(s->time_s, &s->truth, &s->true_velocity);

    V3 observed = plus(s->true_velocity, s->run_bias);
    observed = plus(observed, v3(0.07*normal(s), 0.07*normal(s), 0.05*normal(s)));
    if (s->time_s >= 65 && s->time_s < 87)
        observed = plus(observed, v3(0.035, -0.025, 0.010));

    V3 disp = times(observed, DT);
    s->enc   = plus(s->enc, disp);
    s->fused = plus(s->fused, disp);

    s->covariance.x += pow(0.07*DT, 2) + 0.018*0.018;
    s->covariance.y += pow(0.07*DT, 2) + 0.018*0.018;
    s->covariance.z += pow(0.05*DT, 2) + 0.018*0.018;

    s->fix_status = 0;
    if (s->tick % FIX_PERIOD == 0) {
        s->fix_attempts++;
        int available = 1;
        if (s->scenario == RANDOM_LOSS && uniform01(s) < 0.30) available = 0;
        if (s->scenario == OCCLUSION_BIAS) {
            if (s->time_s >= 80 && s->time_s < 110) available = 0;
            else if (uniform01(s) < 0.15) available = 0;
        }
        if (!available) {
            s->missing++;
        } else {
            V3 meas = plus(s->truth, v3(0.15*normal(s), 0.15*normal(s), 0.10*normal(s)));
            if (s->scenario == OCCLUSION_BIAS && s->time_s >= 110 && s->time_s < 137)
                meas = plus(meas, v3(0.95, -0.70, 0.45));

            V3 residual = minus(meas, s->fused);
            V3 R = v3(0.15*0.15, 0.15*0.15, 0.10*0.10);
            double d2 = residual.x*residual.x / (s->covariance.x + R.x)
                      + residual.y*residual.y / (s->covariance.y + R.y)
                      + residual.z*residual.z / (s->covariance.z + R.z);

            if (d2 > GATE_D2) {
                s->rejected++;
                s->fix_status = -1;
            } else {
                double kx = s->covariance.x / (s->covariance.x + R.x);
                double ky = s->covariance.y / (s->covariance.y + R.y);
                double kz = s->covariance.z / (s->covariance.z + R.z);
                s->fused = plus(s->fused, v3(kx*residual.x, ky*residual.y, kz*residual.z));
                s->covariance = v3((1-kx)*(1-kx)*s->covariance.x + kx*kx*R.x,
                                   (1-ky)*(1-ky)*s->covariance.y + ky*ky*R.y,
                                   (1-kz)*(1-kz)*s->covariance.z + kz*kz*R.z);
                s->accepted++;
                s->fix_status = 1;
                s->last_fix_s = s->time_s;
            }
        }
    }

    double ee = sim_err_enc(s), ef = sim_err_fused(s);
    s->sq_enc += ee*ee;
    s->sq_fused += ef*ef;
    if (ee > s->max_enc) s->max_enc = ee;
    if (ef > s->max_fused) s->max_fused = ef;

    coil_event(s);
    if (s->tick % 3 == 0) sim_record(s);
}

static double rmse(double sq, int n) { return n > 0 ? sqrt(sq / n) : 0.0; }

/* ------------------------------------------------------------------ */
/* 3D Visualization                                                   */
/* ------------------------------------------------------------------ */

static Color col_truth   = { 100, 180, 255, 255 };   /* blue  */
static Color col_enc     = { 255, 160,  60, 255 };   /* orange */
static Color col_fused   = {  100, 220, 200, 255 };   /* cyan  */
static Color col_crane   = { 200, 200, 210, 255 };
static Color col_rail    = { 140, 150, 160, 255 };
static Color col_floor   = {  40,  55,  70, 255 };
static Color col_grid    = {  60,  80, 100, 120 };
static Color col_coil    = { 220, 120,  50, 255 };
static Color col_hook    = { 255, 220,  80, 255 };

static void DrawWarehouseFloor(void) {
    /* large floor */
    DrawPlane((Vector3){45, 0, 25}, (Vector2){100, 60}, col_floor);

    /* grid lines */
    for (int x = 0; x <= 90; x += 5) {
        DrawLine3D((Vector3){(float)x, 0.02f, 0}, (Vector3){(float)x, 0.02f, 55}, col_grid);
    }
    for (int z = 0; z <= 55; z += 5) {
        DrawLine3D((Vector3){0, 0.02f, (float)z}, (Vector3){90, 0.02f, (float)z}, col_grid);
    }

    /* coordinate axes helper */
    DrawLine3D((Vector3){0,0.05f,0}, (Vector3){10,0.05f,0}, RED);
    DrawLine3D((Vector3){0,0.05f,0}, (Vector3){0,5,0}, GREEN);
    DrawLine3D((Vector3){0,0.05f,0}, (Vector3){0,0.05f,10}, BLUE);
}

static void DrawOverheadCrane(V3 pos, int carrying) {
    float x = (float)pos.x;
    float y = (float)pos.y;   /* height */
    float z = (float)pos.z;   /* depth  — note: raylib Y-up, we map sim Y→Z */

    /* In simulation: X, Y (depth), Z (height)
       In raylib:     X, Y (up),   Z (depth)
       Mapping:  sim(x,y,z) → ray(x, z, y)   */

    float rx = x;
    float ry = (float)pos.z;   /* height */
    float rz = (float)pos.y;   /* depth  */

    /* long rails along X (at high Y) */
    DrawCube((Vector3){45, 14.5f, 5},  92, 0.4f, 0.6f, col_rail);
    DrawCube((Vector3){45, 14.5f, 50}, 92, 0.4f, 0.6f, col_rail);

    /* bridge (moves in Z / depth) */
    DrawCube((Vector3){45, 14.2f, rz}, 92, 0.35f, 1.2f, col_crane);

    /* trolley (moves in X) */
    DrawCube((Vector3){rx, 13.8f, rz}, 2.2f, 0.7f, 1.8f, (Color){180,190,200,255});

    /* vertical rope */
    DrawLine3D((Vector3){rx, 13.5f, rz}, (Vector3){rx, ry + 0.8f, rz}, (Color){220,220,230,255});
    DrawLine3D((Vector3){rx+0.05f, 13.5f, rz}, (Vector3){rx+0.05f, ry + 0.8f, rz}, (Color){180,180,190,200});

    /* hook */
    DrawSphere((Vector3){rx, ry + 0.4f, rz}, 0.35f, col_hook);
    DrawCube((Vector3){rx, ry + 0.15f, rz}, 0.6f, 0.25f, 0.4f, (Color){160,160,170,255});

    /* load if carrying */
    if (carrying) {
        DrawCylinderEx((Vector3){rx, ry - 0.6f, rz},
                       (Vector3){rx, ry - 1.8f, rz},
                       0.9f, 0.9f, 16, col_coil);
        DrawCylinderEx((Vector3){rx, ry - 1.8f, rz},
                       (Vector3){rx, ry - 2.0f, rz},
                       1.0f, 1.0f, 16, (Color){180,90,30,255});
    }
}

static void DrawCoilOnFloor(const Coil *c) {
    if (c->status == 1) return; /* currently carried */
    float rx = (float)c->storage.x;
    float ry = (float)c->storage.z;
    float rz = (float)c->storage.y;

    Color col = (c->status == 2) ? (Color){100, 180, 100, 255} : col_coil;
    DrawCylinderEx((Vector3){rx, 0.05f, rz},
                   (Vector3){rx, 1.6f, rz},
                   1.1f, 1.1f, 20, col);
    DrawCylinderEx((Vector3){rx, 1.6f, rz},
                   (Vector3){rx, 1.75f, rz},
                   1.2f, 1.2f, 20, (Color){col.r/2, col.g/2, col.b/2, 255});
}

static void DrawTrajectory(const Sim *s, int which) {
    /* which: 0=truth, 1=enc, 2=fused */
    if (s->hist_count < 2) return;

    Color col = (which == 0) ? col_truth : (which == 1) ? col_enc : col_fused;
    float thick = (which == 2) ? 0.12f : 0.07f;

    for (int i = 1; i < s->hist_count; i++) {
        int i0 = (s->hist_pos - s->hist_count + i - 1 + HIST_CAP) % HIST_CAP;
        int i1 = (s->hist_pos - s->hist_count + i     + HIST_CAP) % HIST_CAP;
        const Sample *a = &s->history[i0];
        const Sample *b = &s->history[i1];

        V3 pa = (which == 0) ? a->truth : (which == 1) ? a->enc : a->fused;
        V3 pb = (which == 0) ? b->truth : (which == 1) ? b->enc : b->fused;

        Vector3 va = { (float)pa.x, (float)pa.z + 0.15f, (float)pa.y };
        Vector3 vb = { (float)pb.x, (float)pb.z + 0.15f, (float)pb.y };

        DrawLine3D(va, vb, col);
        /* small spheres for better visibility */
        if (i % 4 == 0)
            DrawSphere(va, thick * 0.6f, col);
    }
}

static void DrawHUD(const Sim *s, int paused, int speed, int follow) {
    const char *sc_name[] = {"Nominal", "30% Random Loss", "Occlusion + Bias"};
    char buf[256];

    DrawRectangle(10, 10, 420, 210, (Color){20, 30, 45, 210});
    DrawRectangleLines(10, 10, 420, 210, (Color){80, 120, 160, 255});

    DrawText("TEAM TBJ — Crane 3D Tracking", 25, 22, 20, RAYWHITE);
    DrawText("Synthetic PoC  |  NOT plant data", 25, 46, 14, (Color){160,180,200,255});

    snprintf(buf, sizeof(buf), "Scenario : %s", sc_name[s->scenario]);
    DrawText(buf, 25, 72, 16, col_fused);

    snprintf(buf, sizeof(buf), "Time     : %.1f / 180.0 s", s->time_s);
    DrawText(buf, 25, 94, 16, RAYWHITE);

    snprintf(buf, sizeof(buf), "Encoder RMSE : %.3f m", rmse(s->sq_enc, s->tick));
    DrawText(buf, 25, 118, 16, col_enc);

    snprintf(buf, sizeof(buf), "Hybrid  RMSE : %.3f m", rmse(s->sq_fused, s->tick));
    DrawText(buf, 25, 140, 16, col_fused);

    snprintf(buf, sizeof(buf), "Fixes A/R/M  : %ld / %ld / %ld",
             s->accepted, s->rejected, s->missing);
    DrawText(buf, 25, 162, 15, (Color){200,210,220,255});

    snprintf(buf, sizeof(buf), "Carrying    : %s   Moves: %d",
             s->carrying[0] ? s->carrying : "(none)", s->completed_moves);
    DrawText(buf, 25, 184, 15, (Color){255,200,120,255});

    /* right side controls */
    DrawRectangle(GetScreenWidth() - 280, 10, 270, 150, (Color){20, 30, 45, 210});
    DrawText("Controls", GetScreenWidth() - 265, 22, 18, RAYWHITE);
    DrawText("SPACE  pause/resume", GetScreenWidth() - 265, 50, 14, LIGHTGRAY);
    DrawText("1/2/3  scenario", GetScreenWidth() - 265, 70, 14, LIGHTGRAY);
    DrawText("UP/DN  speed", GetScreenWidth() - 265, 90, 14, LIGHTGRAY);
    DrawText("R      restart", GetScreenWidth() - 265, 110, 14, LIGHTGRAY);
    DrawText("C      camera follow", GetScreenWidth() - 265, 130, 14, LIGHTGRAY);

    /* status line */
    if (paused)
        DrawText("PAUSED", GetScreenWidth()/2 - 50, 20, 28, YELLOW);
    else {
        snprintf(buf, sizeof(buf), "Speed x%d", speed);
        DrawText(buf, GetScreenWidth()/2 - 40, 20, 22, (Color){100,220,160,255});
    }

    /* event banner */
    if (s->latest_event[0]) {
        DrawRectangle(10, GetScreenHeight() - 50, GetScreenWidth() - 20, 40,
                      (Color){30, 40, 55, 220});
        DrawText(s->latest_event, 25, GetScreenHeight() - 38, 18, (Color){255,230,150,255});
    }

    /* legend */
    DrawRectangle(10, 230, 200, 90, (Color){20, 30, 45, 200});
    DrawCircle(30, 250, 7, col_truth);   DrawText("Truth", 48, 243, 14, col_truth);
    DrawCircle(30, 275, 7, col_enc);     DrawText("Encoder only", 48, 268, 14, col_enc);
    DrawCircle(30, 300, 7, col_fused);   DrawText("Hybrid fused", 48, 293, 14, col_fused);
}

int main(int argc, char **argv) {
    Scenario start_sc = NOMINAL;
    int start_speed = 5;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--scenario") && i+1 < argc) {
            int v = atoi(argv[++i]);
            if (v >= 0 && v <= 2) start_sc = (Scenario)v;
        } else if (!strcmp(argv[i], "--speed") && i+1 < argc) {
            int v = atoi(argv[++i]);
            if (v >= 1 && v <= 20) start_speed = v;
        }
    }

    const int W = 1280, H = 720;
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(W, H, "TEAM TBJ — Industrial Crane 3D Tracking (Synthetic)");
    SetTargetFPS(60);

    /* camera */
    Camera3D cam = {0};
    cam.position = (Vector3){30.0f, 28.0f, -25.0f};
    cam.target   = (Vector3){40.0f, 6.0f, 25.0f};
    cam.up       = (Vector3){0.0f, 1.0f, 0.0f};
    cam.fovy     = 55.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    Sim sim;
    sim_init(&sim, start_sc, 2051);

    int paused = 0;
    int speed  = start_speed;
    int follow = 1;
    double last_step = GetTime();

    while (!WindowShouldClose()) {
        /* input */
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_R)) {
            sim_init(&sim, sim.scenario, (uint64_t)(GetTime()*1000));
        }
        if (IsKeyPressed(KEY_ONE))   { sim_init(&sim, NOMINAL, 2051); }
        if (IsKeyPressed(KEY_TWO))   { sim_init(&sim, RANDOM_LOSS, 2051); }
        if (IsKeyPressed(KEY_THREE)) { sim_init(&sim, OCCLUSION_BIAS, 2051); }
        if (IsKeyPressed(KEY_UP))    speed = clampd(speed + 1, 1, 20);
        if (IsKeyPressed(KEY_DOWN))  speed = clampd(speed - 1, 1, 20);
        if (IsKeyPressed(KEY_C))     follow = !follow;
        if (IsKeyPressed(KEY_Q) || IsKeyPressed(KEY_ESCAPE)) break;

        /* advance simulation */
        double now = GetTime();
        if (!paused && sim.time_s < RUN_SECONDS - 1e-6) {
            double elapsed = now - last_step;
            int steps = (int)(elapsed * 10.0 * speed); /* 10 Hz base */
            if (steps > 30) steps = 30;
            for (int i = 0; i < steps; i++) sim_step(&sim);
            if (steps > 0) last_step = now;
        } else {
            last_step = now;
        }

        /* camera follow */
        if (follow) {
            Vector3 target = {
                (float)sim.fused.x,
                (float)sim.fused.z + 4.0f,
                (float)sim.fused.y
            };
            cam.target = Vector3Lerp(cam.target, target, 0.04f);
            Vector3 desired_pos = {
                target.x - 25.0f,
                target.y + 18.0f,
                target.z - 30.0f
            };
            cam.position = Vector3Lerp(cam.position, desired_pos, 0.03f);
        } else {
            /* free orbit with mouse */
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                Vector2 delta = GetMouseDelta();
                /* simple horizontal rotation */
                float yaw = delta.x * 0.005f;
                Vector3 offset = Vector3Subtract(cam.position, cam.target);
                float cosy = cosf(yaw), siny = sinf(yaw);
                float nx = offset.x * cosy - offset.z * siny;
                float nz = offset.x * siny + offset.z * cosy;
                cam.position = (Vector3){cam.target.x + nx, cam.position.y, cam.target.z + nz};
            }
            float wheel = GetMouseWheelMove();
            if (wheel != 0) {
                Vector3 dir = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
                cam.position = Vector3Add(cam.position, Vector3Scale(dir, wheel * 2.5f));
            }
        }

        /* draw */
        BeginDrawing();
        ClearBackground((Color){16, 24, 36, 255});

        BeginMode3D(cam);
            DrawWarehouseFloor();
            DrawTrajectory(&sim, 0);  /* truth */
            DrawTrajectory(&sim, 1);  /* encoder */
            DrawTrajectory(&sim, 2);  /* fused  */

            DrawOverheadCrane(sim.fused, sim.carrying[0] != 0);
            DrawCoilOnFloor(&sim.coils[0]);
            DrawCoilOnFloor(&sim.coils[1]);

            /* small markers at current positions */
            DrawSphere((Vector3){(float)sim.truth.x, (float)sim.truth.z+0.3f, (float)sim.truth.y},
                       0.25f, col_truth);
            DrawSphere((Vector3){(float)sim.enc.x, (float)sim.enc.z+0.3f, (float)sim.enc.y},
                       0.22f, col_enc);
            DrawSphere((Vector3){(float)sim.fused.x, (float)sim.fused.z+0.3f, (float)sim.fused.y},
                       0.28f, col_fused);
        EndMode3D();

        DrawHUD(&sim, paused, speed, follow);
        DrawFPS(GetScreenWidth() - 90, GetScreenHeight() - 30);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
