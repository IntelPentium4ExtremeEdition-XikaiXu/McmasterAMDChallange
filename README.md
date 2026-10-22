# Team TBJ — Crane Position Tracking (C Simulation + Dashboard)

**Client challenge:** ArcelorMittal Dofasco Challenge #1, “Crane Position Tracking System for Steel Coil Visibility”  
**Team:** TBJ · Yuchen Tao · Zihao Zhou · Xikai Xu  
**Implementation:** C11/POSIX simulation and JSON/TCP server; browser dashboard written in plain HTML/CSS/JavaScript for visualization only.  
**Scope:** Research prototype, not certified hardware, crane control software, an operational SCADA connection, or an industrial accuracy validation.

## 1. Quick start (macOS / Linux)

Use a terminal in the **extracted project folder** (the same folder as `Makefile`):

```bash
make
./tbj_crane --serve 8080
```

Open **http://127.0.0.1:8080** in your web browser. The page updates automatically; no internet connection, packages or hosted accounts are required. The simulation runs at **5× speed** initially and lasts 180 simulated seconds; use **Restart** to repeat. It uses synthetic sensors only.

If `make` is unavailable, compile directly:

```bash
cc -O2 -std=c11 -Wall -Wextra -pedantic tbj_crane.c -o tbj_crane -lm -lpthread
cc -O2 -std=c11 -Wall -Wextra -pedantic tbj_scada_client.c -o tbj_scada_client
./tbj_crane --serve 8080
```

On macOS you can install command-line C tooling with `xcode-select --install` if needed. On Linux use GCC or Clang. Windows users can run the project under WSL2 or port the POSIX socket layer to Winsock; a native Windows build is **not** provided. Keep the process running in the terminal while demonstrating the browser interface. Press **Ctrl+C** to stop.

**Troubleshooting:** If port 8080 is occupied, use `./tbj_crane --serve 8081`, then visit `http://127.0.0.1:8081`. If web files cannot load, change directory into the extracted project root and restart the program. The server binds to **loopback only** (`127.0.0.1`), not to public or industrial networks.

## 2. What the simulator demonstrates

| Challenge deliverable | Implemented demonstration / exported evidence | Still required before plant deployment |
|---|---|---|
| XYZ position | Native C mathematical 3D trajectory, estimates and 10 Hz updates | Calibrated real sensor data and surveyed coordinate frame |
| Technology assessment | Dashboard comparison plus `results/technology_comparison.csv` | Real equipment inspection and vendor verification |
| Proposed architecture | Sensor → fusion → read-only HTTP/TCP JSON → visual/CSV | OT-approved connectivity, authentication and hardening |
| Proof-of-concept | Working browser-based 3D visualization + C simulation | Hardware pilot in a representative industrial zone |
| Accuracy/reliability | Encoder and hybrid 3D RMSE, P95, fault scenarios, innovation gating | Truth-instrumented tests, uncertainty budget |
| SCADA query | Versioned API and standalone `tbj_scada_client` in C | Agreed plant SCADA protocol/namespace and cybersecurity |
| Historical travel | Browser traces and `results/live_trace.csv` | Persistent historical data store and retention plan |
| Coil visibility | Two synthetic pick-up and drop-off cycles, updated inventory | Verified coil ID and hook/gripper event feeds; safety review |
| Cost/scalability | CAD component assumptions and one-/five-crane scenarios | Real quotes, installation access survey, detailed BOM |
| Risks/deployment plan | `results/risks.csv`, requirements matrix and 8-week plan | Pilot acceptance gates and approved operational procedures |
| 24/7 aspiration | Uninterrupted estimator fast-forward command | Actual soak/uptime testing under continuous live traffic and sensor hardware |

The Challenge explicitly accepts analysis, simulation, prototype or proof-of-concept development rather than requiring a production-ready system.

## 3. Interactive live controls

**Live tracking** contains: three XYZ cards, instantaneous validity, a 3D *isometric projection* (not a CAD model), moving reference and estimated traces, error graph, RMSE/P95, absolute-fix acceptance statistics, and simulated coil inventory.

**Scenario menu:**

- **Nominal:** baseline velocity sensor noise and UWB-like absolute XYZ fixes.
- **Random loss:** approximately 30% of attempted absolute position fixes are missing.
- **Occlusion + bias:** 80–110 s loss of all absolute fixes plus 110–137 s position bias, with background losses. Outlier rejection may reject biased fixes, but **it is not guaranteed to reject gradual or correlated error**.

**Profile menu:**

- **Warehouse route:** 0–180 s crane waypoints, plus emulated events for `A1058` and `B2062` (at 26, 77, 111 and 156 s). Good for a visual demonstration of operations.
- **Report trajectory:** the previous TBJ Python report's smooth sinusoidal 3D reference model, with no coil actions. Good for aligning mathematical assumptions with the written report.

Changing a scenario or profile resets the seed and simulation, and **overwrites `results/live_trace.csv` and `results/coil_events.csv`**. Copy those files first if you want to preserve a run. The browser polls at approximately 600 ms. A 0.5 s history sample interval is used for display/CSV; the core simulation updates every 0.1 s.

## 4. Reproducible research runs (no browser required)

### Monte Carlo experiment — three scenarios × 120 trials

```bash
./tbj_crane --benchmark 120
```

Outputs:

- `results/benchmark_runs.csv` — 360 individual experiments (scenario, seed, RMSE/P95, accepted/rejected/missed fixes).
- `results/benchmark_summary.csv` — scenario-level **means of run-level metrics**; not pooled sample P95.
- `results/benchmark_comparison.svg` — self-contained vector chart generated by **C** (usable directly in presentation slides).

The experiment uses run seeds 2026–2145, `dt=0.1 s`, total simulated time 180 s, encoder-derived velocity noise `(0.07,0.07,0.05) m/s`, mean velocity bias `(+0.006,-0.004,+0.002) m/s` plus run variability, temporary disturbance at 65–87 s, 2 Hz absolute XYZ position fixes with sigma `(0.15,0.15,0.10) m`, and a 3D normalized innovation gate at `d²=16.27`.

The earlier **Python** technical-report study and this **C** implementation do not produce exactly the same numerical table: the RNG families differ, the C model calculates analytic trajectory derivatives, and C metrics evaluate time points 0.1–180.0 s (1,800 samples), while the earlier Python report includes time zero (1,801 samples). They share the same conceptual experiment and parameter scale. **Regenerate statistics for a final submission if you want to report C results.** Do not mix tables from the two runs as though they are identical measurements.

### One full experiment and coil event log

```bash
./tbj_crane --run 2 0
```

Mode is `0`, `1`, or `2`; profile is `0` (warehouse) or `1` (report trajectory). Creates `results/live_trace.csv` and `results/coil_events.csv`, plus two charts generated entirely by **C**: `results/error_over_time.svg` and `results/warehouse_trajectory.svg`. The reference-truth, encoder estimate and fused estimate are stored in meters, using an illustrative local coordinate frame.

### Simulated-time software endurance exercise

```bash
./tbj_crane --soak 86400
```

This continuously executes 864,000 **C estimation steps** (24 hours of mathematical time at 10 Hz) without intermediate reinitialization, measuring computational execution and estimator continuity. **It does not run the server continuously for 24 wall-clock hours, demonstrate physical uptime, or validate maintenance-free operation.**

### Local SCADA-style query from C

Leave the server running in terminal 1, then in terminal 2:

```bash
./tbj_scada_client 8080
```

Or use `curl http://127.0.0.1:8080/api/v1/cranes/TBJ-01/position`.

| Endpoint | HTTP method | Purpose |
|---|---|---|
| `/api/v1/cranes/TBJ-01/position` | `GET` | Latest crane coordinates, quality, simulated coil inventory |
| `/api/state` | `GET` | Same latest simulated state for the dashboard |
| `/api/history` | `GET` | Historical trajectory and error-array samples |
| `/api/metrics` | `GET` | RMSE, P95, missing/rejected/accepted fixes |
| `/api/control?scenario=2` | `POST` | Demo-only fault scenario reset (not crane control) |
| `/api/control?profile=1` | `POST` | Demo-only trajectory selection |
| `/api/control?restart=1` | `POST` | Restart simulation |
| `/api/control?pause=1` | `POST` | Pause simulated time |
| `/api/control?pause=0` | `POST` | Resume simulated time |
| `/api/control?speed=5` | `POST` | Choose playback multiplier (1–20) |

`/api/state` returns illustrative JSON along these lines:

```json
{
  "crane_id":"TBJ-01",
  "coordinate_frame":"TBJ_WAREHOUSE_LOCAL_METRES",
  "scenario":"nominal",
  "time_s":39.5,
  "status":"VALID_SIMULATED",
  "units":"m",
  "truth_xyz":[17.0,13.0,12.0],
  "encoder_xyz":[17.1,12.9,12.0],
  "fused_xyz":[17.0,13.1,12.0]
}
```

The example above is illustrative rather than a recorded output. The actual endpoint provides more fields; `truth_xyz` and error metrics are **simulation-only evaluation data** and would not normally be available to a production SCADA consumer. The API is read-only for telemetry, but demonstration control endpoints are available locally. It does **not** implement OPC UA, authentication, TLS, industrial certificate handling, real SCADA subscriptions or real timestamps.

## 5. Estimation model

At each 0.1-second step:

1. Generate a reference position and velocity from the motion model.
2. Simulate noisy/bias-affected encoder-derived velocities and integrate displacement.
3. Predict each axis of the fused position using velocity and diagonal process uncertainty.
4. Every fifth step, create an optional UWB-like absolute XYZ measurement. The simulation **does not** implement anchor geometry, time-of-flight propagation, or multilateration.
5. Apply a **joint 3-axis** innovation test, `sum((z_i - pred_i)²/(P_i+R_i)) < 16.27`.
6. For an accepted fix, apply a diagonal Kalman correction, `K_i = P_i/(P_i+R_i)`, and Joseph-form diagonal covariance update.
7. Compare encoder-only and fused estimates with the synthetic reference and update RMSE/P95 plus health/freshness information.

A missing absolute fix does not mean the encoder-derived output stops. The simulated system emits a `DEGRADED_STALE` label after **more than 1.0 s** without an accepted absolute correction; this is a proposed conservative monitoring rule, not a plant-approved safety threshold. It is intentionally distinct from proof of actual sensor validity.

**Limitations:** Absolute measurements are Gaussian noisy positions with manually injected bias. Real UWB propagation, variable geometry, cross-axis covariance, transient dynamics, hook sway, encoder quantization/calibration, data timestamp uncertainty, communications latency and industrial cybersecurity are out of scope. A synthetic acceptance rate is not system availability.

## 6. Risks, costs and deliverables

The Engineering and Validation tabs contain the core non-quantitative deliverables. The program generates four editable evidence files each time it runs:

- `results/technology_comparison.csv`
- `results/cost_estimates_CAD.csv`
- `results/risks.csv`
- `results/requirements_matrix.csv`

Assumed economic scenarios mirror the earlier TBJ draft report: **CAD 10,065** for an encoder-led baseline per crane; **CAD 25,225** for a one-crane hybrid pilot; and **CAD 72,925** for a five-crane / one-zone hybrid illustration. These numbers are team-created *budget assumptions* and exclude important unmeasured plant requirements; **they are not supplier quotes or an AMD commitment**.

## 7. Tests and expected outputs

```bash
make test
```

The smoke tests use C and a small Python script **only to assert results and exercise the HTTP endpoint**. Python is **not** used by the simulation, estimator, or dashboard server. The tests cover: three scenario results, hybrid vs encoder performance, four coil events, a short soak, JSON history/state/metrics, scenario selection and the native C SCADA client.

### Example presentation flow (4–6 minutes)

1. **Problem (30 s):** Crane movement creates inconsistent inventory location records.
2. **Architecture (45 s):** Navigate to *Engineering assessment*; explain why existing feedback should be surveyed before adding UWB anchors.
3. **Nominal demo (90 s):** Use warehouse route at 5× speed. Show fused XYZ, 3D movement and actual coil-transfer log.
4. **Industrial stress (60 s):** Select scenario 3; show stale-fix status, error increase and limitations of outlier gating.
5. **Integration (30 s):** Run `./tbj_scada_client 8080` in a second terminal and show the returned position JSON.
6. **Validation & conclusion (45 s):** Point to batch CSV statistics, risk/cost analysis, and field-pilot decision gates.

**Important:** In a demonstration, do not claim the synthetic system has been tested at ArcelorMittal Dofasco, will satisfy their actual accuracy target, or can safely control industrial lifting equipment.

## 8. Suggested team split

- **Yuchen Tao** — reference trajectories, C position estimation, RMSE/P95 and Monte Carlo verification.
- **Zihao Zhou** — encoder/UWB/LiDAR trade study, estimated cost, reliability/risk assessment.
- **Xikai Xu** — local HTTP/TCP API, native C SCADA client, dashboard, CSV data, and integration verification.

This is a **suggested assignment**, not a claim that members completed these tasks individually.

## 9. Folder structure

```text
TBJ_Crane_C_Simulation/
├── tbj_crane.c                # Full C simulation, estimator, events, batch analysis, API server
├── tbj_scada_client.c         # Independent read-only HTTP/TCP client in C
├── Makefile                   # Build and tests
├── web/
│   ├── index.html             # Visual interface
│   ├── style.css              # Styling
│   └── app.js                 # Browser rendering/polling (no physics)
├── results/
│   ├── benchmark_runs.csv
│   ├── benchmark_summary.csv
│   ├── benchmark_comparison.svg
│   ├── error_over_time.svg
│   ├── warehouse_trajectory.svg
│   ├── live_trace.csv
│   ├── coil_events.csv
│   ├── cost_estimates_CAD.csv
│   ├── technology_comparison.csv
│   ├── requirements_matrix.csv
│   └── risks.csv
├── tests/
│   └── smoke_test.sh
└── README.md
```

**Document basis:** the two-page ArcelorMittal Dofasco Engineering Fall Mini Challenge — Challenge #1 problem statement supplied by the team, and the earlier Team TBJ technical report. All facility dimensions, experimental assumptions, numeric provisional gates and costs are created for demonstration rather than supplied as plant acceptance criteria.
