# TBJ — Five-Minute Demonstration Card

**Start:** Open Terminal in the project folder. Run `make && ./tbj_crane --serve 8080`. Browser: http://127.0.0.1:8080.

**1 | Challenge:** "We are building a proof of concept for continuous overhead-crane XYZ positioning to support future steel-coil tracking."

**2 | Architecture:** Open *Engineering assessment*. Explain **encoder motion + independently referenced absolute fixes + outlier gate + TCP/IP telemetry**. Clarify that the project uses simulated measurements.

**3 | Live tracking:** Go to *Live tracking*. Choose scenario **S1**, profile **Warehouse route**, playback **5×**, click **Restart**. Show real-time XYZ, three trajectories and rising history plot.

**4 | Coil workflow:** At simulated time **26 s**, A1058 is picked up; **77 s**, dropped at a new position. At **111 s** B2062 is picked up; **156 s**, dropped. The inventory updates use **emulated external identity/events**; crane coordinates alone do not establish coil identity.

**5 | Stress test:** Select **S3**. At simulated time **80–110 s**, absolute fixes are unavailable; from **110–137 s**, biased fixes are injected. Watch the amber stale-reference indicator and changing error chart. Do not claim industrial field accuracy.

**6 | SCADA query:** In another Terminal from the folder run `./tbj_scada_client 8080` (or `curl http://127.0.0.1:8080/api/v1/cranes/TBJ-01/position`). Highlight XYZ, units, status, coordinate frame and quality metadata.

**7 | Quantitative evidence:** Run `./tbj_crane --benchmark 120` **after stopping the server** or in a separate copied project directory, to avoid writing to the same result files. Show `results/benchmark_summary.csv` (3 scenarios and 120 trials). Alternatively use the CSV results already included in the archive.

**8 | Recommendation:** Present cost and risks in *Engineering assessment*, then conclude: confirm existing PLC/encoder access first, establish required precision, survey the metal-rich space, and pilot additional absolute references only where needed.

*No physical crane, measured UWB range, actual SCADA link, or plant safety functions are implemented.*
