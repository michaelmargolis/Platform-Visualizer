# MDX Platform Twin - Final-Year Project Ideas

**Audience:** Design Engineering, mechatronics, robotics, embedded-systems and software students  
**Based on:** the current v56 MDX Platform Twin and the motion-platform instrumentation handover (27 September 2026)  
**Companion guide:** [Architecture and Maintainer Guide](architecture.md)

## Why this is an interesting project platform

The MDX system is an **inverted six-degree-of-freedom Stewart platform**, suspended using six pneumatic muscles. The Platform Operators Console (POC) processes simulator motion and commands muscle lengths and pressures. MDX Platform Twin renders that motion using either POC's **commanded muscle lengths** or an **empirical pressure-response estimate**, and can overlay POC's applied transform using **Pose Diff** when the platform is unparked.

Early comparison has found that the **Lengths representation closely tracks the POC applied transform**, while Pressure mode shows larger, still-useful differences. That makes the Twin a starting point for real experiments rather than a finished description of physical performance.

**Essential distinction:** the current Twin receives **commands, not sensor readings**. A calculated platform pose is not a measurement of the real mechanism. Projects that add reliable physical evidence could improve the Twin, and potentially support future control research.

A strong final-year project should ask one testable question, build a prototype or research tool, and produce repeatable evaluation data. It need not redesign the complete visualizer or platform controller.

## Choosing a project

| Project | Primary discipline | Additional hardware | Suggested challenge |
| --- | --- | --- | --- |
| 1. Optical 6-DOF measurement | Computer vision / mechatronics | Camera, markers | Substantial |
| 2. Direct muscle displacement sensor | Mechanical / embedded | Custom sensor prototype | Substantial |
| 3. Camera + IMU sensor fusion | Sensing / signal processing | Camera, IMU | Advanced |
| 4. Pneumatic response identification | Modelling / data analysis | Recorded physical-response data | Substantial |
| 5. Load-aware pressure model | Software / modelling | None initially | Moderate |
| 6. Replay and comparison workbench | Software / data visualization | None | Moderate |
| 7. Geometry calibration and uncertainty | CAD / kinematics / metrology | Optional reference fixture | Moderate |
| 8. Network and solver performance study | Embedded / software | Windows/Pi hardware | Moderate |
| 9. Repeatable platform test protocol | Test engineering | Access to supervised platform tests | Substantial |
| 10. Educational 6-DOF explorer | Visualization / HCI | None | Moderate |
| 11. Hardware-in-the-loop test emulator | Embedded / verification | Optional Pi/PLC test equipment | Substantial |
| 12. Closed-loop control feasibility | Controls / research | Recorded data; no live control initially | Advanced |

The challenge levels are qualitative; a supervisor should adapt each scope to the student's experience, resources and available platform access. Several projects can share a single **well-documented command-versus-measurement dataset**.

---

## Measurement and instrumentation

### 1. Low-cost optical 6-DOF platform tracker

**Question:** How accurately and quickly can a fixed camera recover the physical platform pose?

Build a system using a rigid AprilTag or ArUco/ChArUco board on the moving platform. Calibrate camera intrinsics, marker geometry and the transform between camera and POC coordinates. Estimate `[x, y, z, roll, pitch, yaw]` and use the known Stewart attachment coordinates to derive the six **pose-derived** actuator lengths.

**Deliverables:** Calibrated capture application, repeatable pose dataset, live or recorded visualization, error/latency report and practical mounting design. **Evaluation:** Static repeatability, pose error against reference measurements, frame rate, end-to-end latency, coverage and failure/occlusion rate.

**Scope control:** Start with stationary and slow controlled tests. A single fixed camera plus one rigid tag board is a suitable first milestone. A 60 fps or higher global-shutter camera is worth considering for dynamics, but students should justify their choice experimentally.

### 2. Portable pneumatic-muscle displacement sensor

**Question:** Can one muscle's extension be measured continuously using a low-cost, non-intrusive instrument?

Prototype a draw-wire encoder, cable potentiometer or another displacement sensor mounted approximately along one muscle. Focus on mechanical mounting, sensor resolution, tension, calibration and protection against snagging or interference.

**Deliverables:** Working one-channel instrument, calibration rig, embedded logger with timestamps, safety/mounting assessment and measured displacement traces. **Evaluation:** Resolution, hysteresis, repeatability, sample rate, noise and agreement with a reference displacement.

**Scope control:** Instrument **one** muscle first. Extending to six channels is a possible follow-up project, not an initial requirement. Direct measured length and camera pose-derived length must be labelled differently.

### 3. Vision and IMU sensor fusion

**Question:** Does combining absolute camera pose with high-rate inertial data improve estimates of velocity, acceleration and short-term motion?

Combine a calibrated moving-platform IMU with a camera pose stream. Compare a simple complementary filter with a more sophisticated estimator if time permits. Pay particular attention to clock alignment, sensor bias, coordinate transforms and data dropout.

**Deliverables:** Synchronized sensor logger, fusion algorithm, uncertainty plots and comparative results. **Evaluation:** Delay, short-term velocity accuracy, drift, robustness during partial tag occlusion and improvement versus each sensor alone.

**Scope control:** An IMU alone is **not** a reliable absolute-position or muscle-length sensor: double integration of acceleration drifts. A static rig followed by safely limited motion is a more credible progression than immediately attempting a full ride.

## Modelling and data analysis

### 4. Identify a data-driven pneumatic response model

**Question:** What simple model predicts physical response better than the current pressure-table-plus-slew approximation?

Use synchronized POC commands and independently measured physical pose/lengths to estimate command-to-motion delay, inflation/exhaust asymmetry, rate limits, hysteresis and load dependence. Compare progressively richer models: constant asymmetric slew, pressure-dependent slew, first-order fill/exhaust dynamics, then a lightweight identified dynamic model.

**Deliverables:** Reproducible notebook or Python package, fitted models, train/test datasets and a documented improvement to `pressure_model.py` if justified. **Evaluation:** Length and pose prediction errors, timing/phase errors on held-out experiments, model complexity and Pi runtime cost.

**Scope control:** Preserve the original simple model as a baseline. Do not claim physically measured accuracy from POC command records alone.

### 5. Load-aware muscle prediction

**Question:** Would selecting or interpolating the pressure model by platform load improve predictions?

The current code supports load-dependent CSV curves, but its runtime pressure estimate uses the configured `pressure_model.empirical_load`. Investigate how reliable POC's transmitted load metadata is, how it maps to the empirical CSV load definitions, and whether measured response improves when that information is used.

**Deliverables:** Load-selection policy, annotated comparison datasets, validation plots and a backward-compatible model switch. **Evaluation:** Errors across several supported loads, stability when load metadata is missing, and the absence of discontinuities when load changes.

**Scope control:** An offline recorded-data study can be completed before modifying any live code. Distinguish a commanded/configured load value from an independent load measurement.

### 6. Replay, plots and fault-analysis workbench

**Question:** Can recorded sessions make it easier to diagnose model mismatch than watching live motion alone?

Create a log/replay tool that reads recorded `POC_TWIN` packets or CSV datasets and supports pause, single-step, speed control and synchronized graphs. Plot applied real transform, commanded lengths, predicted lengths, UDP timing and Pose Diff model differences. Optionally add externally measured pose when available.

**Deliverables:** Offline replay file format, annotated viewer or notebook, example sessions and regression tests using deterministic input. **Evaluation:** Reproducible playback, accurate timestamp handling, missing-packet behaviour and ability to identify known anomalies.

**Scope control:** Begin with **offline playback**, avoiding changes to the real-time receiver. A future version might share a pure packet-to-pose function with the live visualizer.

### 7. CAD geometry, calibration and uncertainty

**Question:** Which attachment-coordinate, orientation or clearance errors most affect reconstructed pose?

Compare POC geometry, `source.json`, compiled `visual.json` and the rendered deck/attachment geometry. Use controlled synthetic offsets and, if available, measured reference poses to quantify errors in platform position and orientation. Consider uncertainty in fixed hardware lengths and joint-centre definition.

**Deliverables:** Geometry validation utility, sensitivity study, before/after coordinate validation and instructions for regenerating visual artifacts. **Evaluation:** Reconstruction sensitivity (mm or degrees per mm of geometry error), detectability of mismatches and verified parked/neutral reference positions.

**Scope control:** Do not silently change POC kinematics to make the visualization look better. The existing MDX CAD-derived parked pose and 17 mm clevis representation offer useful initial test cases.

## Software, visualization and validation

### 8. Raspberry Pi network and solver performance

**Question:** What limits the Twin's update rate and how can it preserve useful motion information on low-cost hardware?

Measure input rate, receiver packet count, superseded packet count, solver iterations/time, render FPS and age of the latest input. Test realistic packet bursts, missing packets and different model complexities on Windows and a Pi. Evaluate targeted optimizations without changing physical modelling semantics.

**Deliverables:** Benchmark harness, plots and profiling results, a small optimized patch if justified, and a documented performance budget. **Evaluation:** CPU load, motion/update latency, packet drops, pose error and reproducibility across machines.

**Scope control:** The existing receiver intentionally keeps only the latest snapshot. A higher FPS display is not automatically a higher-fidelity measurement; distinguish source packet loss from render-side packet skipping.

### 9. Repeatable motion-platform validation protocol

**Question:** What controlled experiments give a defensible measure of platform and Twin performance?

Develop standard tests: static reference poses, slow single-axis sine waves, safe ramps/steps, stepped frequency sweeps, combined-axis motion and representative simulator segments. Repeat at relevant supported loads. Design a logging format, calibration procedure and automated report generator.

**Deliverables:** Test protocol, safety/checklist template, analysis scripts and a report with uncertainty and pass/fail or target metrics. **Evaluation:** Repeatability, ease of use, confidence intervals, measured bandwidth, delay, steady-state errors and cross-axis coupling.

**Scope control:** Use the physical platform only with competent supervision and its established safety systems. The first instrumentation is **observation-only**; do not integrate new automatic actuation into these tests.

### 10. Interactive educational Stewart-platform explorer

**Question:** Can the Twin teach students how six muscle lengths produce combined translation and rotation?

Build an optional instructional view with selectable axes, numerical pose readouts, labelled joint centres and a visualization of how each muscle length changes. Contrast commanded, model-predicted and (only if separately instrumented) measured motion, using accurate terms and units.

**Deliverables:** A small, maintainable UI extension, exercises for a laboratory session and formative user testing. **Evaluation:** Usability on 800 x 480 touchscreens, frame rate, student comprehension and correct interpretation of measurements versus estimates.

**Scope control:** Preserve the current `visualizer_ui.py` presentation boundary and `platform_visualizer.py` runtime ownership. Avoid a second GUI framework or a full application redesign.

### 11. Hardware-in-the-loop test emulator

**Question:** Can a low-cost Pi-based test station exercise POC's output protocol without requiring the physical Stewart platform?

Extend the existing receive-only Twin as a **test instrument** or build a closely related isolated tool for protocol verification. Generate repeatable simulated stimuli, classify malformed and valid messages, display timing and response status, and save test results. Consider how a controlled acknowledgement emulator would be isolated from the real Festo system.

**Deliverables:** Test harness, network topology, protocol test cases, reproducible fixtures and a clear account of what is simulated. **Evaluation:** Compatibility with known POC output sequences, packet robustness, fault detection and isolation from physical hardware.

**Scope control:** Do **not** turn the current read-only EasyIP receiver into a live controller as a side effect. Any protocol responses should run in a separate explicitly selected laboratory mode and must not masquerade as actual actuator feedback.

## Advanced research

### 12. Closed-loop control feasibility using measured pose

**Question:** Would sufficiently accurate low-latency pose feedback enable better tracking than the current command-driven approach?

Start with recorded command/measurement datasets and simulation. Estimate achievable controller bandwidth given measurement delays, dropped observations and actuator nonlinearity. Compare offline feed-forward correction, conservative feedback and a simple model-based approach. Identify failure modes and minimum instrumentation requirements.

**Deliverables:** Simulated control architecture, stability and latency analysis, offline comparisons and a safety case for any proposed later hardware experiments. **Evaluation:** Predicted reduction in tracking error, stability margins, robustness to sensor failure and practical sampling/latency needs.

**Scope control:** **No unsupervised physical closed-loop control.** Live control changes require separate engineering authorization, independent limits, emergency-stop considerations and a formal test plan. It is entirely valid for a final-year project to finish at demonstrated simulation feasibility.

---

## Shared data and evaluation conventions

Projects become more useful when they agree on naming, units and timestamps. A possible shared record is:

| Category | Suggested fields | Meaning |
| --- | --- | --- |
| Time | Monotonic host time, source timestamp, frame number, POC sequence | Enables latency/alignment analysis |
| Commands | `applied_transform_real`, six commanded lengths, six commanded pressures | What POC requested/applied computationally |
| System state | Platform profile, parking state, software mode, configured load | Interprets whether a sample is comparable |
| Independent observations | Camera/IMU pose, reference measurement, measurement validity/confidence | What sensors *observed* |
| Derived observations | Six pose-derived lengths, estimated velocity/acceleration | Calculations from measurements |
| Model outputs | Twin predicted pose/lengths, solver residual and timings | What the model *predicted* |

Preserve raw observations and raw packet timestamps. Store smoothing/filtering outputs separately. Record camera calibration, sensor placement, model version, load, coordinate transform and configuration with each experiment. A calibrated coordinate frame is essential: POC pose uses **millimetres and radians**, while `transform` from legacy Twin packets is **normalized**.

**Useful metrics** include static and dynamic pose error, per-muscle length error, command-to-motion delay, phase lag and gain versus frequency, peak/rms velocity, inflation versus exhaust asymmetry, skipped packet counts, computation time, uncertainty and repeatability. Identify an independent reference for every accuracy claim.

## Suggested experiment sequence

1. **Validate offline first.** Run Demo, use synthetic/recorded Twin packets, check units, park/unpark semantics and solver results.
2. **Calibrate sensors at rest.** Establish coordinate transforms, reference lengths, timestamps and uncertainty.
3. **Try slow controlled single-axis motion.** Compare commanded and observed displacement without challenging platform limits.
4. **Advance to supervised ramps, steps and frequency tests.** Collect repeat trials and data at approved loads.
5. **Reserve combined-axis/ride trials for the end.** Hold back data for unbiased validation of models developed earlier.

## Collaboration and handoff expectations

Students can work independently if they agree on shared file formats and platform test procedures. A measurement team might deliver validated pose/length data; a modelling team can then improve `pressure_model.py`; and an HCI/replay team can present the results without editing the same core logic.

A good project handoff includes a short research question, baseline version, bill of materials (where applicable), setup/calibration guide, safe test plan, source code with tests, example data, reproducible results, known limitations and suggested next experiments. Prefer a focused contribution with measured evidence over a large unvalidated rewrite.

## Useful starting points in this repository

- [Architecture and Maintainer Guide](architecture.md) - modules, UDP data flow, data ownership and extension practices.
- [`README.md`](../README.md) - installation, controls and runtime modes.
- `../pressure_model.py` and `../pressure_data/` - current pressure-response assumptions and CSV data.
- `../forward_solver.py` and `../poc_platform.py` - pose reconstruction and geometry handling.
- `../twin_udp.py`, `../easyip_receiver.py` and `../udp_diagnostics.py` - communication and timing instrumentation.
- `../tests/` - examples of regression tests for geometry, packet validation and interaction.
- `../PLATFORM_BEHAVIOUR_INSTRUMENTATION_HANDOVER_v1.md` - camera/AprilTag measurement plan, alternative sensors and staged physical validation ideas.

**Safety boundary:** The Twin is a visualization and modelling application, not a certified safety system. Never treat its predicted pose, packet reception or simulated parking position as evidence that the real platform is safe to approach. Hardware experiments require supervised procedures, mechanical safeguards and independent emergency-stop systems.
