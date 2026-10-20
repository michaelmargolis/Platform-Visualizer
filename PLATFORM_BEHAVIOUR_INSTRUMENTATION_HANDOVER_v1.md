<!-- PLATFORM_BEHAVIOUR_INSTRUMENTATION_HANDOVER_v1.md ver 2 -->
# Motion-platform instrumentation handover

Prepared 27 September 2026

## 1. Purpose

Start a new development/research thread focused on measuring the **real-time physical response** of the suspended six-muscle motion platform and comparing it with the Platform Operators Console (POC) command stream.

The immediate motivation is that the Panda3D pressure-driven digital twin estimates muscle contraction from commanded pressure and an empirical pressure/contraction table. It currently applies configurable contraction/extension slew limits (initially 100 mm/s). Dynamic POC scenes commonly request sizeable changes, and the software diagnostics have shown processed length changes up to about 15 mm. Before tuning the digital twin or platform controller, obtain measurements of what the real platform actually does.

The most valuable dataset is a synchronized record of:

```text
POC commanded transform
POC requested muscle lengths
POC commanded pressures
                 +
measured physical platform pose
                 -> derived physical muscle lengths / velocities / accelerations
```

This can support two related goals:

1. improve the digital twin so its pressure-derived motion resembles the real platform; and
2. quantify command-to-motion error/delay and investigate improvements to platform control.

The instrumentation itself, alternative sensing methods, calibration, identification and control use are also suitable scopes for several final-year Design Engineering student projects.

## 2. Existing platform/software context

- Platform is an inverted/suspended Stewart arrangement with six Festo pneumatic muscles.
- The current MDX suspended-platform configuration uses six nominal 1000 mm muscles plus fixed attachment hardware. The separate suspended-chair configurations use 800 mm muscles and are not the geometry assumed by this handover.
- POC calculates a six-DOF transform, inverse kinematics, requested muscle lengths and command pressures.
- POC publishes a one-way `POC_TWIN` UDP snapshot stream at up to 20 Hz (50 ms cycle).
- Useful fields include `sequence`, `timestamp_ms`, processed transform, six `muscle_length_mm`, six `command_pressure_mbar`, load and state information.
- The Panda3D visualizer can currently operate from either the POC length fields or the pressure fields.
- Pressure mode uses empirical pressure-to-contraction curves, rising/falling branch state, load interpolation and separate contraction/extension slew limits before a bounded forward-pose solve.
- Visualizer v20 adds packet/solver diagnostics specifically to determine whether apparent 10-15 mm steps are genuine consecutive 50 ms steps or are caused by skipped/superseded packets or longer processing intervals.

The POC source/configuration supplied to the new thread should be treated as authoritative for current software behavior. Physical measurements are authoritative for actual platform response.

## 3. Preferred first measurement approach: fixed camera + AprilTag board

A promising low-cost approach is a rigid AprilTag target/board fixed to the moving platform or chair structure and observed by a stationary calibrated camera.

Concept:

```text
fixed calibrated camera
        |
        v
AprilTag board rigidly fixed to moving platform
        |
        v
camera pose estimator -> measured platform x,y,z,roll,pitch,yaw
        |
        +-> known Stewart geometry -> six derived actuator/muscle lengths
```

Why this is attractive:

- no sensor has to be fitted along each pneumatic muscle;
- one measurement estimates the complete rigid-platform 6-DOF pose;
- the six physical muscle lengths can then be calculated from known attachment coordinates;
- it can be implemented with inexpensive cameras and open-source computer-vision software;
- it provides a direct commanded-pose versus actual-pose dataset useful for both control and twin validation.

Important design questions for the new thread:

- camera choice and frame rate (30 fps may characterize slow response; 60 fps or higher is preferable for dynamics);
- global-shutter versus rolling-shutter camera;
- lens field of view and working distance;
- target/board physical size and location;
- whether one large tag, an AprilTag grid/board, or ArUco/ChArUco board gives the best robustness/accuracy;
- camera intrinsic calibration and distortion correction;
- defining the transformation between camera coordinates, target coordinates and the POC platform coordinate system;
- timestamping and synchronization with POC UDP packets;
- behavior when tags are partly occluded or viewed at steep angles;
- measurement uncertainty and repeatability over the full platform workspace.

A multi-tag rigid board is likely preferable to a single small tag because it gives more image features, redundancy and better pose estimation over changing viewing angles.

## 4. Proposed logger output

Aim for one synchronized CSV (or equivalent binary + processed CSV) containing at least:

```text
time_local_monotonic
camera_frame_number
camera_timestamp
vision_valid
vision_reprojection_error

measured_x_mm
measured_y_mm
measured_z_mm
measured_roll_rad
measured_pitch_rad
measured_yaw_rad

measured_length_0_mm ... measured_length_5_mm
measured_length_velocity_0_mm_s ... measured_length_velocity_5_mm_s

POC_sequence
POC_timestamp_ms
POC_platform_state
POC_simulator_state
POC_transform_x ... yaw
POC_command_length_0_mm ... 5
POC_pressure_0_mbar ... 5
POC_load
```

Keep the **raw camera-derived pose** as well as any filtered version. Do not overwrite raw data with smoothing. Record enough timing information to estimate camera/POC latency during analysis.

If acceleration or an IMU is added, log raw accelerometer and gyro channels with their own timestamps.

## 5. Deriving actual muscle lengths from measured pose

Once the transformation between the measured target pose and the POC moving-platform coordinate system is calibrated, use the same attachment geometry as the POC kinematics.

For each channel `i`:

```text
fixed attachment Bi
moving local attachment Pi
measured platform rotation R and translation t
world moving attachment = R Pi + t
actuator distance = |Bi - (R Pi + t)|
physical active muscle length = actuator distance - fixed hardware length
```

This provides six inferred physical lengths from one rigid-body pose measurement. The result should be called **pose-derived length**, not direct muscle-length measurement. Its accuracy depends on camera pose accuracy and the attachment geometry/calibration.

## 6. Alternative sensing approaches worth comparing

### A. Draw-wire/string potentiometer or cable encoder

Mount a displacement sensor approximately parallel to a muscle/actuator path.

Advantages:
- direct displacement measurement;
- high update rate is readily achievable;
- simple signal interpretation.

Limitations:
- six channels are expensive/complex if all muscles are instrumented simultaneously;
- mounting must not load, snag or interfere with the platform;
- cable path must track actuator geometry accurately.

A useful student project could build **one portable channel** and move it between muscles for controlled characterization tests before considering six permanent sensors.

### B. Linear optical/magnetic encoders or custom cable sensor

A purpose-designed low-cost extension sensor could use an encoder, potentiometer, Hall sensing or optical spool. This is a possible mechanical/electronic design project in itself.

### C. IMU on the moving platform

An IMU can provide acceleration and angular velocity at high sample rate.

Useful for:
- response onset/delay;
- vibration/resonance;
- angular-rate dynamics;
- acceleration envelopes;
- validating derivatives of optical pose.

Less useful for:
- absolute position or actuator length from acceleration alone; double integration drifts rapidly.

The strongest low-cost combination may be **camera pose + IMU**, with camera providing low-frequency/absolute pose and IMU providing higher-frequency motion information.

### D. Laser/ToF distance sensing

One or more fixed distance sensors could measure selected platform distances.

Advantages:
- inexpensive and simple for one axis/distance;
- potentially high rate.

Limitations:
- target angle changes;
- occlusion;
- one range does not provide six-DOF pose;
- multiple sensors need geometric calibration.

Could be useful as an independent validation channel rather than the primary system.

### E. Stereo/depth camera

A stereo or depth camera can estimate 3D features without relying solely on planar-tag pose.

Possible benefits:
- richer geometry;
- less dependence on a single marker plane.

Trade-offs:
- higher cost/computation;
- depth accuracy may be poorer than a well-calibrated large fiducial at the required working distance;
- synchronization and calibration are more complex.

### F. External motion capture

If university facilities are available, an optical motion-capture system can provide a high-quality reference dataset. It would be valuable as ground truth for validating a low-cost AprilTag solution, even if it is not the intended permanent instrumentation.

### G. Smartphone/high-speed video

A useful very-low-cost proof of concept. A phone at 60/120/240 fps can record markers/rulers for offline analysis. It is less convenient for synchronized continuous telemetry but could quickly establish approximate response speed and delay before building the live system.

## 7. Suggested staged experiments

Do not begin with a full ride. Use controlled repeatable inputs and appropriate platform safety procedures.

Suggested progression:

1. **Static calibration**
   - several known platform poses;
   - verify coordinate signs/scales;
   - quantify pose repeatability and derived-length errors.

2. **Single-axis slow sine**
   - heave first, then roll/pitch/surge/sway/yaw;
   - compare commanded and measured amplitude/phase.

3. **Controlled ramps/steps within safe limits**
   - estimate response delay, maximum velocity and settling behavior;
   - investigate inflation versus exhaust asymmetry.

4. **Frequency sweep / stepped sine frequencies**
   - identify bandwidth, gain attenuation and phase lag versus frequency.

5. **Combined-axis motions**
   - identify coupling and whether one-axis calibration remains valid.

6. **Representative simulator sequence**
   - validate findings under realistic dynamic commands.

Repeat key tests at different supported loads because pneumatic-muscle behavior is load dependent.

## 8. Analyses that would improve the digital twin

From synchronized commanded/actual data estimate:

- command-to-motion delay;
- contraction and extension velocity distributions;
- velocity versus pressure/load/current contraction;
- inflation versus exhaust asymmetry;
- hysteresis;
- steady-state pose/length error;
- maximum practical acceleration/velocity;
- saturation behavior;
- coupling between muscle commands;
- residual error of the current pressure-to-contraction model;
- whether a constant 100 mm/s slew model is adequate or should become pressure-, load- and direction-dependent.

Possible increasingly sophisticated twin models:

```text
constant asymmetric slew rates
 -> slew rate vs pressure/load/contraction
 -> first-order fill/exhaust time constants
 -> empirical state-space / identified dynamic model
 -> force/equilibrium model with payload COM and damping
```

Always retain a simple baseline model for comparison.

## 9. Analyses that could improve platform control

Commanded versus measured pose makes closed-loop/control-oriented investigations possible without immediately changing the production controller.

Potential studies:

- per-axis static calibration/correction;
- feed-forward compensation for repeatable amplitude error;
- delay/phase compensation;
- pressure-command correction from measured response;
- asymmetric inflation/exhaust compensation;
- cross-axis coupling compensation;
- model-predictive or model-based approaches as research prototypes;
- whether feedback of pose is practical and sufficiently reliable for future control.

Any controller changes require separate safety review. The measurement system must initially be **observation-only** and must not command the physical platform.

## 10. Potential final-year Design Engineering project scopes

Several projects can share the same platform and datasets without duplicating work:

### Project A — Low-cost optical 6-DOF measurement
Design, calibrate and validate an AprilTag/ArUco camera system. Compare accuracy, latency, robustness and cost with a reference measurement method.

### Project B — Direct pneumatic-muscle displacement instrument
Design a temporary or permanent low-cost sensor capable of measuring one or more muscle lengths dynamically without interfering with motion.

### Project C — Sensor fusion
Combine camera pose and IMU data to obtain robust low-latency pose/velocity/acceleration estimates. Quantify improvement over camera alone.

### Project D — Dynamic system identification / digital twin
Use measured POC command and physical-response datasets to identify a dynamic model of the platform and improve the Panda3D pressure twin.

### Project E — Control-performance characterization
Quantify command-versus-actual pose, bandwidth, delay, coupling and repeatability across axes and loads; propose evidence-based compensation strategies.

### Project F — Closed-loop control feasibility
Using recorded data and/or simulation first, assess whether external pose feedback could improve control and what sensor latency/reliability would be required. Physical closed-loop tests should only follow an appropriate engineering/safety review.

### Project G — Test methodology and validation rig
Develop standardized motion-platform acceptance tests, automated analysis and repeatable reports for platform/twin/controller revisions.

## 11. Important distinction between measurement types

Keep these labels explicit in files and plots:

- **commanded pose** — POC requested/processed transform;
- **commanded length** — POC inverse-kinematic muscle request;
- **commanded pressure** — pressure sent/requested by POC;
- **vision-measured pose** — camera estimate of actual rigid-platform motion;
- **pose-derived muscle length** — calculated from measured pose and known geometry;
- **direct measured length** — only if a displacement sensor actually measures an actuator/muscle path.

This avoids accidentally treating estimated or commanded quantities as measured physical state.

## 12. Recommended first deliverable in the new conversation

Develop a concrete proof-of-concept plan for **one fixed camera + rigid AprilTag board**, including:

- recommended camera class/frame rate and approximate placement;
- tag/board dimensions and mounting location;
- OpenCV/AprilTag software options on Windows/Linux;
- camera and coordinate calibration procedure;
- synchronized `POC_TWIN` UDP capture;
- CSV schema;
- live measured-pose display;
- conversion of measured pose to the six MDX suspended-platform muscle lengths;
- offline plots of commanded versus measured pose/length;
- uncertainty/latency validation tests;
- a staged safe test sequence.

Then compare that plan against at least one direct displacement method and an IMU-assisted method before committing hardware.
