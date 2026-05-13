# KUKA youBot Autonomous Pick-and-Drop Simulation

**Platform:** CoppeliaSim EDU v4.10.0 (rev. 0) 64-bit (serial version 25)  
**Script type:** Threaded Python simulation script  
**Robot:** KUKA youBot — omnidirectional mobile platform with 5-DOF manipulator arm and parallel-jaw gripper

---

## Table of Contents
1. [Project Overview](#1-project-overview)
2. [Platform and Environment](#2-platform-and-environment)
3. [System Architecture](#3-system-architecture)
4. [Configuration Reference (CFG)](#4-configuration-reference-cfg)
5. [Pose Registry (POSES)](#5-pose-registry-poses)
6. [Module Breakdown](#6-module-breakdown)
   - 6.1 [Logging Utilities](#61-logging-utilities)
   - 6.2 [Math Helpers](#62-math-helpers)
   - 6.3 [Controller Class — YouBotPickController](#63-controller-class--youbotpickcontroller)
     - 6.3.1 [Initialisation and Handle Binding](#631-initialisation-and-handle-binding)
     - 6.3.2 [Object Access](#632-object-access)
     - 6.3.3 [Geometry and Sensing](#633-geometry-and-sensing)
     - 6.3.4 [Base Movement Primitives](#634-base-movement-primitives)
     - 6.3.5 [Navigation Actions](#635-navigation-actions)
     - 6.3.6 [Arm Movement Primitives](#636-arm-movement-primitives)
     - 6.3.7 [Wrist Orientation Computation](#637-wrist-orientation-computation)
     - 6.3.8 [Arm Actions (High-Level)](#638-arm-actions-high-level)
     - 6.3.9 [Gripper Control](#639-gripper-control)
     - 6.3.10 [Object Attachment / Detachment](#6310-object-attachment--detachment)
     - 6.3.11 [Drop Sequence](#6311-drop-sequence)
     - 6.3.12 [Mission Execution](#6312-mission-execution)
7. [End-to-End Mission Flow](#7-end-to-end-mission-flow)
8. [Key Algorithms and Design Decisions](#8-key-algorithms-and-design-decisions)
9. [CoppeliaSim Entry Points](#9-coppeliasim-entry-points)
10. [Debug Utilities](#10-debug-utilities)
11. [Known Tunable Parameters Summary](#11-known-tunable-parameters-summary)
12. [Known Limitations](#12-known-limitations)

---

## 1. Project Overview

This project implements a fully autonomous **pick-and-drop** mission for the KUKA youBot inside a CoppeliaSim physics simulation. A single Python threaded script drives the entire robot — wheels, 5-DOF arm, and gripper — through a repeatable cycle:

1. **Navigate** to a target cuboid using closed-loop proportional control.
2. **Pick** the cuboid using a multi-phase arm sequence and a symmetric screw-drive gripper.
3. **Carry** the cuboid to a floor edge while the arm holds a neutral carry pose.
4. **Drop** the cuboid off the edge.
5. **Return** the robot to the world origin and repeat for the next target.

The design is modular and data-driven: all physical parameters, gains, and timing values are centralised in a single `CFG` dictionary, and all arm configurations are stored as named entries in a `POSES` registry. This separation makes the system straightforward to re-tune without touching any control logic.

---

## 2. Platform and Environment

| Item | Details |
|------|---------|
| Simulator | CoppeliaSim EDU v4.10.0 (rev. 0) 64-bit |
| Script model | Threaded Python (blocking `sim.wait` calls allowed) |
| Robot model | KUKA youBot |
| Mobile base | 4-wheel mecanum/omnidirectional drive |
| Manipulator | 5-DOF serial arm (youBotArmJoint0–4) |
| End-effector | Parallel-jaw gripper with two screw-drive joints |
| Sensing | Down-facing proximity sensor (edge detection), wrist orientation sensor, forward-facing RGB vision sensor (`frontCam`) |
| Scene objects | Up to 8 cuboid targets (`/Cuboid` through `/Cuboid6`), a flat floor, a virtual home dummy at the world origin, two coloured goal posts (green and red) marking the drop-off area |

The script interfaces with CoppeliaSim exclusively through the `sim` API object (injected via `require("sim")` at runtime). No external Python libraries beyond the standard `math` and `traceback` modules are required.

---

## 3. System Architecture

```
youBot_script.py
│
├── CFG  ─────────────── Global configuration dictionary (all tunables)
├── POSES ────────────── Named arm configurations (degrees + movement mode)
│
├── log_info() ───────── Dual-output logger (stdout + CoppeliaSim log panel)
├── StepLogger ───────── Context manager for START/DONE/FAIL step tracing
│
├── Math helpers ─────── clamp, deg2rad, deg_pose_to_rad,
│                         _wrap_pi, _quat_to_R,
│                         _axis_world_from_pose, _proj_xy_unit, _signed_angle_2d
│
├── YouBotPickController (main controller class)
│   ├── init_handles()           — bind all sim object handles
│   ├── get_object_strict()      — robust path-based object lookup
│   │
│   ├── Geometry / Sensing
│   │   ├── forward_lateral()    — decompose base-relative position
│   │   ├── heading_error()      — angular error to target
│   │   ├── get_clearance_to()   — proximity via checkDistance API
│   │   ├── floor_detected()     — edge sensor proximity poll
│   │   ├── get_front_cam_rgb()  — capture RGB image from frontCam
│   │   └── detect_goalposts()   — colour-threshold goalpost detection
│   │
│   ├── Base Movement — Primitives
│   │   ├── wheels_from_twist()  — mecanum inverse kinematics
│   │   ├── stop_base()          — zero all wheel velocities
│   │   └── _drive_for_duration()— timed open-loop drive
│   │
│   ├── Navigation Actions
│   │   ├── turn_to_face_target()        — P-controller yaw alignment
│   │   ├── drive_to_stop()              — P-controller approach + stop
│   │   ├── drive_to_floor_edge()        — edge-seeking drive loop
│   │   ├── drive_to_dropzone_visual()   — camera-guided goalpost alignment + approach
│   │   └── return_base_to_world_origin() — home navigation
│   │
│   ├── Arm Movement — Primitives
│   │   ├── move_joint_smooth()       — single-joint rate-limited motion
│   │   ├── move_arm_sequential()     — joint-by-joint arm move
│   │   └── move_arm_interpolated()   — simultaneous LERP arm move
│   │
│   ├── Wrist Computation
│   │   └── compute_wrist_for_target() — quaternion-based wrist alignment
│   │
│   ├── Arm Actions (High-Level)
│   │   ├── go_pregrip()           — hover position above target
│   │   ├── go_grip_ready()        — descend to grasp position
│   │   ├── return_arm_to_neutral()— carry pose
│   │   └── go_tucked_pose()       — compact travel pose
│   │
│   ├── Gripper
│   │   ├── open_gripper()
│   │   ├── close_gripper_until_confirmed()
│   │   ├── step_close_gripper_symmetric()
│   │   └── is_grip_confirmed()
│   │
│   ├── Object Attachment
│   │   ├── attach_object_to_grip()
│   │   └── detach_object_from_grip()
│   │
│   ├── Drop
│   │   └── drop_cuboid_off_edge()
│   │
│   └── Mission
│       ├── pick_and_drop_one()    — full single-object cycle
│       └── run_mission_loop()     — iterate over all pick targets
│
└── CoppeliaSim entry points
    ├── sysCall_init()
    ├── sysCall_thread()
    └── sysCall_cleanup()
```

---

## 4. Configuration Reference (CFG)

All parameters are stored in the module-level `CFG` dictionary. Values can be modified without changing any logic.

### Scene Object Paths

| Key | Default | Description |
|-----|---------|-------------|
| `cuboid_path` | `"/Cuboid"` | Absolute scene path of the first/default cuboid |
| `base_ref_path` | `":/Rectangle0"` | Model-relative path to the youBot reference frame used for all relative position queries |
| `grip_attach_path` | `":/gripAttach"` | Dummy object to which picked objects are parented during carry |
| `arm_joint_paths` | (list, 5 entries) | Model-relative paths for arm joints 0–4 |
| `gripper_joint_paths` | (list, 2 entries) | Paths for the two screw-drive gripper joints |
| `wheel_joint_paths` | (list, 4 entries) | Paths for the four mecanum wheel joints (FL, RL, RR, FR) |
| `pick_targets` | (list, 8 cuboid paths) | Ordered list of scene objects to pick during the mission |

### Navigation Parameters

| Key | Default | Description |
|-----|---------|-------------|
| `stop_distance` | `0.16 m` | Target clearance at which the base stops before grasping |
| `dt` | `0.02 s` | Main control loop timestep |
| `forward_axis` | `"y"` | Which local axis of `base_ref` points forward (`"x"` or `"y"`) |
| `forward_sign` | `1.0` | Negate to reverse the definition of forward |
| `omega_sign` | `1.0` | Negate if turning direction is inverted |
| `ang_kp` | `0.8` | Proportional gain for yaw control |
| `ang_max` | `0.6 rad/s` | Saturation limit on yaw-rate command |
| `face_tol_deg` | `8.0°` | Angular tolerance for "facing target" convergence |
| `face_stable_steps` | `10` | Number of consecutive steps inside tolerance before turn is declared complete |
| `drive_angle_gate_deg` | `10.0°` | Robot must be facing within this angle before forward drive is applied |
| `approach_speed` | `0.15 m/s` | Maximum forward drive speed during approach |
| `approach_kp` | `1.0` | Proportional gain for forward drive |
| `wheel_speed_max` | `5 rad/s` | Hard clamp on individual wheel velocity commands |
| `wheel_dir` | `[1,1,1,1]` | Per-wheel direction flip flags |
| `vx_sign` | `-1.0` | Sign correction for forward drive (negate if robot drives backwards) |
| `stop_band` | `0.03 m` | Hysteresis band added to `stop_distance` for stable stop detection |
| `stop_stable_steps` | `10` | Consecutive steps inside stop band required before halting |
| `max_drive_time` | `60 s` | Safety timeout for the drive loop |

### Mecanum Kinematics Constants

| Key | Default | Description |
|-----|---------|-------------|
| `wheel_radius` | `0.0475 m` | Radius of each mecanum wheel |
| `lx_plus_ly` | `0.386 m` | Sum of half-wheelbase and half-track (geometric constant for omni IK) |

### Arm / Joint Motion

| Key | Default | Description |
|-----|---------|-------------|
| `joint_move_speed` | `0.8 rad/s` | Default joint angular speed for smooth moves |
| `joint_eps` | `1e-3 rad` | Position tolerance to declare a joint move complete |
| `joint_move_timeout_s` | `100 s` | Safety timeout per joint move |
| `joint_stall_eps` | `1e-6 rad` | Minimum change per step; below this the joint is considered stalled |
| `joint_stall_steps` | `80` | Consecutive stall-detection steps before raising an error |
| `arm_speed_pregrip` | `0.6 rad/s` | Slower joint speed used during the pregrip approach |
| `grip_ready_duration_s` | `1.9 s` | Total duration of the interpolated grip-ready descent |
| `grip_ready_dt` | `0.02 s` | Interpolation step time for grip-ready move |
| `pregrip_pause_s` | `2.5 s` | Dwell time at the pregrip hover position |
| `post_arm_settle_s` | `0.05 s` | Short pause inserted after every arm move |

### Gripper Parameters

| Key | Default | Description |
|-----|---------|-------------|
| `gripper_open_j1` | `0.025` | Open position for screw joint 1 |
| `gripper_open_j2` | `-0.050` | Open position for screw joint 2 |
| `gripper_close_goal_j1` | `0.018` | Closed (grip confirmed) target for joint 1 |
| `gripper_close_goal_j2` | `-0.032` | Closed (grip confirmed) target for joint 2 |
| `gripper_step_j1` | `-0.001` | Per-tick decrement applied to joint 1 during closure |
| `gripper_step_j2` | `+0.002` | Per-tick increment applied to joint 2 during closure |
| `gripper_tol` | `5e-4` | Position tolerance for grip-confirmed check |
| `gripper_close_dt` | `0.05 s` | Delay between gripper closure steps |
| `gripper_close_timeout` | `50 s` | Safety timeout for gripper closure |

### Edge Detection and Drop

| Key | Default | Description |
|-----|---------|-------------|
| `edge_sensor_path` | `":/edgeSensorDown"` | Down-facing proximity sensor path |
| `floor_entity_path` | `"/Floor"` | Scene floor shape path (optional) |
| `edge_drive_speed` | `0.10 m/s` | Forward speed while searching for the edge |
| `edge_timeout_s` | `60 s` | Safety timeout for edge search |
| `edge_lost_steps` | `6` | Consecutive "no floor" readings required to confirm edge |
| `edge_backoff_s` | `0.3 s` | Duration of backward nudge after edge detected |
| `edge_backoff_speed` | `-0.06 m/s` | Speed of the backoff move |
| `edge_drop_pause_s` | `0.2 s` | Settle pause before opening the gripper to drop |
| `edge_reverse_s` | `1.0 s` | Duration of reverse drive after drop |
| `edge_reverse_speed` | `-0.04 m/s` | Speed of reverse drive after drop |

### Vision-Guided Drop Zone (Goal Posts)

The robot uses a forward-facing RGB vision sensor (`frontCam`) to locate a pair of coloured goal posts — one green, one red — that mark the boundaries of the drop-off area. Once both posts are detected the robot visually servo-drives between them and then transitions to the standard floor-edge drop.

#### Camera / Scene

| Key | Default | Description |
|-----|---------|-------------|
| `front_cam_path` | `":/frontCam"` | Path to the forward-facing vision sensor (model-relative); use `"/frontCam"` if the sensor lives at the scene root |

#### Colour Thresholds (RGB 0–255)

| Key | Default | Description |
|-----|---------|-------------|
| `gp_green_min` | `[0, 180, 0]` | Lower bound for green post pixels (R, G, B) |
| `gp_green_max` | `[80, 255, 80]` | Upper bound for green post pixels |
| `gp_red_min` | `[180, 0, 0]` | Lower bound for red post pixels |
| `gp_red_max` | `[255, 80, 80]` | Upper bound for red post pixels |
| `gp_min_pixels` | `40` | Minimum pixel count for a colour blob to be accepted as a post detection |
| `gp_y0_frac` | `0.0` | Top fraction of the image to include in colour scanning (0 = top row) |
| `gp_y1_frac` | `1.0` | Bottom fraction of the image to include (1 = bottom row) |
| `gp_stride` | `1` | Pixel stride when scanning the image (increase to reduce CPU cost) |

#### Visual Servo Control

| Key | Default | Description |
|-----|---------|-------------|
| `gp_kp_omega` | `0.6` | Proportional turning gain from horizontal pixel centring error |
| `gp_max_omega` | `0.35 rad/s` | Saturation limit on yaw-rate command from visual servo |
| `gp_omega_sign` | `1.0` | Flip to `-1.0` if the robot steers away from the posts |
| `gp_align_tol_px` | `16 px` | Maximum pixel error to be considered centred on the gate midpoint |
| `gp_align_stable_steps` | `2` | Consecutive aligned frames required before committing to a forward step |
| `gp_drive_tol_px` | `40 px` | Relaxed alignment tolerance used during forward steps |

#### Step-Drive Behaviour

| Key | Default | Description |
|-----|---------|-------------|
| `gp_forward_speed` | `0.10 m/s` | Forward speed when locked onto the gate |
| `gp_step_time` | `0.25 s` | Duration of each forward step |
| `gp_search_omega` | `0.25 rad/s` | Rotation speed when no posts are visible (searching) |
| `gp_timeout_s` | `200 s` | Overall timeout for the visual approach phase |

#### Terminal Approach (Stage B)

| Key | Default | Description |
|-----|---------|-------------|
| `gp_lost_hold_s` | `1.2 s` | Seconds to keep driving forward after both posts disappear (final commit) |
| `gp_near_pixels` | `800 px` | Single-post pixel count that triggers the near-stop / terminal hold |
| `gp_final_forward_speed` | `0.08 m/s` | Forward speed during the terminal hold phase |
| `gp_sep_decay` | `0.7` | Smoothing (EMA) factor for the stored inter-post separation estimate |

#### Early-Trigger Retry

| Key | Default | Description |
|-----|---------|-------------|
| `gp_min_align_steps` | `2` | Minimum number of forward-step phases required before a floor-edge transition is treated as a genuine approach; fewer steps triggers a retry |
| `gp_early_trigger_max_retries` | `3` | Maximum number of early-trigger retries before aborting |

### Miscellaneous

| Key | Default | Description |
|-----|---------|-------------|
| `clearance_check_threshold` | `5.0 m` | Maximum range for `checkDistance` API calls |
| `log_every_steps` | `25` | Print/log interval inside tight loops |
| `post_stop_settle_s` | `0.1 s` | Settle pause after `stop_base()` |
| `reset_base_xy` | `[0.0, 0.0]` | World XY target for home return and debug reset |
| `reset_base_yaw_deg` | `0.0°` | Yaw at reset (debug utility only) |

---

## 5. Pose Registry (POSES)

All arm configurations are stored centrally in the `POSES` dictionary. Joint angles are kept in **degrees** and converted to radians at the point of use via `deg_pose_to_rad()`, keeping the registry human-readable.

| Pose | joints_deg [J0–J3] | wrist_deg (J4) | Movement order | Mode |
|------|--------------------|----------------|----------------|------|
| `neutral` | [0, −40, −50, 0] | 0.0° | (0,1,2,3,4) | sequential |
| `pregrip` | [0, −40, −88, −52] | computed | (0,1,2,3) | sequential |
| `grip_ready` | [0, −52, −95, −33] | computed | (3,2,1) | interpolated |
| `tucked` | [0, 30, 60, 30] | 0.0° | (0,1,2,3,4) | sequential |

- **`neutral`** — upright carry pose used between pick and drop.
- **`pregrip`** — hover position directly above the target; wrist angle is computed geometrically from the target's orientation.
- **`grip_ready`** — final lowered grasp position; reached via simultaneous (LERP) motion of joints 1–3 in reverse order to avoid collisions.
- **`tucked`** — compact stowed pose used during base navigation.

`wrist_deg: None` means that joint 4 (the wrist) is not part of the static pose and is instead computed dynamically by `compute_wrist_for_target()`.

---

## 6. Module Breakdown

### 6.1 Logging Utilities

#### `log_info(msg)`
Dual-channel logger: writes to `stdout` (visible in the terminal) **and** calls `sim.addLog(sim.verbosity_scriptinfos, msg)` to also write to the CoppeliaSim log window. The `sim.addLog` call is wrapped in a try/except so the function degrades gracefully if called before the `sim` object is available.

#### `StepLogger`
A Python context manager that wraps any named step with consistent `[STEP START]`, `[STEP DONE]`, and `[STEP FAIL]` log messages. Used with `with StepLogger("Step name"):` throughout the codebase to make execution flow traceable in logs without repetitive boilerplate.

---

### 6.2 Math Helpers

All helper functions are pure (no side effects) and module-level.

| Function | Signature | Description |
|----------|-----------|-------------|
| `clamp` | `(x, lo, hi) → float` | Clamps `x` to `[lo, hi]`. Used everywhere velocity or angle commands are saturated. |
| `deg2rad` | `(deg) → float` | Degrees to radians conversion. |
| `deg_pose_to_rad` | `(joints_deg[4], wrist_rad=None) → list[5]` | Converts a 4-element degrees list to a 5-element radians list, appending the wrist value (default 0.0). |
| `_wrap_pi` | `(a) → float` | Wraps an angle in radians to (−π, π]. |
| `_quat_to_R` | `(qx, qy, qz, qw) → 3×3 list` | Converts a unit quaternion to a 3×3 rotation matrix. |
| `_axis_world_from_pose` | `(pose[7], local_axis) → [x,y,z]` | Extracts a world-frame column vector from a CoppeliaSim `[x,y,z, qx,qy,qz,qw]` pose. |
| `_proj_xy_unit` | `(v[3]) → [x,y]` | Projects a 3-D vector onto the XY plane and normalises it to a unit 2-D vector. |
| `_signed_angle_2d` | `(a[2], b[2]) → float` | Computes the signed angle (radians) from 2-D unit vector **a** to **b** using `atan2`. |

---

### 6.3 Controller Class — `YouBotPickController`

The central controller class encapsulates all robot state and all motion primitives. One instance (`ctrl`) is created at simulation start and persists for the entire run.

#### Instance Variables

| Variable | Description |
|----------|-------------|
| `model` | Handle to the youBot model root |
| `cuboid` | Handle to the first/default cuboid (init-time only; actual targets are resolved from `CFG["pick_targets"]`) |
| `base_ref` | Handle to the youBot base reference frame (all relative measurements use this) |
| `arm` | List of 5 joint handles (J0–J4) |
| `gripper` | List of 2 gripper joint handles |
| `wheels` | List of 4 wheel joint handles (FL, RL, RR, FR) |
| `grip_attach` | Handle to the gripAttach dummy (objects are re-parented here during carry) |
| `wrist_sensor` | Handle to the wrist orientation sensor used for wrist angle computation |
| `edge_sensor` | Handle to the down-facing proximity sensor for floor-edge detection |
| `floor_entity` | Handle to the floor shape (optional; used to narrow proximity sensor checks) |
| `home_dummy` | Scene dummy placed at (0,0,0) — used as a navigation target for home return |
| `attached_object` | Handle of the currently held object, or `None` |
| `_attached_prev_parent` | Parent handle before attachment — restored on release |
| `_attached_prev_static` | Static flag before attachment — restored on release |
| `_wrist_latched` | Cached wrist angle for the current pick cycle; cleared at the start of each pick |
| `_arm_moved` | Boolean flag — set `True` once the arm starts moving, so the `finally` block knows to tuck it back |

---

#### 6.3.1 Initialisation and Handle Binding

**`init_handles()`**
Resolves all required CoppeliaSim object handles by path and stores them as instance variables. Errors at this stage are fatal — they cause `INIT_OK = False` and abort the mission.

Specific responsibilities:
- Retrieves the model root via `sim.getObject(":")`.
- Calls `get_object_strict()` for every joint, gripper, wheel, and sensor.
- Binds the forward-facing vision sensor (`frontCam`) to `self.front_cam` for use by the visual goal-post approach.
- Creates (or reuses) a small scene dummy named `HOME_0_0` at the world origin; this is the navigation target for the home-return phase. Reusing an existing dummy prevents object leaks if the script is re-initialised.
- Sets all wheel velocities to zero as a safety measure.

---

#### 6.3.2 Object Access

**`get_object_strict(path, label)`**
Robust object lookup that handles both absolute scene paths (starting with `/`) and model-relative paths (starting with `:`). If the direct path fails, it also tries common prefix variants (`:/`, `./`, `/`). Raises a descriptive `RuntimeError` if no handle is found, making scene configuration problems easy to diagnose.

---

#### 6.3.3 Geometry and Sensing

**`forward_lateral(rel)`**
Decomposes a base-relative 3-D position vector into `(forward, lateral)` using the `forward_axis` and `forward_sign` configuration. Abstracts the axis convention so the rest of the code can always use `forward` and `lateral` regardless of how the youBot model is oriented.

**`heading_error(rel)`**
Returns the angular error (radians) from the robot's forward direction to the direction of a target, computed from the forward/lateral decomposition using `atan2`.

**`get_clearance_to(target_handle)`**
Calls `sim.checkDistance(base_ref, target_handle, threshold)` to get the minimum clearance (shortest distance between the base collision mesh and the target object). Returns `inf` on failure. This is more geometrically accurate than a point-to-point distance and is used as the stop criterion during cuboid approach.

**`floor_detected()`**
Polls the down-facing proximity sensor via `sim.checkProximitySensor`. Returns `(True, distance)` when the floor is detected, `(False, None)` otherwise. The floor entity handle is passed as the sensing target to avoid spurious hits from other scene objects.

**`get_front_cam_rgb()`**
Captures a fresh RGB image from the forward-facing vision sensor (`frontCam`). Calls `sim.handleVisionSensor` to ensure the image is up-to-date, then calls `sim.getVisionSensorImg` and converts the raw byte buffer to a flat `[R,G,B, R,G,B, …]` integer list. Returns `(rgb_list, width, height)`.

**`detect_goalposts()`**
Analyses the most recent camera image to locate the green and red goal posts that mark the drop-off area. For each colour it computes the horizontal centroid (`cx`) and pixel count (`n`) of all pixels whose RGB values fall within the configured min/max thresholds. Returns:

- `seen_both` — `True` when both posts exceed `gp_min_pixels`.
- `err_px` — signed horizontal error (pixels) from the midpoint of the two posts to the image centre; positive means the gate centre is to the right.
- `img_w` — image width in pixels.
- `info` — diagnostic dict containing `seen_g`, `seen_r`, `cx_g`, `cx_r`, `n_g`, `n_r`, and `sep` (inter-post pixel separation, when both are visible).

---

#### 6.3.4 Base Movement Primitives

**`wheels_from_twist(vx, vy, omega)`**
Implements the **mecanum wheel inverse kinematics**:

```
w_FL = (vx − vy − L·ω) / r
w_RL = (vx + vy − L·ω) / r
w_RR = (vx − vy + L·ω) / r
w_FR = (vx + vy + L·ω) / r
```

Where `r` is the wheel radius and `L = lx + ly` is the combined half-wheelbase and half-track. The result is further scaled by per-wheel direction flags (`CFG["wheel_dir"]`). This single function is called by every motion primitive.

**`stop_base()`**
Sets all four wheel target velocities to 0.0. Called at the end of every move and in `finally` blocks.

**`_drive_for_duration(vx, duration_s)`**
Open-loop timed drive: applies a fixed forward velocity `vx` (with the `vx_sign` correction applied internally) for a specified duration, then stops. Used for the post-edge backoff and the post-drop reverse.

---

#### 6.3.5 Navigation Actions

**`turn_to_face_target(target_handle)`**
Closed-loop P-controller for yaw alignment:
- Computes heading error to the target each step.
- Applies a proportional yaw command: `ω = clamp(kp · err, −ωmax, ωmax)`.
- Stops when the error stays within `face_tol_deg` for `face_stable_steps` consecutive steps.
- Stability check prevents premature convergence caused by oscillation.

**`drive_to_stop(target_handle, stop_distance, use_clearance)`**
Closed-loop P-controller for forward approach:
- Simultaneously corrects heading and drives forward.
- Forward drive (`vx`) is only applied when the heading error is within `drive_angle_gate_deg` — this prevents the robot from slewing sideways when misaligned.
- Stop criterion switches between `checkDistance` clearance (accurate, used for cuboid approach) and planar XY distance (used for home navigation) via the `use_clearance` flag.
- A hysteresis band (`stop_band`) and stable-step counter prevent oscillation at the target.
- A hard timeout (`max_drive_time`) prevents infinite loops in edge cases.

**`drive_to_floor_edge()`**
Drives the robot forward at a fixed speed while polling the down-facing proximity sensor. The edge is confirmed when the sensor returns "no floor" for `edge_lost_steps` consecutive frames (debounce). After stopping, a short backward nudge (`edge_backoff_s`) ensures only the cuboid falls, not the robot.

**`drive_to_dropzone_visual()`**
Camera-guided approach to the drop-off area using the two coloured goal posts as visual beacons. The method operates in two stages:

*Stage A — Visual servo alignment and step-drive:*
- Each iteration calls `detect_goalposts()` to locate the green and red posts in the camera image.
- If both posts are visible the robot computes the horizontal pixel error between the gate midpoint and the image centre and applies a proportional yaw correction (`gp_kp_omega`). When the error is within `gp_align_tol_px` for `gp_align_stable_steps` consecutive frames, the robot takes a short forward step (`gp_step_time`).
- If neither post is visible the robot spins at `gp_search_omega` until at least one post reappears.
- The floor-edge sensor is polled on every iteration; a confirmed edge immediately ends Stage A and transitions to the drop sequence.

*Stage B — Terminal hold:*
- Triggered when the posts grow very large (`gp_near_pixels`) or both posts disappear after the robot was previously aligned. The robot continues forward at `gp_final_forward_speed` for up to `gp_lost_hold_s` seconds, relying on the edge sensor for the final stop.

*Early-trigger retry:*
- If the edge is reached after fewer than `gp_min_align_steps` forward steps (robot started too close to the posts), the approach is considered a false trigger. The robot returns home and restarts the visual search. Up to `gp_early_trigger_max_retries` retries are attempted before aborting.

**`return_base_to_world_origin()`**
Convenience sequencer: calls `turn_to_face_target` then `drive_to_stop` targeting the `home_dummy` at (0,0,0).

---

#### 6.3.6 Arm Movement Primitives

**`move_joint_smooth(joint, target, speed, joint_name)`**
Rate-limited single-joint controller:
- Reads the current joint position each step and issues a target incremented by at most `speed × dt`.
- Respects the joint's configured limits (read via `sim.getJointInterval`).
- Declares completion when `|error| ≤ joint_eps`.
- Detects stall (no motion for `joint_stall_steps` consecutive steps while error is large) and raises `RuntimeError`.
- Detects near-stall (no motion while already very close to target) and snaps to target to avoid hang.

**`move_arm_sequential(target_config, order)`**
Moves each arm joint in the specified order by calling `move_joint_smooth` sequentially. The `order` tuple allows arbitrary joint sequencing (e.g., wrist first, then shoulder) to avoid collisions.

**`move_arm_interpolated(target_config, joints, duration, dt)`**
Simultaneously interpolates a set of joints from their current positions to target positions over a fixed duration using linear interpolation (LERP):

```
q_i(t) = q_start_i + α · (q_target_i − q_start_i),   α = min(1, t / duration)
```

Used for the grip-ready descent to ensure smooth, coordinated arm motion.

---

#### 6.3.7 Wrist Orientation Computation

**`compute_wrist_for_target(target_handle)`**

The wrist joint (J4) must be aligned with the horizontal face of the target cuboid to ensure reliable grasping. This method computes the required wrist angle geometrically:

1. **Sensor and target poses** are retrieved as 7-element `[x,y,z,qx,qy,qz,qw]` vectors.
2. **Current sensor direction** is the world-frame X-axis of the wrist orientation sensor, projected to the XY plane and normalised.
3. **Target directions** are the world-frame X and Y axes of the cuboid, also projected and normalised.
4. The **signed angle** from the sensor direction to each target axis is computed. Both 90°-equivalent axes and their 180° flips are considered, giving up to 6 candidate wrist deltas.
5. Each candidate is evaluated: the desired wrist position = `wrap_π(current + delta)`. Candidates that exceed joint limits are penalised by `10 + overshoot`. The candidate with the **lowest score** (smallest angular change within limits) is selected.
6. The result is cached in `self._wrist_latched` and reused for the grip-ready phase of the same pick cycle. It is cleared (`None`) at the start of each new pick.

This approach handles arbitrary cuboid yaw orientations and joint-limit constraints automatically.

---

#### 6.3.8 Arm Actions (High-Level)

**`go_pregrip(target_handle)`**
Two-phase hover move:
- **Phase A:** Moves joints 0–3 to the `pregrip` configuration (keeping the current wrist angle) at `arm_speed_pregrip`.
- **Phase B:** Computes the wrist alignment angle via `compute_wrist_for_target`, then moves joint 4 only.
The split ensures the arm is roughly above the target before wrist alignment is computed, making the sensor geometry more representative.

**`go_grip_ready(target_handle)`**
Interpolated descent to the `grip_ready` configuration, moving joints 1–3 simultaneously over `grip_ready_duration_s` seconds. Reuses the cached wrist angle.

**`return_arm_to_neutral()`**
Sequential move to the `neutral` carry pose. Called after a successful grip.

**`go_tucked_pose()`**
Sequential move to the `tucked` compact pose. Used before and after base navigation.

---

#### 6.3.9 Gripper Control

The youBot gripper uses two opposed screw-drive joints: joint 1 decreases and joint 2 increases to close.

**`open_gripper()`**
Detaches any held object, then sets both gripper joints to their open positions and waits 0.3 s.

**`close_gripper_until_confirmed(target_handle)`**
Iteratively closes the gripper one small step per tick using `step_close_gripper_symmetric()`. Each tick:
- Joint 1 moves toward `gripper_close_goal_j1` by `gripper_step_j1`.
- Joint 2 moves toward `gripper_close_goal_j2` by `gripper_step_j2`.

On each iteration `is_grip_confirmed()` checks whether both joints have reached their goal positions (within `gripper_tol`). When confirmed, `attach_object_to_grip()` is called immediately to parent the object to the gripper. A safety timeout prevents infinite looping if the gripper is blocked.

---

#### 6.3.10 Object Attachment / Detachment

**`attach_object_to_grip(obj_handle)`**
Simulates a physical grip by re-parenting the picked object to the `gripAttach` dummy:
1. Saves the object's current parent and `shapeintparam_static` flag.
2. Re-parents the object to `gripAttach` with `keepInPlace=True` (preserves world pose).
3. Sets the object to static and non-respondable to eliminate physics jitter while carried.
4. Calls `sim.resetDynamicObject` to flush any residual velocity.

**`detach_object_from_grip(obj_handle)`**
Reverses the attachment:
1. Re-parents the object to its original parent (or the scene root if it had none).
2. Restores the original static and respondable flags.
3. Calls `sim.resetDynamicObject` to allow the physics engine to take over (so the object falls naturally).

---

#### 6.3.11 Drop Sequence

**`drop_cuboid_off_edge()`**
Called after the robot is positioned at the floor edge:
1. Waits `edge_drop_pause_s` for the scene to settle.
2. Calls `open_gripper()` (which detaches the object first).
3. Reverses the robot away from the edge at `edge_reverse_speed` for `edge_reverse_s` seconds.

---

#### 6.3.12 Mission Execution

**`pick_and_drop_one(target_handle)`**
Full single-object pick-carry-drop-return cycle in a guaranteed-cleanup `try/finally` block:

```
turn to face target
→ drive to stop distance
→ go_pregrip
→ pause at pregrip
→ go_grip_ready
→ close gripper until confirmed
→ return arm to neutral
→ drive to floor edge
→ drop cuboid off edge
→ go tucked pose
→ return base to world origin
```

`finally` block: stops the base, detaches any held object, and if `_arm_moved` is True (arm was extended when an exception occurred), executes an emergency tuck.

**`run_mission_loop()`**
Iterates through `CFG["pick_targets"]` in order. For each target:
- Resolves the scene handle. If the object does not exist (`getObject` returns −1), logs a skip message and continues.
- Calls `pick_and_drop_one()`.
- On exception, logs the full traceback and continues to the next target rather than aborting the entire mission.

---

## 7. End-to-End Mission Flow

```
sysCall_init()
  └─ init_handles()         ← bind all object handles (incl. frontCam); abort on failure

sysCall_thread()
  └─ wait 0.2 s             ← let physics settle
  └─ run_mission_loop()
       └─ FOR EACH cuboid in pick_targets:
            turn_to_face_target()
            drive_to_stop()              ← approach with clearance-based stop
            go_pregrip()                 ← arm phases A + B (wrist computed here)
            pause 2.5 s                  ← settle at hover
            go_grip_ready()              ← interpolated descent
            close_gripper()              ← incremental closure until confirmed
            attach_object_to_grip()      ← object parented, frozen, non-respondable
            return_arm_to_neutral()      ← carry pose
            drive_to_dropzone_visual()   ← frontCam visual servo to goal-post gate
                                            → Stage A: align + step-drive toward posts
                                            → Stage B: terminal hold to floor edge
            backoff (brief)
            drop_cuboid_off_edge()       ← open gripper, detach, reverse
            go_tucked_pose()             ← compact stow
            return_base_to_world_origin()  ← home: turn then drive

sysCall_cleanup()
  └─ stop_base()            ← ensure wheels stopped on sim end
```

---

## 8. Key Algorithms and Design Decisions

### Mecanum Omnidirectional Kinematics
The robot uses standard mecanum inverse kinematics with a constant `L = lx + ly` (sum of half-wheelbase and half-track). Commands are expressed as body-frame twists `(vx, vy, ω)` and converted to individual wheel velocities per the standard formula. Only `vx` and `ω` are used in practice (no lateral `vy` component), since the navigation strategy is turn-then-drive.

### Turn-Then-Drive Navigation
Rather than pursuing a curved trajectory to the target, the robot first aligns its heading and then drives straight forward. This decouples the angular and linear degrees of freedom, simplifying controller design and making behaviour predictable in a cluttered scene.

### Clearance-Based Stop Criterion
During cuboid approach, `sim.checkDistance` is used instead of a point-to-point position measurement. This accounts for the non-zero extents of both the robot collision mesh and the target object, ensuring a consistent physical stand-off regardless of object size or orientation.

### Wrist Alignment via Quaternion Geometry
Rather than hard-coding a wrist angle for each cuboid, the wrist angle is derived geometrically from the relative orientation between the wrist sensor's current direction and the two principal axes of the target. Axis ambiguity (a box face looks the same from 0° and 180°) is resolved by considering all axis-flip candidates and selecting the one requiring the smallest joint movement within limits.

### Gripper Closure with Confirmation
Rather than commanding the gripper directly to a closed position, the script steps it incrementally each tick and monitors joint positions. The grip is only "confirmed" when both joints reach their goal positions. This allows the gripper to self-detect when it has clamped onto an object (resistance) and prevents over-closing.

### Object Attachment via Re-Parenting
CoppeliaSim's physics does not natively support kinematic constraints for grasping. The script simulates a firm grip by re-parenting the object to a dummy rigidly attached to the gripper, and setting the object to static/non-respondable. Original properties are saved and fully restored on release, so the object falls normally after being dropped.

### Safe Mission Loop with Exception Recovery
Each pick cycle is wrapped in `try/finally` to guarantee wheel stop and object release even if any step raises an exception. The outer mission loop catches exceptions per-object, allowing the mission to continue to remaining targets after a single failure.

### Vision-Guided Drop-Zone Navigation
The drop-off area is marked by a green post on the left and a red post on the right, forming a gate. Rather than relying on a fixed coordinate for the drop zone, the robot uses its forward `frontCam` vision sensor to locate the posts by colour thresholding and servo-drives toward the midpoint of the gate. This makes the drop-zone approach robust to small variations in where the robot ends up after carrying a cuboid, without requiring any additional positioning infrastructure.

### Wrist Latch per Pick Cycle
The computed wrist angle is cached after the first computation (at `go_pregrip`) and reused during `go_grip_ready`. This ensures that the arm descends to the same wrist orientation it aligned to at the hover height, without recomputing (which could give a different result as the scene geometry changes). The cache is explicitly cleared at the start of each new pick.

---

## 9. CoppeliaSim Entry Points

CoppeliaSim calls three functions in a threaded script:

| Function | When called | Responsibilities |
|----------|-------------|-----------------|
| `sysCall_init()` | Once at simulation start | Imports the `sim` API; creates the controller; calls `init_handles()`; sets `INIT_OK` flag |
| `sysCall_thread()` | Once; runs in a dedicated thread | Checks `INIT_OK`; waits 0.2 s; calls `run_mission_loop()` |
| `sysCall_cleanup()` | Once when simulation stops | Calls `stop_base()` as a safety measure |

The threaded execution model allows `sim.wait()` to be called freely inside control loops; CoppeliaSim yields the simulation clock and resumes the thread each simulation step.

---

## 10. Debug Utilities

**`_debug_reset_robot_to_world_origin(ctrl)`**
Teleports the robot model to the world origin by directly setting its pose. Preserves the Z height. Not called during normal mission execution — intended for manual recovery via the CoppeliaSim script console.

---

## 11. Known Tunable Parameters Summary

The following parameters are most likely to need adjustment when porting to a different scene layout or robot model:

| Parameter | Effect if too low | Effect if too high |
|-----------|-------------------|--------------------|
| `stop_distance` | Robot collides with cuboid | Robot stops too far away to grasp |
| `ang_kp` | Slow yaw alignment | Oscillation/overshoot |
| `approach_kp` | Slow approach | Overshoot, oscillation |
| `face_tol_deg` | Over-precise alignment (slow) | Misaligned approach |
| `gripper_step_j1/j2` | Very slow closure | Skips over closed position, misses confirmation |
| `edge_lost_steps` | False edge triggers | Drives past the edge |
| `grip_ready_duration_s` | Jerky descent (potential collision) | Very slow descent |
| `pregrip_pause_s` | Arm still settling when wrist computed | Wasted simulation time |
| `joint_move_speed` | Slow arm moves | Jerky motion, possible instability |
| `gp_kp_omega` | Slow visual centering on goal posts | Oscillation/overshoot when aligning |
| `gp_min_pixels` | Spurious post detections from noise | Real posts missed at distance |
| `gp_align_tol_px` | Over-precise alignment (many micro-corrections) | Robot drives toward posts while off-centre |
| `gp_lost_hold_s` | Robot stops short of the edge after posts disappear | Robot drives too far past the edge |

---

## 12. Known Limitations

### Objects Too Close to Goal Posts Confuse the Visual Pathing

The visual drop-zone approach relies entirely on colour thresholding to distinguish the green and red goal posts from the rest of the scene. If a picked cuboid (or any other coloured object in the scene) happens to be placed **close to, or partially overlapping, the goal posts in the camera's field of view**, the colour blobs detected by `detect_goalposts()` can include pixels from the nearby object. This corrupts the centroid calculation and shifts the perceived midpoint of the gate, causing the robot to steer off-course.

**Symptoms:**
- Robot curves to one side instead of driving straight through the gate.
- Gate-centre pixel error oscillates or jumps unexpectedly as the robot approaches.
- Robot enters the early-trigger retry loop even when starting at a normal distance.

**Root cause:** The colour segmentation step has no depth information and cannot separate objects that overlap in 2-D image space. A cuboid resting against a goal post or another brightly coloured object in the field of view blends into the post's pixel blob.

**Workarounds / mitigations:**
- Ensure the drop zone is kept clear of other objects during the carry phase.
- Increase `gp_min_pixels` to require larger blobs, which reduces the chance that a small overlap is misclassified as a full post.
- Tighten the colour thresholds (`gp_green_min/max`, `gp_red_min/max`) to accept only the most saturated post pixels, reducing contamination from similar-coloured surfaces.
- Reduce `gp_y0_frac` / `gp_y1_frac` to restrict the image scan to the vertical band where the posts actually appear, ignoring floor-level clutter.
- A future improvement would be to add a spatial consistency check (e.g., require the detected posts to be on opposite sides of the image and separated by at least a minimum pixel distance) to reject detections that are implausibly close together.
