# Report Evidence Pack — youBot Debris-Collection Mission

> **Source:** `youBot_script.py` (1,584 lines), the sole Python control file in the repository.  
> **Author clarifications** (Q&A session, 2026-05-14) are integrated inline and marked **[AUTHOR]**.

---

## (1) Repository Map

### Structure

The repository contains a single Python file (`youBot_script.py`) that runs as a **threaded CoppeliaSim script**.
CoppeliaSim calls `sysCall_init()` (line 1552) to initialise, then `sysCall_thread()` (line 1568) as the entry point;
the thread simply calls `ctrl.run_mission_loop()`.

### Main Classes and Key Functions

| Element | Lines | Role |
|---|---|---|
| `CFG` dict | 11–185 | All tunable parameters (see table below) |
| `POSES` dict | 194–219 | Named arm pose registry (stored in degrees) |
| `log_info()` | 225–230 | Print + CoppeliaSim `sim.addLog` wrapper |
| `StepLogger` | 233–247 | Context-manager; emits `[STEP START/DONE/FAIL]` bracketed logs |
| `clamp()`, `deg2rad()`, `deg_pose_to_rad()` | 253–269 | Scalar / unit helpers |
| `_wrap_pi()`, `_quat_to_R()`, `_axis_world_from_pose()`, `_proj_xy_unit()`, `_signed_angle_2d()` | 274–310 | Module-level wrist geometry helpers (no side-effects) |
| `YouBotPickController` | 316–1529 | Main controller class |
| `sysCall_init()` / `sysCall_thread()` / `sysCall_cleanup()` | 1552–1584 | CoppeliaSim lifecycle hooks |

**Key methods of `YouBotPickController`:**

| Method | Lines | Purpose |
|---|---|---|
| `init_handles()` | 349–385 | Resolve all simulator object handles |
| `wheels_from_twist()` | 447–456 | Mecanum IK: (vx, vy, ω) → wheel velocities |
| `turn_to_face_target()` | 483–516 | Pure-rotation P-controller |
| `drive_to_stop()` | 518–591 | Combined heading + distance P-controller |
| `drive_to_dropzone_visual()` | 595–938 | Full visual-servo drop-zone approach |
| `drive_to_floor_edge()` | 940–976 | Proximity-sensor edge-detection forward drive |
| `return_base_to_world_origin()` | 978–981 | Face + drive to home dummy at world (0, 0) |
| `move_joint_smooth()` | 987–1040 | Smooth joint move with stall detection |
| `move_arm_sequential()` | 1042–1050 | Drive joints one-at-a-time in specified order |
| `move_arm_interpolated()` | 1052–1073 | LERP joints simultaneously over fixed duration |
| `compute_wrist_for_target()` | 1079–1138 | Quaternion-based wrist alignment computation |
| `go_pregrip()` | 1144–1164 | Move arm to hover pose + wrist align |
| `go_grip_ready()` | 1166–1177 | Interpolated descent to grasp pose |
| `close_gripper_until_confirmed()` | 1225–1255 | Incremental gripper close with position check |
| `detect_goalposts()` | 1396–1433 | RGB threshold centroid detection |
| `pick_and_drop_one()` | 1446–1481 | Single-cube full pick-carry-drop cycle |
| `run_mission_loop()` | 1483–1529 | Outer mission loop (greedy nearest-first) |

### Tunable Parameters (`CFG`) — Key Values

| Parameter | Default | Meaning |
|---|---|---|
| `stop_distance` | `0.16` m | Clearance target for object approach |
| `dt` | `0.02` s | Simulation step time |
| `forward_axis` | `"y"` | Base-frame axis pointing "forward" |
| `forward_sign` | `1.0` | Sign flip for forward axis |
| `omega_sign` | `1.0` | Sign flip for yaw command |
| `ang_kp` | `0.8` | Heading P-gain |
| `ang_max` | `0.6` rad/s | Max yaw rate |
| `face_tol_deg` | `8.0°` | Heading tolerance for turn convergence |
| `face_stable_steps` | `10` | Consecutive steps required inside tolerance |
| `drive_angle_gate_deg` | `10.0°` | Max heading error before forward drive is allowed |
| `approach_speed` | `0.15` m/s | Max forward approach speed |
| `approach_kp` | `1.0` | Distance P-gain |
| `wheel_speed_max` | `5` rad/s | Wheel velocity clamp |
| `wheel_dir` | `[1,1,1,1]` | Per-wheel sign flip array |
| `vx_sign` | `-1.0` | Forward drive direction correction |
| `stop_band` | `0.03` m | Hysteresis margin on distance stop threshold |
| `stop_stable_steps` | `10` | Steps inside band before stop is accepted |
| `max_drive_time` | `60` s | Drive safety timeout |
| `wheel_radius` | `0.0475` m | Mecanum wheel radius (from CoppeliaSim model) |
| `lx_plus_ly` | `0.386` m | `lx + ly = 0.228 + 0.158`; sum of half-wheelbase + half-track (from CoppeliaSim model) |
| `joint_move_speed` | `0.8` rad/s | Default arm joint speed |
| `joint_eps` | `1e-3` rad | Joint arrival tolerance |
| `joint_stall_eps` | `1e-6` rad | Stall detection threshold per step |
| `joint_stall_steps` | `80` | Steps at stall before exception |
| `joint_move_timeout_s` | `100` s | Per-joint move timeout |
| `grip_ready_duration_s` | `1.9` s | Descent LERP duration |
| `gripper_open_j1 / j2` | `0.025 / −0.050` m | Open positions |
| `gripper_close_goal_j1 / j2` | `0.018 / −0.032` m | Close target positions |
| `gripper_step_j1 / j2` | `−0.001 / +0.002` m/tick | Per-tick closure increments |
| `gripper_tol` | `5e-4` m | Joint-position tolerance for "confirmed grip" |
| `gripper_close_dt` | `0.05` s | Time between closure steps |
| `gripper_close_timeout` | `50` s | Gripper close safety timeout |
| `pregrip_pause_s` | `2.5` s | Settle pause at pregrip hover pose |
| `arm_speed_pregrip` | `0.6` rad/s | Arm speed during pregrip/wrist moves |
| `edge_drive_speed` | `0.10` m/s | Speed during edge-search drive |
| `edge_lost_steps` | `6` | Consecutive "no floor" frames to confirm edge |
| `edge_backoff_s` | `0.3` s | Backoff duration after edge detected |
| `edge_backoff_speed` | `−0.06` m/s | Backoff speed |
| `edge_reverse_s` | `1.0` s | Post-drop reverse duration |
| `edge_reverse_speed` | `−0.04` m/s | Post-drop reverse speed |
| `gp_green_min / max` | `[0,180,0] / [80,255,80]` | Green post RGB threshold |
| `gp_red_min / max` | `[180,0,0] / [255,80,80]` | Red post RGB threshold |
| `gp_min_pixels` | `40` | Min pixels to accept a post detection |
| `gp_kp_omega` | `0.6` | Visual-servo heading gain |
| `gp_max_omega` | `0.35` rad/s | Visual-servo max yaw rate |
| `gp_forward_speed` | `0.10` m/s | Forward speed when gate is centred |
| `gp_align_tol_px` | `16` px | Pixel alignment tolerance for stable lock-on |
| `gp_align_stable_steps` | `2` | Frames for stable lock-on |
| `gp_search_omega` | `0.25` rad/s | Spin speed when posts not visible |
| `gp_lost_hold_s` | `1.2` s | HOLD phase duration after last valid sighting |
| `gp_near_pixels` | `800` px | Single-post pixel count triggering near-stop |
| `gp_min_sep_frac` | `0.25` | Gate must span ≥ 25 % of image width (manually tuned) |
| `gp_min_approach_s` | `2.0` s | Minimum elapsed time before edge transition is allowed |
| `gp_early_trigger_max_retries` | `3` | Max home-return retries for early-trigger safety |

---

## (2) Mission Loop + Target Selection (Section 3.1)

**Mission-loop function:** `run_mission_loop()`, lines 1483–1529.

### Pseudocode

```
run_mission_loop():
  loop forever:
    candidates = []
    for each path in CFG["pick_targets"]:   # ["/Cuboid", "/Cuboid0", ..., "/Cuboid6"]
      h = sim.getObject(path)
      if h == -1  : skip (object missing from scene)
      if h in collected_cubes: skip (already handled)
      candidates.append((path, h))

    if candidates is empty:
      log "all cubes collected or unavailable"
      break

    # Greedy nearest-first selection
    best = argmin over candidates of hypot(rel.x, rel.y)   # Euclidean in base frame

    try:
      pick_and_drop_one(best_h)
      collected_cubes.append(best_h)         # SUCCESS: mark done
    except Exception as e:
      log failure + traceback
      collected_cubes.append(best_h)         # FAILURE: mark attempted → no infinite retry
      continue
```

### Target Selection

**Greedy nearest-first.** At each iteration the 2D Euclidean distance from `self.base_ref` to every uncollected cube is computed via `sim.getObjectPosition(h, self.base_ref)` and the minimum is selected (lines 1502–1509).

**[AUTHOR]** The 8 cuboids are **randomly placed** in the scene at the start of each run, so the greedy selection order varies between runs. The controller does not use a pre-planned route.

### Collected / Attempted Tracking

A single list `self.collected_cubes` (initialised `[]` in `__init__`, line 342) stores **both** successfully picked and failed cube handles. Both the success branch (line 1515) and the exception branch (line 1528) append `best_h` to this list. A cube once attempted — regardless of outcome — is **never retried**.

---

## (3) Mobile Base Kinematics (Section 3.2)

**Function:** `wheels_from_twist()`, lines 447–456.

### Mecanum Inverse-Kinematics Formula

```python
r = CFG["wheel_radius"]     # 0.0475 m
L = CFG["lx_plus_ly"]       # lx + ly = 0.228 + 0.158 = 0.386 m

w_fl = (vx - vy - L * omega) / r
w_rl = (vx + vy - L * omega) / r
w_rr = (vx - vy + L * omega) / r
w_fr = (vx + vy + L * omega) / r
```

This is the standard 4-wheel mecanum kinematic model where `L = lx + ly` is the sum of half-wheelbase (longitudinal, `lx = 0.228 m`) and half-track (lateral, `ly = 0.158 m`). Each output is a wheel angular velocity in rad/s.

**[AUTHOR]** Both `lx` and `ly` values were taken directly from the **CoppeliaSim youBot model** geometry; they were not empirically re-tuned.

### Sign Corrections

- **Per-wheel flip array** `CFG["wheel_dir"] = [1,1,1,1]`: applied element-wise after the formula (line 455). Each entry can be set to `−1` to flip an individual wheel if the physical wiring or model convention differs.
- **Global drive-direction fix** `CFG["vx_sign"] = −1.0`: applied by all callers before passing `vx` into `wheels_from_twist` (e.g., line 570: `vx = CFG["vx_sign"] * v`). This corrects for the robot model's forward-axis orientation in CoppeliaSim.

### Lateral Velocity (`vy`)

`vy = 0.0` is passed in every navigation call throughout the mission.  
**[AUTHOR]** Lateral strafing is **intentionally disabled**. The approach strategy relies on the robot always driving straight-on toward targets (after a pure-rotation turn-to-face step), which simplifies gripper alignment because the arm always approaches from directly in front. Enabling `vy` would require additional lateral compensation in the arm/wrist logic.

---

## (4) Navigation Controllers (Section 3.3)

### Heading Error Definition

`heading_error()`, lines 421–423:

```python
def heading_error(self, rel):
    forward, lateral = self.forward_lateral(rel)   # forward=rel[1], lateral=rel[0] (axis="y")
    return math.atan2(lateral, forward)
```

With `forward_axis = "y"`: **`err = atan2(rel_x, rel_y)`** — the signed angle from the robot's forward direction to the vector pointing at the target.

### Control Laws

**Turn controller** (`turn_to_face_target()`, lines 483–516) — pure rotation, no translation:

```
omega = clamp(0.8 × err, −0.6, +0.6) × omega_sign
vx    = 0
```

**Drive controller** (`drive_to_stop()`, lines 518–591) — simultaneous heading correction and forward drive:

```
err_ang = atan2(lateral, forward)
omega   = clamp(0.8 × err_ang, −0.6, +0.6) × omega_sign

if |err_ang| ≤ gate (= 10°):
    err_lin = proximity − stop_distance
    v       = clamp(1.0 × err_lin, 0.0, 0.15)
    vx      = −1.0 × v          # vx_sign = −1.0
else:
    vx = 0
```

### Forward-Drive Gating

**Yes, gating is implemented.** Forward drive is enabled only when `|err_ang| ≤ math.radians(10.0°) ≈ 0.175 rad` (line 567). Outside this window the robot rotates in place until it is facing the target.

### Stop Criteria

| Mode | Proximity Measure | Stop Threshold |
|---|---|---|
| Object approach (`use_clearance=True`) | `sim.checkDistance(base_ref, target, 5.0)` shortest clearance | `≤ 0.16 + 0.03 = 0.19 m` |
| Home return (`use_clearance=False`) | `hypot(forward, lateral)` from `getObjectPosition` | `≤ 0.05 + 0.03 = 0.08 m` |

Both require the condition to hold for **10 consecutive steps** (`stop_stable_steps`) before the robot actually stops.

**[AUTHOR]** The `checkDistance` return value is used directly from the CoppeliaSim simulation data (`distData[-1]`); this index was verified to match the shortest distance reported by the simulator for the scene in use.

### Stability Counters, Hysteresis Bands, and Timeouts

| Counter / Band | Value | Purpose |
|---|---|---|
| `face_stable_steps = 10` | 10 × 0.02 s = 0.2 s | `turn_to_face_target`: hold within ±8° for 10 steps before stopping |
| `face_tol_deg = 8.0°` | ±8° | Heading tolerance hysteresis band |
| `stop_stable_steps = 10` | 10 steps | Drive: must be within `stop_band` for 10 consecutive steps |
| `stop_band = 0.03 m` | 3 cm | Hysteresis margin on distance stop |
| `max_drive_time = 60 s` | 60 s | Drive safety timeout → log and break |
| `joint_move_timeout_s = 100 s` | 100 s | Per-joint move timeout → exception |
| `joint_stall_steps = 80` | 80 steps | Joint stall exception (no movement, target not reached) |

---

## (5) Arm Control + Sequencing (Section 3.4)

### Two Motion Primitives

**1. Sequential** (`move_arm_sequential()`, lines 1042–1050):  
Drives joints one at a time in the order given by the `order` tuple. Each joint runs `move_joint_smooth()` to completion before the next begins. Used for `neutral`, `pregrip`, and `tucked` poses.

**2. Interpolated** (`move_arm_interpolated()`, lines 1052–1073):  
Moves all joints in the `joints` tuple **simultaneously** using linear interpolation over a fixed `duration`. Used for the `grip_ready` descent.

### LERP Equation (lines 1059–1064)

```
alpha(t) = min(1.0,  t / duration)

q_i(t)  = q_i_start + alpha(t) × (q_i_target − q_i_start)
         = (1 − alpha) × q_i_start + alpha × q_i_target
```

Pure linear interpolation; `alpha` is clamped at 1.0 so the final pose is always reached exactly.

### Arm Pose Registry (`POSES`, lines 194–219)

All angles stored in **degrees**; converted to radians by `deg_pose_to_rad()` at use time.  
`wrist_deg = None` means the wrist angle is computed dynamically by `compute_wrist_for_target()`.

| Pose | J0 | J1 | J2 | J3 | Wrist | Joint order | Motion mode |
|---|---|---|---|---|---|---|---|
| `neutral` | 0° | −40° | −50° | 0° | 0° | (0,1,2,3,4) | sequential |
| `pregrip` | 0° | −40° | −88° | −52° | *dynamic* | (0,1,2,3) | sequential |
| `grip_ready` | 0° | −52° | −95° | −33° | *dynamic* | (3,2,1) | interpolated |
| `tucked` | 0° | 30° | 60° | 30° | 0° | (0,1,2,3,4) | sequential |

### Arm Speed: Pregrip vs Default

`arm_speed_pregrip = 0.6` rad/s is used for the pregrip and wrist moves;  
the default `joint_move_speed = 0.8` rad/s is used elsewhere.

**[AUTHOR]** The slower pregrip speed (`0.6` vs `0.8` rad/s) was chosen **to avoid knocking the cuboid** and to increase positional accuracy by reducing momentum as the arm approaches the object. The value was determined empirically.

### Stall Detection and Joint Limits

`move_joint_smooth()`, lines 987–1040:

- Per step: if `|Δjoint| < stall_eps (1e-6 rad)` **and** `|error| > near_eps (3e-3 rad)` → increment `stall` counter.
- After `stall_steps = 80` consecutive stalled steps → raise `RuntimeError(stalled)`.
- Soft landing: if stalled but `|error| ≤ near_eps` → issue `setJointTargetPosition(target)` directly and exit.
- Joint limits: `sim.getJointInterval()` queried before movement; non-cyclic joints have target clamped to `[jmin + 1e-4, jmax − 1e-4]` (lines 1000–1006).

---

## (6) Wrist Alignment Math (Section 3.5)

**Function:** `compute_wrist_for_target()`, lines 1079–1138.  
Result is latched in `self._wrist_latched`; reset to `None` at the start of each pick cycle so it is recomputed per object.

### Step-by-Step Derivation

**Step 1 — Query joint limits** (lines 1088–1093)  
`sim.getJointInterval(arm[4])`: cyclic joint → limits `[−π, +π]`; non-cyclic → reported interval.

**Step 2 — Retrieve world poses** (lines 1095–1096)

```python
sens_pose = sim.getObjectPose(wrist_sensor, sim.handle_world)  # [x, y, z, qx, qy, qz, qw]
cube_pose = sim.getObjectPose(target_handle, sim.handle_world) # [x, y, z, qx, qy, qz, qw]
```

**Step 3 — Build rotation matrix from quaternion** (`_quat_to_R()`, lines 282–290)

```
R[0][0] = 1 − 2(y²+z²),   R[0][1] = 2(xy − wz),   R[0][2] = 2(xz + wy)
R[1][0] = 2(xy + wz),      R[1][1] = 1 − 2(x²+z²), R[1][2] = 2(yz − wx)
R[2][0] = 2(xz − wy),      R[2][1] = 2(yz + wx),    R[2][2] = 1 − 2(x²+y²)
```

**Step 4 — Extract world-frame axes and project to XY** (lines 1098–1100, `_proj_xy_unit()`)

```
sens_dir   = normalise_xy( R_sensor  × [1,0,0] )   # wrist sensor local X-axis
cube_dir_x = normalise_xy( R_cube    × [1,0,0] )   # cuboid local X-axis
cube_dir_y = normalise_xy( R_cube    × [0,1,0] )   # cuboid local Y-axis
```

`normalise_xy(v) = [v.x, v.y] / hypot(v.x, v.y)`  (returns `[1,0]` if `hypot < 1e-9`)

**Step 5 — Compute signed angle candidates** (`_signed_angle_2d()`, lines 308–310, 1102–1105)

```
signed_angle(a → b) = atan2( a.x·b.y − a.y·b.x,  a.x·b.x + a.y·b.y )

delta_candidates = [
    signed_angle(sens_dir → cube_dir_x),   # align to cuboid X-axis
    signed_angle(sens_dir → cube_dir_y),   # align to cuboid Y-axis
]
```

**Step 6 — Expand for 180° axis symmetry** (lines 1108–1112)

Because rectangular cuboid alignment is bidirectional, each `d` is expanded to three variants:

```
{wrap_pi(d),  wrap_pi(d + π),  wrap_pi(d − π)}
```

This yields 6 candidates total.

**Step 7 — Select minimum-rotation candidate within joint limits** (lines 1114–1127)

For each candidate `d`:
```
desired = wrap_pi(current_wrist + d)

if desired within [jmin, jmax]:
    score = |d|                            # prefer smallest rotation
else:
    desired = clamp to violated limit
    score   = 10 + |overshoot|             # penalise limit violations
```

Pick `desired` with the lowest `score`.

**Step 8 — Limit warning** (lines 1130–1135)  
If the selected angle is within 1 mrad of a limit, a log warning is emitted suggesting rotating joint 0 to gain more wrist range.

### Report-Friendly Summary

> The wrist joint (joint 4) is aligned to the target cuboid's horizontal orientation by: (1) querying the world-frame quaternion of both the wrist sensor and the cuboid; (2) building rotation matrices; (3) projecting each object's local X- and Y-axes onto the horizontal plane; (4) computing the signed rotation angle `atan2(a×b, a·b)` from the wrist's current horizontal axis to each of the cuboid's two face-aligned axes; (5) expanding each candidate by ±180° to handle axis bidirectionality; and (6) choosing the candidate that requires the smallest absolute joint rotation while staying within the physical joint limits. The result is cached for the duration of one pick cycle.

---

## (7) Gripper Close & Grasp Confirmation (Section 3.6)

**Key functions:** `close_gripper_until_confirmed()` (lines 1225–1255), `is_grip_confirmed()` (lines 1204–1209), `step_close_gripper_symmetric()` (lines 1211–1217), `open_gripper()` (lines 1219–1223).

### Mechanism

The gripper uses **two opposing screw joints** (j1 and j2 move toward each other). Each step, both joints are incremented by a fixed symmetric amount toward their close goals:

```python
j1_next = max(j1_goal,  j1 + step_j1)   # step_j1 = −0.001 m  (j1 decreases toward 0.018)
j2_next = min(j2_goal,  j2 + step_j2)   # step_j2 = +0.002 m  (j2 increases toward −0.032)
```

### Confirmed Grip Detection

```python
ok1 = j1 <= gripper_close_goal_j1 + gripper_tol   # ≤ 0.018 + 5e-4 = 0.0185 m
ok2 = j2 >= gripper_close_goal_j2 − gripper_tol   # ≥ −0.032 − 5e-4 = −0.0325 m
confirmed = ok1 AND ok2
```

Detection is **joint-position-based only** — no force/torque sensing.

**[AUTHOR]** The position-based confirmation is a deliberate design choice for real-world generalisation: a gripper that stops too far from the close position may lack sufficient or stable torque/contact force to maintain a secure hold. In this simulation there is no physics-based resistance so the gripper always reaches the target positions, but the logic is kept consistent with a real-world gripper scenario. The `gripper_close_timeout = 50 s` acts as a safety fallback in case the joint stalls before reaching the goal.

Upon confirmation, `attach_object_to_grip()` re-parents the cuboid to `gripAttach` and sets `shapeintparam_static = 1` to freeze it kinematically during carry.

### Parameter Summary

| Parameter | Value | Role |
|---|---|---|
| `gripper_open_j1` | `0.025` m | Open position, joint 1 |
| `gripper_open_j2` | `−0.050` m | Open position, joint 2 |
| `gripper_close_goal_j1` | `0.018` m | Close target, joint 1 |
| `gripper_close_goal_j2` | `−0.032` m | Close target, joint 2 |
| `gripper_step_j1` | `−0.001` m/tick | Joint 1 closure step per tick |
| `gripper_step_j2` | `+0.002` m/tick | Joint 2 closure step per tick |
| `gripper_tol` | `5e-4` m | Position tolerance for confirmed grip |
| `gripper_close_dt` | `0.05` s | Time between closure ticks |
| `gripper_close_timeout` | `50` s | Safety timeout → exception |

---

## (8) Drop-Zone Visual Servoing + Safety Reset + Edge Detection (Section 3.7)

### Camera Image Acquisition

`get_front_cam_rgb()`, lines 1335–1348:

1. `sim.handleVisionSensor(front_cam)` — forces a fresh image capture this step.
2. `img_bytes, res = sim.getVisionSensorImg(front_cam)` — returns raw bytes and `[w, h]`.
3. `rgb = sim.unpackUInt8Table(img_bytes)` — flat `[R, G, B, R, G, B, …]` list, values 0–255.

### Centroid Computation (`_centroid_rgb_threshold()`, lines 1351–1390)

Scans a configurable vertical band (`gp_y0_frac = 0.0` to `gp_y1_frac = 1.0`, i.e., the whole image) stepping by `gp_stride = 1`. For each pixel `(x, y)`:

```
accept if  rmin ≤ R ≤ rmax  AND  gmin ≤ G ≤ gmax  AND  bmin ≤ B ≤ bmax
```

Accumulates `count` and `xsum`; returns `(xsum / count, count)`.

**Green thresholds:** R ∈ [0, 80], G ∈ [180, 255], B ∈ [0, 80]  
**Red thresholds:** R ∈ [180, 255], G ∈ [0, 80], B ∈ [0, 80]  
**Minimum pixels:** `gp_min_pixels = 40`; detection accepted only if `count ≥ 40`.

### Detection Logic (`detect_goalposts()`, lines 1396–1433)

```python
seen_g = (n_g >= 40)
seen_r = (n_r >= 40)
if seen_g and seen_r:
    sep    = cx_r - cx_g                 # inter-post pixel separation
    center = 0.5 * (cx_g + cx_r)        # gate midpoint
    err_px = center - (w / 2.0)         # +ve = gate is to the right
    return True, err_px, w, info
```

### Control Law: Pixel Error → Omega (Stage A — both posts visible)

```
err_norm = err_px / (w × 0.5)                                    # normalised to [−1, +1]
omega    = clamp(0.6 × err_norm, −0.35, +0.35) × gp_omega_sign
```

### Forward-Drive Policy

| Condition | `vx` |
|---|---|
| `\|err_px\| > gp_drive_tol_px (40 px)` | 0 (align first) |
| `\|err_px\| ≤ 40 px` | `vx_sign × 0.5 × 0.10 = 0.05 m/s` (creep) |
| stable ≥ 2 AND `\|err_px\| ≤ 16 px` | `vx_sign × 0.10 m/s` (full speed) + timed step of `0.25 s` |

### Terminal Approach — When Posts No Longer Both Fit in Frame (Stage B)

Once both posts have been acquired and `seen_both` becomes `False`:

- **HOLD phase** (`time_since_last_sighting ≤ gp_lost_hold_s = 1.2 s`): continues driving at `gp_final_forward_speed = 0.08 m/s` using the last known pixel error. If one post is still visible, the gate centre is estimated from `_gp_last_sep_px` (exponentially smoothed: `α = gp_sep_decay = 0.7`).
- **Transition** (`time_since_last_sighting > 1.2 s` and posts were previously acquired): calls `drive_to_floor_edge()` to complete the approach by proximity sensor.
- **Near-stop heuristic**: if a single post exceeds `gp_near_pixels = 800` px, the robot stops and backs off immediately (treated as "close enough").

### Two Safety Reset Triggers

Both are gated on `(elapsed < gp_min_approach_s = 2.0 s  OR  _gp_both_seen_valid = False)  AND  attempt < max_retries = 3`:

| Trigger | Condition | Location |
|---|---|---|
| **Early edge** | Floor sensor fires before time/separation gates cleared | Lines 650–661 |
| **Hold-timeout** | Hold window expires without valid both-post confirmation | Lines 799–810 |
| **Near-stop early** | Single post pixel count ≥ 800 before approach validated | Lines 716–728 |

**Minimum gate separation gate:** `gp_min_sep_frac = 0.25` — both posts must simultaneously span at least 25 % of the image width. This ensures the robot is roughly centred in front of the gate rather than viewing it from a steep side angle.  
**[AUTHOR]** This threshold was **manually tuned by trial and error**.

**Return-home function:** `return_base_to_world_origin()` (lines 978–981):
```python
turn_to_face_target(home_dummy)              # face world origin (0, 0)
drive_to_stop(home_dummy, stop_distance=0.05) # drive to within 5 cm of origin
```
`home_dummy` is a CoppeliaSim dummy object placed at world `(0, 0, 0)` during initialisation.

### Edge Detection and Drop Sequence (`drive_to_floor_edge()`, lines 940–976)

1. Drive forward at `edge_drive_speed = 0.10 m/s`.
2. Each step: `floor_detected()` calls `sim.checkProximitySensor(edge_sensor, floor_entity)`.
3. Floor present → `lost = 0`; floor absent → `lost += 1`.
4. When `lost ≥ edge_lost_steps (6)` → stop. (6 consecutive "no floor" frames = debouncing.)
5. Back off: `_drive_for_duration(−0.06, 0.3 s)` (robot retreats so only the cube falls).
6. **Drop sequence** (`drop_cuboid_off_edge()`, lines 1319–1328): wait `edge_drop_pause_s = 0.2 s` → `open_gripper()` (releases object to physics) → reverse at `−0.04 m/s` for `1.0 s`.

> **Note on double `drive_to_floor_edge()` call:** In `pick_and_drop_one()` (lines 1466–1467), `drive_to_dropzone_visual()` is followed immediately by an unconditional call to `drive_to_floor_edge()`. **[AUTHOR]** This is a known code bug (the visual-servo function already calls `drive_to_floor_edge()` internally on the genuine approach path). It has **no functional impact** because by the time the outer `drive_to_floor_edge()` runs, the robot is already at or past the edge, so the sensor immediately fires and the function exits in one or two steps.

---

## (9) Results Instrumentation Hooks (Section 4)

### Existing Logging Infrastructure

- **`StepLogger`** (lines 233–247): Every major action emits `[STEP START]` and `[STEP DONE]` / `[STEP FAIL]` to both stdout and `sim.addLog`. All navigation, arm, gripper, and mission stages are wrapped.
- **`log_info()`**: Used throughout for per-step debug data (drive distance, gripper positions, pixel counts, etc.).
- **Mission-level logs** (lines 1511–1522): Cube path, distance, attempt count, and list of collected aliases logged after each cube.

### Extractable Metrics from Current Logs

| Metric | Log pattern to match |
|---|---|
| Pick success / failure per cube | `[STEP DONE ] FULL PICK SEQUENCE` vs `[STEP FAIL ]` |
| Visual approach retry count | `[gp] Returning to home position before retry (attempt N/M)` |
| Gripper confirmation achieved | `grip confirmed at j1=... j2=...` |
| Wrist angle used | `wrist target (rad): ...` |
| Final approach proximity | `[drive_to_stop] arrived: proximity=...m` |
| Per-cube time | timestamp delta between matching `[STEP START]` and `[STEP DONE/FAIL]` |

### Suggested Minimal Log Additions (no behaviour change)

Insert these lines in `run_mission_loop()` and `compute_wrist_for_target()`:

```python
# 1. Before pick_and_drop_one():
t_pick_start = sim.getSimulationTime()
log_info(f"[METRICS] pick_start cube={best_path} t={t_pick_start:.3f}")

# 2. After collected_cubes.append() (success branch):
log_info(f"[METRICS] pick_success cube={best_path} duration={sim.getSimulationTime()-t_pick_start:.2f}s")

# 3. In the except block (failure branch):
log_info(f"[METRICS] pick_failure cube={best_path} duration={sim.getSimulationTime()-t_pick_start:.2f}s err={e}")

# 4. In compute_wrist_for_target(), before return:
log_info(f"[METRICS] wrist_angle_deg={math.degrees(desired):.1f} score={best[0]:.4f} at_limit={'yes' if desired<=jmin+1e-3 or desired>=jmax-1e-3 else 'no'}")

# 5. In drive_to_dropzone_visual(), when _gp_both_seen_valid becomes True:
log_info(f"[METRICS] gp_valid_seen sep_px={sep:.1f} img_w={img_w} frac={sep/img_w:.3f}")
```

---

## Appendix A — Resolved Questions Summary

| # | Question | Answer (from author) |
|---|---|---|
| 1 | Double `drive_to_floor_edge()` call | Known bug; no functional impact — robot is already past edge when outer call runs |
| 4 | Why `vy = 0` always? | Intentional: straight-on approach simplifies gripper alignment (always face-on) |
| 5 | Are cuboids randomly placed? | Yes — randomly placed each run; greedy nearest-first handles variable layouts |
| 6 | `checkDistance` index `distData[-1]` | Uses CoppeliaSim simulation data directly; verified correct for this scene |
| 7 | Why slower `arm_speed_pregrip = 0.6`? | To avoid knocking the cuboid and improve accuracy by reducing approach momentum |
| 8 | How was `gp_min_sep_frac = 0.25` tuned? | Manually, by trial and error |
| 9 | Origin of `lx = 0.228`, `ly = 0.158`? | Taken directly from the CoppeliaSim youBot model geometry |
| 10 | Gripper position-based confirmation (not force)? | Deliberate: real-world grippers too far open may lack stable torque/force; timeout is the safety net |

## Appendix B — Remaining Open Items

1. **Camera resolution (`w × h`):** Not defined in code; set by the CoppeliaSim vision sensor properties in the scene. Pixel-space thresholds (`gp_align_tol_px`, `gp_min_sep_frac`) are only fully interpretable in angular terms once `w` is known.
2. **Wrist sensor physical orientation:** The code hardcodes `local_axis = 0` (X-axis) in `compute_wrist_for_target()`. This must be consistent with the `wristOrientSensor` geometry in the CoppeliaSim scene.
3. **Actual arm joint limits:** Queried at runtime; not listed in `CFG`. Relevant for understanding wrist reachability and the frequency of limit-warning log messages.
