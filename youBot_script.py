# Threaded simulation script (Python) for CoppeliaSim
# Modular youBot pick sequence with robust object-path handling

import math
import traceback


# -----------------------------
# Configuration (use object PATHS where possible)
# -----------------------------
CFG = {
    # Use absolute path for scene-root objects:
    "cuboid_path": "/Cuboid",

    # Use model-relative paths for objects inside the youBot model:
    "base_ref_path": ":/Rectangle0",

    "grip_attach_path": ":/gripAttach",

    "arm_joint_paths": [
        ":/youBotArmJoint0",
        ":/youBotArmJoint1",
        ":/youBotArmJoint2",
        ":/youBotArmJoint3",
        ":/youBotArmJoint4",  # wrist
    ],
    "gripper_joint_paths": [
        ":/youBotGripperJoint1",
        ":/youBotGripperJoint2",
    ],
    "wheel_joint_paths": [
        ":/rollingJoint_fl",
        ":/rollingJoint_rl",
        ":/rollingJoint_rr",
        ":/rollingJoint_fr",
    ],

    "pick_targets": [
        "/Cuboid", "/Cuboid0", "/Cuboid1", "/Cuboid2",
        "/Cuboid3", "/Cuboid4", "/Cuboid5", "/Cuboid6",
    ],

    # Behavior params
    "stop_distance": 0.16,
    "dt": 0.02,

    # Base turn-then-drive settings:
    "forward_axis": "y",         # "x" or "y" axis of youBot_ref that points "forward"
    "forward_sign":  1.0,        # flip to -1 if forward seems backwards
    "omega_sign":    1.0,        # flip to -1 if turning goes the wrong way

    "ang_kp":  0.8,              # rotation gain
    "ang_max": 0.6,              # max yaw rate command (rad/s equiv input to wheels_from_twist)

    "face_tol_deg": 8.0,         # stop rotating when within this angle
    "face_stable_steps": 10,     # stable steps required for turn_to_face_target convergence
    "drive_angle_gate_deg": 10.0,# only drive forward if facing within this angle

    "approach_speed": 0.15,
    "wheel_speed_max": 5,
    "approach_kp": 1.0,
    "wheel_dir": [1, 1, 1, 1],   # flip any wheel by changing to -1

    "vx_sign": -1.0,             # flip forward/backward drive; -1.0 fixes "drives backwards"

    "stop_band": 0.03,           # extra margin (m): stop when dist <= stop_distance + stop_band
    "stop_stable_steps": 10,     # must be inside band for N steps before stopping
    "max_drive_time": 60,        # safety timeout (seconds)

    "clearance_check_threshold": 5.0,  # distance threshold for checkDistance call (m)
    "log_every_steps": 25,             # log every N steps in drive/turn loops
    "post_stop_settle_s": 0.1,         # settle pause after stop_base()
    "post_arm_settle_s": 0.05,         # settle pause after arm moves

    "grip_ready_duration_s": 1.9,  # total time for the grip-ready descent move
    "grip_ready_dt": 0.02,         # step time for grip-ready interpolation

    # Mecanum/omni constants (tune if needed)
    "wheel_radius": 0.0475,
    "lx_plus_ly": 0.228 + 0.158,

    # Motion smoothing
    "joint_move_speed": 0.8,
    "joint_eps": 1e-3,
    "joint_move_timeout_s": 100,
    "joint_stall_eps": 1e-6,
    "joint_stall_steps": 80,

    # --- Gripper (screw joints) ---
    "gripper_open_j1":  0.025,
    "gripper_open_j2": -0.050,

    "gripper_close_goal_j1":  0.018,
    "gripper_close_goal_j2": -0.032,

    # per-tick symmetric closure steps
    "gripper_step_j1": -0.001,   # reduce joint1
    "gripper_step_j2":  0.002,   # increase joint2

    "gripper_tol": 5e-4,         # position tolerance for "confirmed"
    "gripper_close_dt": 0.05,    # seconds between steps
    "gripper_close_timeout": 50, # safety timeout

    # Pause at pregrip (seconds)
    "pregrip_pause_s": 2.5,

    "arm_speed_pregrip": 0.6,    # rad/s (slower = smoother)

    # --- Edge drop ---
    "edge_sensor_path": ":/edgeSensorDown",  # down-facing proximity sensor near front
    "floor_entity_path": "/Floor",           # optional: floor shape path
    "edge_drive_speed": 0.10,               # m/s forward while searching edge
    "edge_timeout_s": 60,                   # safety timeout
    "edge_lost_steps": 6,                   # require N consecutive "no floor" frames
    "edge_backoff_s": 0.3,                  # seconds to back off after edge found
    "edge_backoff_speed": -0.06,            # m/s (negative = backwards) for backoff
    "edge_drop_pause_s": 0.2,              # settle pause before dropping

    "edge_reverse_s": 1.0,        # reverse duration after drop
    "edge_reverse_speed": -0.04,  # m/s (negative = reverse)

    "reset_base_xy": [0.0, 0.0],   # world origin target
    "reset_base_yaw_deg": 0.0,     # yaw at reset
    
    
    # --- Vision-guided drop zone (goalposts) ---
    "front_cam_path": ":/frontCam",  # if the vision sensor is parented under the youBot model
    # If not inside the model tree, use "/frontCam" instead.
    
    "gp_y0_frac": 0.0,
    "gp_y1_frac": 1.0,
    "gp_stride": 1,
    "gp_min_pixels": 40,


    # Color thresholds in RGB (0..255) for PURE colored posts:
    "gp_green_min": [0, 180, 0],    # green: G high, R/B low
    "gp_green_max": [80, 255, 80],

    "gp_red_min":   [180, 0, 0],    # red: R high, G/B low
    "gp_red_max":   [255, 80, 80],

    # Minimum number of pixels required to accept detection:
    

    # Visual servo control:
    "gp_kp_omega": 0.6,             # turning gain from pixel error
    "gp_max_omega": 0.35,            # rad/s command into wheels_from_twist

    # Step-drive behaviour:
    "gp_forward_speed": 0.10,       # m/s forward when aligned
    "gp_step_time": 0.25,           # seconds per forward step
    "gp_align_tol_px": 16,          # must centre gate within this pixel tolerance
    "gp_align_stable_steps": 2,     # stable frames required for "locked-on"
    "gp_search_omega": 0.25,        # rad/s spin when posts not visible
    "gp_timeout_s": 200,             # overall timeout
    "gp_omega_sign": 1.0,   # flip to -1.0 if it steers away from the posts
    "gp_drive_tol_px": 40,

    # --- Terminal approach (Stage B) ---
    "gp_lost_hold_s": 1.2,          # seconds to keep driving after both posts disappear
    "gp_near_pixels": 800,          # single-post pixel count that triggers near-stop
    "gp_final_forward_speed": 0.08, # forward speed during terminal hold phase
    "gp_sep_decay": 0.7,            # smoothing factor for stored inter-post separation

}


# -----------------------------
# Pose registry
# All joint angles stored in degrees; converted once at use via deg_pose_to_rad().
# wrist_deg=None means the wrist angle is computed dynamically per target.
# "mode" drives which movement primitive is used.
# -----------------------------
POSES = {
    "neutral": {
        "joints_deg": [0, -40, -50, 0],
        "wrist_deg": 0.0,
        "order": (0, 1, 2, 3, 4),
        "mode": "sequential",
    },
    "pregrip": {
        "joints_deg": [0, -40, -88, -52],
        "wrist_deg": None,   # wrist computed dynamically
        "order": (0, 1, 2, 3),
        "mode": "sequential",
    },
    "grip_ready": {
        "joints_deg": [0, -52, -95, -33],
        "wrist_deg": None,   # wrist computed dynamically
        "order": (3, 2, 1),
        "mode": "interpolated",
    },
    "tucked": {
        "joints_deg": [0, 30, 60, 30],
        "wrist_deg": 0.0,
        "order": (0, 1, 2, 3, 4),
        "mode": "sequential",
    },
}


# -----------------------------
# Logging helpers
# -----------------------------
def log_info(msg: str):
    print(msg)
    try:
        sim.addLog(sim.verbosity_scriptinfos, msg)
    except Exception:
        pass


class StepLogger:
    """Context manager for consistent START/DONE/FAIL logs."""
    def __init__(self, name: str):
        self.name = name

    def __enter__(self):
        log_info(f"[STEP START] {self.name}")
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc_type is None:
            log_info(f"[STEP DONE ] {self.name}")
        else:
            log_info(f"[STEP FAIL ] {self.name} -> {exc_type.__name__}: {exc}")
        return False


# -----------------------------
# Math helpers
# -----------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def deg2rad(deg):
    return deg * math.pi / 180.0


def deg_pose_to_rad(joints_deg, wrist_rad=None):
    """Convert a 4-element degrees list to a 5-element radians list.

    joints_deg: [j0, j1, j2, j3] in degrees
    wrist_rad:  joint4 value in radians; defaults to 0.0 when None
    """
    result = [deg2rad(d) for d in joints_deg]
    result.append(0.0 if wrist_rad is None else wrist_rad)
    return result


# --- Wrist orientation geometry (module-level, no side-effects) ---

def _wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _quat_to_R(qx, qy, qz, qw):
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return [
        [1 - 2*(yy + zz),   2*(xy - wz),     2*(xz + wy)],
        [2*(xy + wz),       1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy),       2*(yz + wx),     1 - 2*(xx + yy)],
    ]


def _axis_world_from_pose(pose, local_axis):
    """Return a world-frame axis vector from a [x y z qx qy qz qw] pose."""
    qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
    R = _quat_to_R(qx, qy, qz, qw)
    return [R[0][local_axis], R[1][local_axis], R[2][local_axis]]


def _proj_xy_unit(v):
    x, y = v[0], v[1]
    n = math.hypot(x, y)
    if n < 1e-9:
        return [1.0, 0.0]
    return [x / n, y / n]


def _signed_angle_2d(a, b):
    """Signed angle (radians) from 2-D unit vector a to unit vector b."""
    return math.atan2(a[0]*b[1] - a[1]*b[0], a[0]*b[0] + a[1]*b[1])


# -----------------------------
# Controller
# -----------------------------
class YouBotPickController:

    # -------------------------------------------------------------------------
    # Initialisation
    # -------------------------------------------------------------------------

    def __init__(self):
        self.model        = None
        self.cuboid       = None   # init-time default; not mutated during mission
        self.base_ref     = None
        self.arm          = []
        self.gripper      = []
        self.wheels       = []
        self.grip_attach  = None
        self.wrist_sensor = None
        self.edge_sensor  = None
        self.floor_entity = None
        self.home_dummy   = None
        self.front_cam    = None

        self.attached_object       = None
        self._attached_prev_parent = None
        self._attached_prev_static = None
        self._wrist_latched        = None
        self._arm_moved            = False

        self.collected_cubes = []   # handles of cubes already picked up

        # Visual drop-zone navigation state (Stage B terminal approach)
        self._gp_last_seen_t  = -1.0   # sim time when both posts were last seen
        self._gp_last_err_px  = 0.0    # last known gate-centre pixel error
        self._gp_last_sep_px  = None   # smoothed inter-post separation (pixels)

    def init_handles(self):
        with StepLogger("Init handles"):
            self.model = sim.getObject(":")
            try:
                model_path = sim.getObjectAlias(self.model, 2)
            except Exception:
                model_path = "<unknown>"
            log_info(f"  Model root: {model_path}")

            self.cuboid   = self.get_object_strict(CFG["cuboid_path"],   label="Cuboid")
            self.base_ref = self.get_object_strict(CFG["base_ref_path"], label="Base ref (youBot_ref)")

            self.arm     = [self.get_object_strict(p, label=f"Arm joint {i}")       for i, p in enumerate(CFG["arm_joint_paths"])]
            self.gripper = [self.get_object_strict(p, label=f"Gripper joint {i+1}") for i, p in enumerate(CFG["gripper_joint_paths"])]
            self.wheels  = [self.get_object_strict(p, label=f"Wheel {i}")           for i, p in enumerate(CFG["wheel_joint_paths"])]

            self.grip_attach  = self.get_object_strict(CFG["grip_attach_path"],  label="Grip attach dummy")
            self.wrist_sensor = self.get_object_strict(":/wristOrientSensor",    label="Wrist orient sensor")
            self.edge_sensor  = self.get_object_strict(CFG["edge_sensor_path"],  label="Edge sensor (down)")
            self.front_cam = self.get_object_strict(CFG["front_cam_path"], label="Front vision sensor (frontCam)")
            # Floor entity (optional)
            if CFG.get("floor_entity_path"):
                h = sim.getObject(CFG["floor_entity_path"], {"noError": True})
                self.floor_entity = h if h != -1 else None

            # Home dummy ? reuse existing to avoid leaking scene objects on re-init
            existing = sim.getObject("/HOME_0_0", {"noError": True})
            if existing != -1:
                self.home_dummy = existing
            else:
                self.home_dummy = sim.createDummy(0.01)
                sim.setObjectAlias(self.home_dummy, "HOME_0_0", 0)
            sim.setObjectPosition(self.home_dummy, [0.0, 0.0, 0.0], sim.handle_world)

            # Ensure wheels stopped at init
            for w in self.wheels:
                sim.setJointTargetVelocity(w, 0.0)

    # -------------------------------------------------------------------------
    # Object access
    # -------------------------------------------------------------------------

    def get_object_strict(self, path: str, label="Object"):
        """Robust object fetch: tries path directly, then common prefixes."""
        if path and path[0] in ["/", ".", ":"]:
            h = sim.getObject(path, {"noError": True})
            if h != -1:
                return h
            raise RuntimeError(f"{label} not found at path '{path}'")

        candidates = [f":/{path}", f"./{path}", f"/{path}"]
        for c in candidates:
            h = sim.getObject(c, {"noError": True})
            if h != -1:
                log_info(f"  Resolved {label}: '{path}' -> '{c}'")
                return h

        raise RuntimeError(f"{label} not found. Tried: {candidates}")

    # -------------------------------------------------------------------------
    # Geometry & sensing
    # -------------------------------------------------------------------------

    def forward_lateral(self, rel):
        """Decompose a base-frame relative position into (forward, lateral)."""
        if CFG["forward_axis"] == "x":
            forward, lateral = rel[0], rel[1]
        else:  # "y"
            forward, lateral = rel[1], rel[0]
        forward *= CFG["forward_sign"]
        return forward, lateral

    def heading_error(self, rel):
        forward, lateral = self.forward_lateral(rel)
        return math.atan2(lateral, forward)

    def get_clearance_to(self, target_handle):
        """Return closest clearance to target_handle via checkDistance, or inf."""
        threshold = CFG["clearance_check_threshold"]
        result, distData, objPair = sim.checkDistance(self.base_ref, target_handle, threshold)
        if result == 1:
            return distData[-1]
        if result == -1:
            log_info("  [get_clearance_to] checkDistance API error (result=-1); treating as inf")
        return float('inf')

    def floor_detected(self):
        """Returns (detected: bool, dist: float|None)."""
        entity = self.floor_entity if self.floor_entity is not None else sim.handle_all
        res, dist, point, obj, n = sim.checkProximitySensor(self.edge_sensor, entity)
        if res == 1:
            return True, dist
        return False, None

    # -------------------------------------------------------------------------
    # Base movement ? primitives
    # -------------------------------------------------------------------------

    def wheels_from_twist(self, vx, vy, omega):
        r = CFG["wheel_radius"]
        L = CFG["lx_plus_ly"]
        w_fl = (vx - vy - L * omega) / r
        w_rl = (vx + vy - L * omega) / r
        w_rr = (vx - vy + L * omega) / r
        w_fr = (vx + vy + L * omega) / r
        ws = [w_fl, w_rl, w_rr, w_fr]
        ws = [d * w for d, w in zip(CFG["wheel_dir"], ws)]
        return ws

    def stop_base(self):
        for h in self.wheels:
            sim.setJointTargetVelocity(h, 0.0)

    def _drive_for_duration(self, vx: float, duration_s: float):
        """Drive at a fixed signed forward speed (vx) for duration_s seconds, then stop.

        vx is treated as a signed velocity in the robot's natural frame; CFG["vx_sign"]
        is applied internally so callers can use positive=forward / negative=reverse
        convention consistently with all other drive methods.
        """
        wmax  = CFG["wheel_speed_max"]
        t_end = sim.getSimulationTime() + duration_s
        while not sim.getSimulationStopping() and sim.getSimulationTime() < t_end:
            ws = self.wheels_from_twist(CFG["vx_sign"] * vx, 0.0, 0.0)
            ws = [clamp(w, -wmax, wmax) for w in ws]
            for h, w in zip(self.wheels, ws):
                sim.setJointTargetVelocity(h, w)
            sim.wait(CFG["dt"])
        self.stop_base()

    # -------------------------------------------------------------------------
    # Base movement ? navigation actions
    # -------------------------------------------------------------------------

    def turn_to_face_target(self, target_handle, label="Turn to face target"):
        with StepLogger(label):
            dt            = CFG["dt"]
            tol           = math.radians(CFG["face_tol_deg"])
            wmax          = CFG["wheel_speed_max"]
            stable_needed = CFG["face_stable_steps"]
            stable_count  = 0

            while not sim.getSimulationStopping():
                rel = sim.getObjectPosition(target_handle, self.base_ref)
                err = self.heading_error(rel)

                if abs(err) <= tol:
                    stable_count += 1
                    if stable_count >= stable_needed:
                        break
                    self.stop_base()
                    sim.wait(dt)
                    continue
                else:
                    stable_count = 0

                omega  = clamp(CFG["ang_kp"] * err, -CFG["ang_max"], CFG["ang_max"])
                omega *= CFG["omega_sign"]

                ws = self.wheels_from_twist(0.0, 0.0, omega)
                ws = [clamp(w, -wmax, wmax) for w in ws]
                for h, w in zip(self.wheels, ws):
                    sim.setJointTargetVelocity(h, w)

                sim.wait(dt)

            self.stop_base()
            sim.wait(CFG["post_stop_settle_s"])

    def drive_to_stop(self, target_handle, stop_distance, use_clearance=False,
                      label="Drive to target"):
        """Drive toward target_handle and halt at stop_distance.

        use_clearance=True  ? stop criterion is checkDistance clearance
                              (used for cuboid approach, more accurate near objects).
        use_clearance=False ? stop criterion is planar getObjectPosition distance
                              (used for home navigation).
        """
        with StepLogger(label):
            dt            = CFG["dt"]
            vmax          = CFG["approach_speed"]
            wmax          = CFG["wheel_speed_max"]
            gate          = math.radians(CFG["drive_angle_gate_deg"])
            band          = CFG["stop_band"]
            stable_needed = CFG["stop_stable_steps"]
            stable        = 0
            log_every     = CFG["log_every_steps"]
            k             = 0

            t_start  = sim.getSimulationTime()
            max_time = CFG["max_drive_time"]

            while not sim.getSimulationStopping():
                if sim.getSimulationTime() - t_start > max_time:
                    log_info("  [drive_to_stop] timeout reached -> stopping")
                    break

                rel              = sim.getObjectPosition(target_handle, self.base_ref)
                forward, lateral = self.forward_lateral(rel)

                if use_clearance:
                    proximity = self.get_clearance_to(target_handle)
                else:
                    proximity = math.hypot(forward, lateral)

                if proximity <= (stop_distance + band):
                    stable += 1
                    self.stop_base()
                    if stable >= stable_needed:
                        log_info(f"  [drive_to_stop] arrived: proximity={proximity:.3f}m")
                        break
                    sim.wait(dt)
                    continue
                else:
                    stable = 0

                err_ang = math.atan2(lateral, forward)
                vx = 0.0
                if abs(err_ang) <= gate:
                    err_lin = proximity - stop_distance
                    v  = clamp(CFG["approach_kp"] * err_lin, 0.0, vmax)
                    vx = CFG["vx_sign"] * v

                omega  = clamp(CFG["ang_kp"] * err_ang, -CFG["ang_max"], CFG["ang_max"])
                omega *= CFG["omega_sign"]

                if k % log_every == 0:
                    log_info(
                        f"  drive: proximity={proximity:.3f} "
                        f"forward={forward:.3f} lat={lateral:.3f} "
                        f"err_deg={math.degrees(err_ang):.1f} vx={vx:.3f} om={omega:.3f}"
                    )
                k += 1

                ws = self.wheels_from_twist(vx, 0.0, omega)
                ws = [clamp(w, -wmax, wmax) for w in ws]
                for h, w in zip(self.wheels, ws):
                    sim.setJointTargetVelocity(h, w)

                sim.wait(dt)

            self.stop_base()
            sim.wait(CFG["post_stop_settle_s"])
            
            

    def drive_to_dropzone_visual(self):
        with StepLogger("Drive to drop zone using frontCam + goalposts"):
            dt      = CFG["dt"]
            wmax    = CFG["wheel_speed_max"]
            t0      = sim.getSimulationTime()
            timeout = CFG["gp_timeout_s"]

            tol_px        = CFG["gp_align_tol_px"]
            stable_needed = CFG["gp_align_stable_steps"]
            stable        = 0

            # Edge confirmation (avoid 1-frame flicker)
            lost_needed = CFG["edge_lost_steps"]
            lost = 0

            # Search robustness: flip scan direction if we can't acquire both posts
            scan_dir = 1.0
            last_flip_t = sim.getSimulationTime()
            flip_every_s = 6.0

            # Terminal-approach parameters
            gp_lost_hold_s         = CFG["gp_lost_hold_s"]
            gp_near_pixels         = CFG["gp_near_pixels"]
            gp_final_forward_speed = CFG["gp_final_forward_speed"]
            gp_sep_decay           = CFG["gp_sep_decay"]

            # Reset per-approach state so stale data from a previous cube cycle
            # cannot trigger a premature edge-drive transition.
            self._gp_last_seen_t  = -1.0
            self._gp_last_err_px  = 0.0
            self._gp_last_sep_px  = None

            while not sim.getSimulationStopping():
                if sim.getSimulationTime() - t0 > timeout:
                    raise RuntimeError("Vision dropzone timeout: could not reach/align with goalposts.")

                # --- Primary stop condition: edge reached (with confirmation) ---
                detected, _ = self.floor_detected()
                if detected:
                    lost = 0
                else:
                    lost += 1

                if lost >= lost_needed:
                    self.stop_base()
                    sim.wait(CFG["post_stop_settle_s"])
                    if CFG["edge_backoff_s"] > 0:
                        self._drive_for_duration(CFG["edge_backoff_speed"], CFG["edge_backoff_s"])
                        sim.wait(CFG["post_stop_settle_s"])
                    return

                # --- Vision sensing ---
                seen_both, err_px, img_w, info = self.detect_goalposts()

                t_now = sim.getSimulationTime()

                # -------------------------------------------------------
                # 1) Update "last seen" memory when both posts are visible
                # -------------------------------------------------------
                if seen_both:
                    self._gp_last_seen_t = t_now
                    self._gp_last_err_px = err_px
                    sep = info["sep"]
                    if self._gp_last_sep_px is None:
                        self._gp_last_sep_px = sep
                    else:
                        self._gp_last_sep_px = (
                            gp_sep_decay * self._gp_last_sep_px
                            + (1.0 - gp_sep_decay) * sep
                        )

                # -------------------------------------------------------
                # 2) Secondary near-stop heuristic (single post, big pixel count)
                # -------------------------------------------------------
                if not seen_both:
                    seen_g = info.get("seen_g", False)
                    seen_r = info.get("seen_r", False)
                    n_g    = info.get("n_g", 0)
                    n_r    = info.get("n_r", 0)
                    one_post_big = (
                        (seen_g and not seen_r and n_g >= gp_near_pixels) or
                        (seen_r and not seen_g and n_r >= gp_near_pixels)
                    )
                    if one_post_big:
                        log_info(
                            f"[gp] Near-stop: single post pixel count "
                            f"n_g={n_g} n_r={n_r} >= {gp_near_pixels}"
                        )
                        self.stop_base()
                        sim.wait(CFG["post_stop_settle_s"])
                        if CFG["edge_backoff_s"] > 0:
                            self._drive_for_duration(CFG["edge_backoff_speed"], CFG["edge_backoff_s"])
                            sim.wait(CFG["post_stop_settle_s"])
                        return

                # -------------------------------------------------------
                # 3) Not seen_both: HOLD or SEARCH
                # -------------------------------------------------------
                if not seen_both:
                    # Flip scan direction occasionally when fully lost
                    if (not info.get("seen_g")) and (not info.get("seen_r")):
                        if (t_now - last_flip_t) > flip_every_s:
                            scan_dir *= -1.0
                            last_flip_t = t_now

                    time_since_seen = t_now - self._gp_last_seen_t

                    if time_since_seen <= gp_lost_hold_s:
                        # --- HOLD / TERMINAL APPROACH ---
                        seen_g = info.get("seen_g", False)
                        seen_r = info.get("seen_r", False)
                        cx_g   = info.get("cx_g")
                        cx_r   = info.get("cx_r")

                        half_img_w = img_w / 2.0
                        if self._gp_last_sep_px is not None:
                            if seen_g and not seen_r and cx_g is not None:
                                center_est = cx_g + self._gp_last_sep_px / 2.0
                                err_est = center_est - half_img_w
                            elif seen_r and not seen_g and cx_r is not None:
                                center_est = cx_r - self._gp_last_sep_px / 2.0
                                err_est = center_est - half_img_w
                            else:
                                err_est = self._gp_last_err_px
                        else:
                            err_est = self._gp_last_err_px

                        err_norm = err_est / max(1.0, half_img_w)
                        omega = clamp(
                            CFG["gp_kp_omega"] * err_norm,
                            -CFG["gp_max_omega"], CFG["gp_max_omega"]
                        )
                        omega *= CFG["gp_omega_sign"]

                        vx = CFG["vx_sign"] * gp_final_forward_speed

                        ws = self.wheels_from_twist(vx, 0.0, omega)
                        ws = [clamp(w, -wmax, wmax) for w in ws]
                        for h, w in zip(self.wheels, ws):
                            sim.setJointTargetVelocity(h, w)

                        sim.wait(dt)
                        continue

                    else:
                        # --- SEARCH (or transition to edge drive) ---
                        # If we previously acquired both posts but have now lost them
                        # beyond the hold window, we must be close to the gate.
                        # Transition straight to drive_to_floor_edge() so the cube is
                        # dropped correctly when the sensor passes over the edge.
                        if self._gp_last_seen_t > 0:
                            log_info(
                                "[gp] Both posts lost beyond hold window (previously "
                                "acquired) ? transitioning to drive_to_floor_edge"
                            )
                            self.stop_base()
                            sim.wait(CFG["post_stop_settle_s"])
                            self.drive_to_floor_edge()
                            return

                        # Posts were never acquired ? keep spinning to find them
                        stable = 0
                        omega = abs(CFG["gp_search_omega"])

                        # Alternate direction only when no posts at all
                        if (not info.get("seen_g")) and (not info.get("seen_r")):
                            omega *= scan_dir

                        # Base rotation sign for scan
                        omega *= CFG["omega_sign"]

                        ws = self.wheels_from_twist(0.0, 0.0, omega)
                        ws = [clamp(w, -wmax, wmax) for w in ws]
                        for h, w in zip(self.wheels, ws):
                            sim.setJointTargetVelocity(h, w)

                        sim.wait(dt)
                        continue

                # -------------------------------------------------------
                # 4) seen_both: normal ALIGN + forward policy (Stage A)
                # -------------------------------------------------------

                # Stable-frame counter with decay
                if abs(err_px) <= tol_px:
                    stable = min(stable + 1, stable_needed)
                else:
                    stable = max(stable - 1, 0)

                # Closed-loop omega from pixel error
                err_norm = err_px / max(1.0, img_w * 0.5)
                omega = clamp(
                    CFG["gp_kp_omega"] * err_norm,
                    -CFG["gp_max_omega"], CFG["gp_max_omega"]
                )
                omega *= CFG["gp_omega_sign"]

                # Forward motion policy
                drive_tol = CFG.get("gp_drive_tol_px", 3 * tol_px)
                err_abs   = abs(err_px)

                if err_abs <= drive_tol:
                    vx = CFG["vx_sign"] * (0.5 * CFG["gp_forward_speed"])  # creep
                else:
                    vx = 0.0

                # Full speed when fully stable
                if stable >= stable_needed and err_abs <= tol_px:
                    vx = CFG["vx_sign"] * CFG["gp_forward_speed"]

                ws = self.wheels_from_twist(vx, 0.0, omega)
                ws = [clamp(w, -wmax, wmax) for w in ws]
                for h, w in zip(self.wheels, ws):
                    sim.setJointTargetVelocity(h, w)

                # --- Step-forward phase ---
                if stable >= stable_needed and err_abs <= tol_px:
                    t_end = sim.getSimulationTime() + CFG["gp_step_time"]

                    while not sim.getSimulationStopping() and sim.getSimulationTime() < t_end:
                        # Edge check during step
                        detected2, _ = self.floor_detected()
                        if not detected2:
                            lost += 1
                            if lost >= lost_needed:
                                break
                        else:
                            lost = 0

                        seen2, err_px2, img_w2, info2 = self.detect_goalposts()
                        if not seen2:
                            # Vision lost during step: stop and re-enter main loop
                            break

                        err_norm2 = err_px2 / max(1.0, img_w2 * 0.5)
                        omega2 = clamp(
                            CFG["gp_kp_omega"] * err_norm2,
                            -CFG["gp_max_omega"], CFG["gp_max_omega"]
                        )
                        omega2 *= CFG["gp_omega_sign"]

                        ws2 = self.wheels_from_twist(
                            CFG["vx_sign"] * CFG["gp_forward_speed"], 0.0, omega2
                        )
                        ws2 = [clamp(w, -wmax, wmax) for w in ws2]
                        for hh, ww in zip(self.wheels, ws2):
                            sim.setJointTargetVelocity(hh, ww)

                        sim.wait(dt)

                    self.stop_base()
                    sim.wait(CFG["post_stop_settle_s"])
                    stable = 0
                else:
                    sim.wait(dt)
                    
    def drive_to_floor_edge(self):
        with StepLogger("Drive to floor edge"):
            dt          = CFG["dt"]
            vmax        = CFG["edge_drive_speed"]
            wmax        = CFG["wheel_speed_max"]
            lost_needed = CFG["edge_lost_steps"]
            lost        = 0
            t0          = sim.getSimulationTime()
            timeout     = CFG["edge_timeout_s"]

            while not sim.getSimulationStopping():
                if sim.getSimulationTime() - t0 > timeout:
                    raise RuntimeError("Edge drive timeout: could not find floor edge.")

                detected, _ = self.floor_detected()
                if detected:
                    lost = 0
                else:
                    lost += 1

                if lost >= lost_needed:
                    break

                ws = self.wheels_from_twist(CFG["vx_sign"] * vmax, 0.0, 0.0)
                ws = [clamp(w, -wmax, wmax) for w in ws]
                for h, w in zip(self.wheels, ws):
                    sim.setJointTargetVelocity(h, w)

                sim.wait(dt)

            self.stop_base()
            sim.wait(CFG["post_stop_settle_s"])

            # Back off so only the cube falls, not the robot
            if CFG["edge_backoff_s"] > 0:
                self._drive_for_duration(CFG["edge_backoff_speed"], CFG["edge_backoff_s"])
                sim.wait(CFG["post_stop_settle_s"])

    def return_base_to_world_origin(self):
        with StepLogger("Return base to world origin"):
            self.turn_to_face_target(self.home_dummy, label="Face home (0,0)")
            self.drive_to_stop(self.home_dummy, stop_distance=0.05, label="Drive to home (0,0)")

    # -------------------------------------------------------------------------
    # Arm movement ? primitives
    # -------------------------------------------------------------------------

    def move_joint_smooth(self, joint, target, speed=None, joint_name="joint"):
        if speed is None:
            speed = CFG["joint_move_speed"]

        dt          = CFG["dt"]
        step        = speed * dt
        timeout     = CFG["joint_move_timeout_s"]
        stall_eps   = CFG["joint_stall_eps"]
        stall_steps = CFG["joint_stall_steps"]
        eps         = CFG["joint_eps"]
        near_eps    = 3.0 * eps

        try:
            cyclic, interval = sim.getJointInterval(joint)
            if not cyclic:
                jmin   = interval[0]
                jmax   = interval[0] + interval[1]
                target = clamp(target, jmin + 1e-4, jmax - 1e-4)
        except Exception:
            pass

        t0    = sim.getSimulationTime()
        last  = sim.getJointPosition(joint)
        stall = 0

        while not sim.getSimulationStopping():
            if sim.getSimulationTime() - t0 > timeout:
                cur = sim.getJointPosition(joint)
                raise RuntimeError(f"{joint_name} timeout cur={cur:.6f} target={target:.6f}")

            cur = sim.getJointPosition(joint)
            err = target - cur

            if abs(err) <= eps:
                break

            if abs(cur - last) < stall_eps and abs(err) <= near_eps:
                sim.setJointTargetPosition(joint, target)
                break

            if abs(cur - last) < stall_eps and abs(err) > near_eps:
                stall += 1
                if stall >= stall_steps:
                    raise RuntimeError(
                        f"{joint_name} stalled cur={cur:.6f} target={target:.6f} err={err:.6f}"
                    )
            else:
                stall = 0

            last = cur
            sim.setJointTargetPosition(joint, cur + clamp(err, -step, step))
            sim.wait(dt)

        sim.setJointTargetPosition(joint, target)

    def move_arm_sequential(self, target_config, order=(0, 1, 2, 3, 4),
                            label="Move arm sequential", speed=None):
        with StepLogger(label):
            for idx in order:
                self.move_joint_smooth(
                    self.arm[idx], target_config[idx],
                    speed=speed, joint_name=f"arm[{idx}]"
                )
            sim.wait(CFG["post_arm_settle_s"])

    def move_arm_interpolated(self, target_config, joints=(1, 2, 3),
                              duration=1.0, dt=0.02, label="Move arm interpolated"):
        with StepLogger(label):
            start = {i: sim.getJointPosition(self.arm[i]) for i in joints}
            t0    = sim.getSimulationTime()

            while not sim.getSimulationStopping():
                t     = sim.getSimulationTime() - t0
                alpha = min(1.0, t / max(1e-6, duration))

                for i in joints:
                    qi = start[i] + alpha * (target_config[i] - start[i])
                    sim.setJointTargetPosition(self.arm[i], qi)

                if alpha >= 1.0:
                    break

                sim.wait(dt)

            for i in joints:
                sim.setJointTargetPosition(self.arm[i], target_config[i])
            sim.wait(CFG["post_arm_settle_s"])

    # -------------------------------------------------------------------------
    # Wrist computation
    # -------------------------------------------------------------------------

    def compute_wrist_for_target(self, target_handle):
        """Compute wrist joint angle to align gripper with target's horizontal axis.

        The result is latched in self._wrist_latched for the duration of one pick.
        Reset self._wrist_latched = None before each pick to force recomputation.
        """
        if self._wrist_latched is not None:
            return self._wrist_latched

        cyclic, interval = sim.getJointInterval(self.arm[4])
        if cyclic:
            jmin, jmax = -math.pi, math.pi
        else:
            jmin = interval[0]
            jmax = interval[0] + interval[1]

        sens_pose  = sim.getObjectPose(self.wrist_sensor, sim.handle_world)
        cube_pose  = sim.getObjectPose(target_handle,     sim.handle_world)

        sens_dir   = _proj_xy_unit(_axis_world_from_pose(sens_pose, 0))
        cube_dir_x = _proj_xy_unit(_axis_world_from_pose(cube_pose, 0))
        cube_dir_y = _proj_xy_unit(_axis_world_from_pose(cube_pose, 1))

        delta_candidates = [
            _signed_angle_2d(sens_dir, cube_dir_x),
            _signed_angle_2d(sens_dir, cube_dir_y),
        ]

        # Also consider 180? flips (axis alignment is bidirectional)
        expanded = []
        for d in delta_candidates:
            expanded.append(_wrap_pi(d))
            expanded.append(_wrap_pi(d + math.pi))
            expanded.append(_wrap_pi(d - math.pi))

        cur  = sim.getJointPosition(self.arm[4])
        best = None
        for d in expanded:
            desired = _wrap_pi(cur + d)
            if desired < jmin:
                score, desired = 10.0 + (jmin - desired), jmin
            elif desired > jmax:
                score, desired = 10.0 + (desired - jmax), jmax
            else:
                score = abs(d)

            if best is None or score < best[0]:
                best = (score, desired)

        desired = best[1]

        if desired <= jmin + 1e-3 or desired >= jmax - 1e-3:
            log_info(
                f"[wrist] Hit wrist limit. desired={math.degrees(desired):.1f}deg "
                f"limits=[{math.degrees(jmin):.1f},{math.degrees(jmax):.1f}]deg. "
                f"Consider rotating base/joint0 for this cuboid yaw."
            )

        self._wrist_latched = desired
        return desired

    # -------------------------------------------------------------------------
    # Arm actions
    # -------------------------------------------------------------------------

    def go_pregrip(self, target_handle):
        with StepLogger("Go to pregrip (hover): joints 0->1->2->3, then wrist align"):
            pose = POSES["pregrip"]

            # Phase A: move joints 0..3 only (keep wrist as-is)
            current_wrist = sim.getJointPosition(self.arm[4])
            cfg = deg_pose_to_rad(pose["joints_deg"], wrist_rad=current_wrist)
            log_info(
                "  pregrip target (rad) [no wrist]: " +
                ", ".join([f"{v:.3f}" for v in cfg])
            )
            self.move_arm_sequential(
                cfg, order=pose["order"],
                label="Arm to pregrip (0-3 only)", speed=CFG["arm_speed_pregrip"]
            )

            # Phase B: compute wrist at pregrip pose, then move wrist only
            self._wrist_latched = None
            wrist = self.compute_wrist_for_target(target_handle)
            log_info(f"  wrist target (rad): {wrist:.3f}")
            self.move_joint_smooth(self.arm[4], wrist, speed=CFG["arm_speed_pregrip"])

    def go_grip_ready(self, target_handle):
        with StepLogger("Go to grip-ready (descend) interpolated"):
            pose  = POSES["grip_ready"]
            wrist = self.compute_wrist_for_target(target_handle)
            cfg   = deg_pose_to_rad(pose["joints_deg"], wrist_rad=wrist)
            self.move_arm_interpolated(
                cfg,
                joints=pose["order"],
                duration=CFG["grip_ready_duration_s"],
                dt=CFG["grip_ready_dt"],
                label=f"Grip-ready interpolated joints={pose['order']}"
            )

    def return_arm_to_neutral(self):
        with StepLogger("Return arm to neutral/carry"):
            pose  = POSES["neutral"]
            wrist = deg2rad(pose["wrist_deg"])
            cfg   = deg_pose_to_rad(pose["joints_deg"], wrist_rad=wrist)
            self.move_arm_sequential(cfg, order=pose["order"], label="Arm to neutral/carry sequential")

    def go_tucked_pose(self):
        with StepLogger("Go to tucked pose"):
            pose  = POSES["tucked"]
            wrist = deg2rad(pose["wrist_deg"])
            cfg   = deg_pose_to_rad(pose["joints_deg"], wrist_rad=wrist)
            self.move_arm_sequential(cfg, order=pose["order"], label="Arm to tucked sequential")

    # -------------------------------------------------------------------------
    # Gripper
    # -------------------------------------------------------------------------

    def get_gripper_positions(self):
        return sim.getJointPosition(self.gripper[0]), sim.getJointPosition(self.gripper[1])

    def set_gripper_targets(self, j1, j2):
        sim.setJointTargetPosition(self.gripper[0], j1)
        sim.setJointTargetPosition(self.gripper[1], j2)

    def is_grip_confirmed(self):
        j1, j2 = self.get_gripper_positions()
        tol    = CFG["gripper_tol"]
        ok1    = j1 <= CFG["gripper_close_goal_j1"] + tol
        ok2    = j2 >= CFG["gripper_close_goal_j2"] - tol
        return ok1 and ok2

    def step_close_gripper_symmetric(self):
        j1, j2  = self.get_gripper_positions()
        j1_goal = CFG["gripper_close_goal_j1"]
        j2_goal = CFG["gripper_close_goal_j2"]
        j1_next = j1_goal if j1 <= j1_goal else max(j1_goal, j1 + CFG["gripper_step_j1"])
        j2_next = j2_goal if j2 >= j2_goal else min(j2_goal, j2 + CFG["gripper_step_j2"])
        self.set_gripper_targets(j1_next, j2_next)

    def open_gripper(self):
        with StepLogger("Open gripper"):
            self.detach_object_from_grip()
            self.set_gripper_targets(CFG["gripper_open_j1"], CFG["gripper_open_j2"])
            sim.wait(0.3)

    def close_gripper_until_confirmed(self, target_handle):
        with StepLogger("Close gripper slowly until confirmed (symmetric screw-joint closure)"):
            t_start   = sim.getSimulationTime()
            timeout   = CFG["gripper_close_timeout"]
            log_every = CFG["log_every_steps"]
            k         = 0

            while not sim.getSimulationStopping():
                if sim.getSimulationTime() - t_start > timeout:
                    j1, j2 = self.get_gripper_positions()
                    raise RuntimeError(
                        f"Gripper close timeout. Current j1={j1:.4f}, j2={j2:.4f} "
                        f"(goals: j1={CFG['gripper_close_goal_j1']:.4f}, "
                        f"j2={CFG['gripper_close_goal_j2']:.4f})"
                    )

                if self.is_grip_confirmed():
                    j1, j2 = self.get_gripper_positions()
                    log_info(f"  grip confirmed at j1={j1:.4f}, j2={j2:.4f}")
                    self.attach_object_to_grip(target_handle)
                    log_info("  attached object to gripAttach")
                    return

                self.step_close_gripper_symmetric()

                if k % log_every == 0:
                    j1, j2 = self.get_gripper_positions()
                    log_info(f"  closing... j1={j1:.4f}  j2={j2:.4f}")
                k += 1

                sim.wait(CFG["gripper_close_dt"])

    # -------------------------------------------------------------------------
    # Object attachment
    # -------------------------------------------------------------------------

    def attach_object_to_grip(self, obj_handle):
        if obj_handle is None or obj_handle == -1:
            raise RuntimeError("attach_object_to_grip: invalid handle")

        self._attached_prev_parent = sim.getObjectParent(obj_handle)

        # Capture current static flag so it can be restored accurately on detach
        try:
            self._attached_prev_static = sim.getObjectInt32Param(
                obj_handle, sim.shapeintparam_static
            )
        except Exception:
            self._attached_prev_static = 0

        sim.setObjectParent(obj_handle, self.grip_attach, True)

        try:
            sim.setObjectInt32Param(obj_handle, sim.shapeintparam_static, 1)       # freeze while carried
            sim.setObjectInt32Param(obj_handle, sim.shapeintparam_respondable, 0)  # reduce jitter
        except Exception:
            pass

        try:
            sim.resetDynamicObject(obj_handle)
        except Exception:
            pass

        self.attached_object = obj_handle

    def detach_object_from_grip(self):
        if self.attached_object is None:
            return

        obj    = self.attached_object
        parent = self._attached_prev_parent if self._attached_prev_parent is not None else -1
        sim.setObjectParent(obj, parent, True)

        try:
            # Restore the original static flag captured at attach time
            prev_static = self._attached_prev_static if self._attached_prev_static is not None else 0
            sim.setObjectInt32Param(obj, sim.shapeintparam_static, prev_static)
            sim.setObjectInt32Param(obj, sim.shapeintparam_respondable, 1)
        except Exception:
            pass

        try:
            sim.resetDynamicObject(obj)
        except Exception:
            pass

        self.attached_object       = None
        self._attached_prev_parent = None
        self._attached_prev_static = None

    # -------------------------------------------------------------------------
    # Drop sequence
    # -------------------------------------------------------------------------

    def drop_cuboid_off_edge(self):
        with StepLogger("Drop cuboid off edge"):
            sim.wait(CFG["edge_drop_pause_s"])

            # Restore dynamic properties so the cube falls under gravity after release
            if self.attached_object is not None:
                try:
                    sim.setObjectInt32Param(self.attached_object, sim.shapeintparam_static, 0)
                    sim.setObjectInt32Param(self.attached_object, sim.shapeintparam_respondable, 1)
                    sim.resetDynamicObject(self.attached_object)
                except Exception as e:
                    log_info(f"  [drop_cuboid_off_edge] failed to restore dynamics: {e}")

            self.open_gripper()
            sim.wait(0.2)
            self._drive_for_duration(CFG["edge_reverse_speed"], CFG["edge_reverse_s"])
            
            
    #---------------------------------------------------
    #Cam stuff
    #----------------------------------------------
    
    def get_front_cam_rgb(self):
            """
            Returns (rgb_list, w, h) where rgb_list is a flat [R,G,B,R,G,B,...] list.
            Requires the sensor to have been handled this step.
            """
            # If frontCam has explicit handling enabled, we MUST handle it ourselves:
            sim.handleVisionSensor(self.front_cam)  # ensures fresh image [3](https://manual.coppeliarobotics.com/en/sim/simGetVisionSensorImg.htm)[2](https://github.com/CoppeliaRobotics/manual/blob/master/en/textureDialog.htm)

            img_bytes, res = sim.getVisionSensorImg(self.front_cam)  # bytes + [w,h] [3](https://manual.coppeliarobotics.com/en/sim/simGetVisionSensorImg.htm)
            w, h = res[0], res[1]

            # Convert bytes -> list of ints (0..255)
            rgb = sim.unpackUInt8Table(img_bytes)  # recommended by docs [3](https://manual.coppeliarobotics.com/en/sim/simGetVisionSensorImg.htm)
            return rgb, w, h
            
    
    def _centroid_rgb_threshold(self, rgb, w, h, mn, mx):
        """
        Fast centroid of pixels within RGB box [mn,mx], scanning only a vertical band
        and skipping pixels by 'gp_stride' for speed.
        Returns (cx, count).
        """
        rmin, gmin, bmin = mn
        rmax, gmax, bmax = mx

        stride = CFG.get("gp_stride", 1)

        # Compute y-band limits HERE (inside the function)
        y0 = int(h * CFG.get("gp_y0_frac", 0.0))
        y1 = int(h * CFG.get("gp_y1_frac", 1.0))

        # Clamp to valid range
        y0 = max(0, min(h, y0))
        y1 = max(0, min(h, y1))
        if y1 <= y0:
            y0, y1 = 0, h  # fallback

        count = 0
        xsum = 0

        for y in range(y0, y1, stride):
            row_base = (y * w) * 3
            for x in range(0, w, stride):
                idx = row_base + x * 3
                r = rgb[idx]
                g = rgb[idx + 1]
                b = rgb[idx + 2]

                if (rmin <= r <= rmax) and (gmin <= g <= gmax) and (bmin <= b <= bmax):
                    count += 1
                    xsum += x

        if count == 0:
            return None, 0

        return xsum / count, count


        
    
    
    def detect_goalposts(self):
        """
        Returns:
          seen_both (bool),
          err_px (float)      = (center_x - w/2) if both seen else 0
          w (int)
          info (dict): always includes seen_g, seen_r, cx_g, cx_r, n_g, n_r;
                       also includes 'sep' (cx_r - cx_g) when both are seen.
        """
        rgb, w, h = self.get_front_cam_rgb()

        cx_g, n_g = self._centroid_rgb_threshold(rgb, w, h, CFG["gp_green_min"], CFG["gp_green_max"])
        cx_r, n_r = self._centroid_rgb_threshold(rgb, w, h, CFG["gp_red_min"],   CFG["gp_red_max"])

        minpix = CFG["gp_min_pixels"]
        seen_g = (cx_g is not None) and (n_g >= minpix)
        seen_r = (cx_r is not None) and (n_r >= minpix)

        info = {
            "seen_g": seen_g, "seen_r": seen_r,
            "cx_g": cx_g,     "cx_r": cx_r,
            "n_g": n_g,       "n_r": n_r,
        }

        if int(sim.getSimulationTime() / CFG["dt"]) % 10 == 0:
            log_info(
                f"[gp] n_g={n_g} n_r={n_r} seen_g={seen_g} seen_r={seen_r} "
                f"cx_g={cx_g} cx_r={cx_r} w={w} err={(0.5*(cx_g+cx_r)-w*0.5) if (seen_g and seen_r) else None}"
            )

        if not (seen_g and seen_r):
            return False, 0.0, w, info

        sep    = cx_r - cx_g
        center = 0.5 * (cx_g + cx_r)
        err_px = center - (w * 0.5)
        info["sep"] = sep
        return True, err_px, w, info


    

    # -------------------------------------------------------------------------
    # Mission
    # -------------------------------------------------------------------------

    def pause_step(self, seconds, label):
        with StepLogger(label):
            sim.wait(seconds)

    def pick_and_drop_one(self, target_handle):
        """Execute a full pick-carry-drop-return cycle for one target object."""
        self._arm_moved = False
        with StepLogger("FULL PICK SEQUENCE"):
            try:
                self._wrist_latched = None
                self.turn_to_face_target(target_handle, label="Face cuboid")
                self.drive_to_stop(
                    target_handle, CFG["stop_distance"],
                    use_clearance=True, label="Drive to cuboid"
                )

                self._arm_moved = True
                self.go_pregrip(target_handle)
                self.pause_step(CFG["pregrip_pause_s"], "Pause at pregrip")
                self.go_grip_ready(target_handle)
                self.close_gripper_until_confirmed(target_handle)

                self._arm_moved = False   # arm back to neutral after this point
                self.return_arm_to_neutral()
                self.drive_to_dropzone_visual()
                self.drive_to_floor_edge()
                self.drop_cuboid_off_edge()
                self.go_tucked_pose()
                self.return_base_to_world_origin()

            finally:
                # Safety cleanup: ensure wheels stop and object is released
                self.stop_base()
                self.detach_object_from_grip()
                # If the arm was extended when an exception occurred, tuck it safely
                if self._arm_moved:
                    try:
                        self.go_tucked_pose()
                    except Exception:
                        pass

    def run_mission_loop(self):
        with StepLogger("MISSION LOOP"):
            while True:
                # Build list of uncollected, available cube handles
                candidates = []
                for path in CFG["pick_targets"]:
                    h = sim.getObject(path, {"noError": True})
                    if h == -1:
                        log_info(f"[skip] missing {path}")
                        continue
                    if h in self.collected_cubes:
                        continue
                    candidates.append((path, h))

                if not candidates:
                    log_info("[mission] all cubes collected or unavailable")
                    break

                # Pick the closest uncollected cube
                best_path, best_h, best_dist = None, None, float('inf')
                for path, h in candidates:
                    rel  = sim.getObjectPosition(h, self.base_ref)
                    dist = math.hypot(rel[0], rel[1])
                    if dist < best_dist:
                        best_dist = dist
                        best_path = path
                        best_h    = h

                log_info(f"[mission] picking closest: {best_path} dist={best_dist:.3f}m "
                         f"(collected so far: {len(self.collected_cubes)})")
                try:
                    self.pick_and_drop_one(best_h)
                    self.collected_cubes.append(best_h)
                    aliases = []
                    for c in self.collected_cubes:
                        try:
                            aliases.append(sim.getObjectAlias(c, 1))
                        except Exception:
                            aliases.append(str(c))
                    log_info(f"[mission] collected {len(self.collected_cubes)} cube(s): {aliases}")
                except Exception as e:
                    log_info(
                        f"[mission] failed on {best_path}: {e}\n{traceback.format_exc()}"
                    )
                    # Mark as attempted so we don't retry a broken cube indefinitely
                    self.collected_cubes.append(best_h)
                    continue


# -----------------------------------------------------------------------
# DEBUG UTILITIES ? not called during normal mission; kept for manual use
# -----------------------------------------------------------------------

def _debug_reset_robot_to_world_origin(ctrl):
    """Teleport the robot model to world origin. For debugging / manual recovery only."""
    with StepLogger("Reset robot to world origin (0,0)"):
        ctrl.stop_base()
        p  = sim.getObjectPosition(ctrl.model, sim.handle_world)
        z  = p[2]
        x0, y0 = CFG.get("reset_base_xy", [0.0, 0.0])
        pose = [x0, y0, z, 0.0, 0.0, 0.0, 1.0]
        sim.setObjectPose(ctrl.model, pose, sim.handle_world)
        ctrl.stop_base()
        sim.wait(0.05)


# -----------------------------
# CoppeliaSim entry points
# -----------------------------
def sysCall_init():
    global sim, ctrl, INIT_OK
    sim    = require("sim")
    INIT_OK = False
    ctrl   = YouBotPickController()
    try:
        ctrl.init_handles()
        INIT_OK = True
    except Exception as e:
        log_info(f"[FATAL] Init failed: {e}\n{traceback.format_exc()}")
        try:
            sim.addLog(sim.verbosity_errors, f"[FATAL] Init failed: {e}")
        except Exception:
            pass


def sysCall_thread():
    # Threaded scripts can safely block/yield with sim.wait
    if not INIT_OK:
        log_info("[ABORT] Init failed; not running sequence.")
        return

    sim.wait(0.2)
    ctrl.run_mission_loop()


def sysCall_cleanup():
    if not INIT_OK:
        return
    try:
        ctrl.stop_base()
    except Exception:
        pass
