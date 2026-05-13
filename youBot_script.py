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

    # Color thresholds in RGB (0..255) for PURE colored posts:
    "gp_green_min": [0, 180, 0],    # green: G high, R/B low
    "gp_green_max": [80, 255, 80],

    "gp_red_min":   [180, 0, 0],    # red: R high, G/B low
    "gp_red_max":   [255, 80, 80],

    # Minimum number of pixels required to accept detection:
    "gp_min_pixels": 40,

    # Visual servo control:
    "gp_kp_omega": 1.2,             # turning gain from pixel error
    "gp_max_omega": 0.7,            # rad/s command into wheels_from_twist

    # Step-drive behaviour:
    "gp_forward_speed": 0.10,       # m/s forward when aligned
    "gp_step_time": 0.25,           # seconds per forward step
    "gp_align_tol_px": 10,          # must centre gate within this pixel tolerance
    "gp_align_stable_steps": 6,     # stable frames required for "locked-on"
    "gp_search_omega": 0.35,        # rad/s spin when posts not visible
    "gp_timeout_s": 60,             # overall timeout

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
            flip_every_s = 6.0  # tweak if you want (or put in CFG)

            while not sim.getSimulationStopping():
                if sim.getSimulationTime() - t0 > timeout:
                    raise RuntimeError("Vision dropzone timeout: could not reach/align with goalposts.")

                # --- Stop condition: edge reached (with confirmation) ---
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

                # Flip scan direction occasionally if not acquired
                if not seen_both and (sim.getSimulationTime() - last_flip_t) > flip_every_s:
                    scan_dir *= -1.0
                    last_flip_t = sim.getSimulationTime()

                if not seen_both:
                    stable = 0

                    # SEARCH / ACQUIRE
                    omega = CFG["gp_search_omega"]

                    if info.get("seen_g") and not info.get("seen_r"):
                        cx = info.get("cx_g")
                        if cx is not None:
                            omega = -abs(omega) if cx < img_w * 0.5 else abs(omega)

                    elif info.get("seen_r") and not info.get("seen_g"):
                        cx = info.get("cx_r")
                        if cx is not None:
                            omega = -abs(omega) if cx < img_w * 0.5 else abs(omega)

                    else:
                        omega = abs(omega)

                    omega *= CFG["omega_sign"] * scan_dir

                    ws = self.wheels_from_twist(0.0, 0.0, omega)
                    ws = [clamp(w, -wmax, wmax) for w in ws]
                    for h, w in zip(self.wheels, ws):
                        sim.setJointTargetVelocity(h, w)

                    sim.wait(dt)
                    continue

                # --- ALIGN logic (stable frames) ---
                if abs(err_px) <= tol_px:
                    stable += 1
                else:
                    stable = 0

                # Closed-loop omega from pixel error
                err_norm = err_px / max(1.0, img_w * 0.5)
                omega = clamp(CFG["gp_kp_omega"] * err_norm, -CFG["gp_max_omega"], CFG["gp_max_omega"])
                omega *= CFG["omega_sign"]

                # Hold position until stable lock, then do a short forward step
                vx = 0.0 if stable < stable_needed else (CFG["vx_sign"] * CFG["gp_forward_speed"])

                # Apply immediate command (mostly for align-in-place)
                ws = self.wheels_from_twist(vx, 0.0, omega)
                ws = [clamp(w, -wmax, wmax) for w in ws]
                for h, w in zip(self.wheels, ws):
                    sim.setJointTargetVelocity(h, w)

                # --- Step forward phase ---
                if stable >= stable_needed and abs(err_px) <= tol_px:
                    t_end = sim.getSimulationTime() + CFG["gp_step_time"]

                    while not sim.getSimulationStopping() and sim.getSimulationTime() < t_end:
                        # Edge check during step too:
                        detected2, _ = self.floor_detected()
                        if not detected2:
                            lost += 1
                            if lost >= lost_needed:
                                break
                        else:
                            lost = 0

                        seen2, err_px2, img_w2, info2 = self.detect_goalposts()
                        if not seen2:
                            # Vision lost: stop and go back to SEARCH
                            break

                        err_norm2 = err_px2 / max(1.0, img_w2 * 0.5)
                        omega2 = clamp(CFG["gp_kp_omega"] * err_norm2, -CFG["gp_max_omega"], CFG["gp_max_omega"])
                        omega2 *= CFG["omega_sign"]

                        ws2 = self.wheels_from_twist(CFG["vx_sign"] * CFG["gp_forward_speed"], 0.0, omega2)
                        ws2 = [clamp(w, -wmax, wmax) for w in ws2]
                        for hh, ww in zip(self.wheels, ws2):
                            sim.setJointTargetVelocity(hh, ww)

                        sim.wait(dt)

                    # stop, settle, and force re-acquire to avoid drifting
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

     
