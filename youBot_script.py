# Threaded simulation script (Python) for CoppeliaSim
# Modular youBot pick sequence with robust object-path handling

import math


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
    
    "pick_targets": ["/Cuboid", "/Cuboid0", "/Cuboid1","/Cuboid2","/Cuboid3","/Cuboid4","/Cuboid5","/Cuboid6"],

    # Behavior params
    "stop_distance": 0.16,
    "dt": 0.02,

    
    # Base turn-then-drive settings:
    "forward_axis": "y",        # "x" or "y" axis of youBot_ref that points "forward"
    "forward_sign":  1.0,       # flip to -1 if forward seems backwards
    "omega_sign":    1.0,       # flip to -1 if turning goes the wrong way

    "ang_kp":  0.8,             # rotation gain
    "ang_max": 0.6,             # max yaw rate command (rad/s equiv input to wheels_from_twist)

    "face_tol_deg": 8.0,        # stop rotating when within this angle
    "drive_angle_gate_deg": 10.0, # only drive forward if facing within this angle

    "approach_speed": 0.15,
    "wheel_speed_max": 5,
    "approach_kp": 1.0,
    "wheel_dir": [1, 1, 1, 1],  # flip any wheel by changing to -1
    
    "vx_sign": -1.0,   # <-- flip forward/backward drive. Start with -1.0 to fix ?drives backwards?

    "stop_band": 0.03,          # extra margin (m). stop when dist <= stop_distance + stop_band
    "stop_stable_steps": 10,    # must be inside band for N steps before stopping
    "max_drive_time": 60,     # safety timeout (seconds)
    
        

    "grip_ready_duration_s": 1.9,   # total time for the descent move
    "grip_ready_dt": 0.02,          # step time


    
    # Mecanum/omni constants (tune if needed)
    "wheel_radius": 0.0475,
    "lx_plus_ly": 0.228 + 0.158,

    # Motion smoothing
    "joint_move_speed": 0.8,
    "joint_eps": 1e-3,

    # --- Gripper (screw joints) ---
    "gripper_open_j1":  0.025,
    "gripper_open_j2": -0.050,

    "gripper_close_goal_j1":  0.018,
    "gripper_close_goal_j2": -0.032,

    # per-tick symmetric closure steps (your rule)
    "gripper_step_j1": -0.001,   # reduce joint1
    "gripper_step_j2":  0.002,   # increase joint2

    "gripper_tol": 5e-4,         # position tolerance for "confirmed"
    "gripper_close_dt": 0.05,    # seconds between steps (controls how "slow" it closes)
    "gripper_close_timeout": 50, # safety timeout

    # Poses (tune these)
    "neutral_arm_deg": [0, -40, -50, 0],   # joints 0..3 in degrees
    "neutral_wrist_deg": 0.0,  
    "pregrip_deg": [0, -40, -88, -52],

    # Pause at pregrip (seconds)
    "pregrip_pause_s": 2.5,
    
    
    "arm_speed_pregrip": 0.6,     # rad/s (slower = smoother)

    # ?Ready to grip? pose (joints 0..3 in degrees)
    "grip_ready_deg": [0, -52, -95, -33],
    "grip_ready_order": (3,2,1),
    
    
    # --- Edge drop ---
    "edge_sensor_path": ":/edgeSensorDown",  # down-facing proximity sensor near front
    "floor_entity_path": "/Floor",           # optional: floor shape path (if you have it)
    "edge_drive_speed": 0.10,                # m/s forward while searching edge
    "edge_timeout_s": 60,                  # safety timeout
    "edge_lost_steps": 6,                    # require N consecutive "no floor" frames
    "edge_backoff_s": 0.3,                   # seconds to back off after edge found
    "edge_backoff_speed": -0.06,             # m/s (negative = backwards) for backoff
    "edge_drop_pause_s": 0.2,                # settle pause before dropping
        
    "edge_reverse_s": 1.0,         # reverse duration
    "edge_reverse_speed": -0.04,   # m/s (negative = reverse)

    
    
    "tucked_arm_deg": [0, 30, 60, 30],   # joints 0..3 in degrees
    "tucked_wrist_deg": 0.0,             # joint4 in degrees
    "reset_base_xy": [0.0, 0.0],         # world origin target
    "reset_base_yaw_deg": 0.0,           # yaw at reset
    
    



    
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
        # Don't swallow exceptions by default:
        return False


# -----------------------------
# Math helpers
# -----------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))
    
    
def deg2rad(deg):
    return deg * math.pi / 180.0




# -----------------------------
# Controller
# -----------------------------
class YouBotPickController:

    # -------------------------------------------------------------------------
    # Initialisation
    # -------------------------------------------------------------------------

    def __init__(self):
        self.model = None
        self.cuboid = None
        self.base_ref = None
        self.arm = []
        self.gripper = []
        self.wheels = []

    def init_handles(self):
        with StepLogger("Init handles"):
            # model containing this script (":") enables model-relative access via ":/..."
            self.model = sim.getObject(":")
            try:
                model_path = sim.getObjectAlias(self.model, 2)
            except Exception:
                model_path = "<unknown>"
            log_info(f"  Model root: {model_path}")

            self.cuboid   = self.get_object_strict(CFG["cuboid_path"],   label="Cuboid")
            self.base_ref = self.get_object_strict(CFG["base_ref_path"], label="Base ref (youBot_ref)")

            self.arm     = [self.get_object_strict(p, label=f"Arm joint {i}")      for i, p in enumerate(CFG["arm_joint_paths"])]
            self.gripper = [self.get_object_strict(p, label=f"Gripper joint {i+1}") for i, p in enumerate(CFG["gripper_joint_paths"])]
            self.wheels  = [self.get_object_strict(p, label=f"Wheel {i}")           for i, p in enumerate(CFG["wheel_joint_paths"])]

            self.grip_attach          = self.get_object_strict(CFG["grip_attach_path"], label="Grip attach dummy")
            self.attached_object      = None
            self._attached_prev_parent = None
            self._attached_prev_static = None
            
            
            # Create a world-fixed dummy marking home (0,0)
            self.home_dummy = sim.createDummy(0.01)
            sim.setObjectAlias(self.home_dummy, "HOME_0_0", 0)

            # Set it at world origin (keep z=0). setObjectPosition uses world frame when relativeTo=sim.handle_world 
            sim.setObjectPosition(self.home_dummy, [0.0, 0.0, 0.0], sim.handle_world)  # 

                        
                        
            # Edge sensor (required)
            self.edge_sensor = self.get_object_strict(CFG["edge_sensor_path"], label="Edge sensor (down)")

            # Floor entity (optional)
            self.floor_entity = None
            if CFG.get("floor_entity_path"):
                self.floor_entity = sim.getObject(CFG["floor_entity_path"], {"noError": True})
                if self.floor_entity == -1:
                    self.floor_entity = None


            # Ensure stopped:
            for w in self.wheels:
                sim.setJointTargetVelocity(w, 0.0)
    
    def floor_detected(self):
        """
        Returns (detected: bool, dist: float or None).
        Uses sim.checkProximitySensor each call. 
        """
        entity = self.floor_entity if self.floor_entity is not None else sim.handle_all
        res, dist, point, obj, n = sim.checkProximitySensor(self.edge_sensor, entity)
        if res == 1:
            return True, dist
        return False, None
        
    
    def drive_to_floor_edge(self):
        with StepLogger("Drive to floor edge"):
            dt = CFG["dt"]
            vmax = CFG["edge_drive_speed"]
            wmax = CFG["wheel_speed_max"]

            lost_needed = CFG.get("edge_lost_steps", 6)
            lost = 0

            t0 = sim.getSimulationTime()
            timeout = CFG.get("edge_timeout_s", 25.0)

            while not sim.getSimulationStopping():
                if sim.getSimulationTime() - t0 > timeout:
                    raise RuntimeError("Edge drive timeout: could not find floor edge.")

                detected, dist = self.floor_detected()
                if detected:
                    lost = 0
                else:
                    lost += 1

                # If we lost floor consistently -> edge
                if lost >= lost_needed:
                    break

                # Drive forward (no steering here; keep it simple)
                ws = self.wheels_from_twist(CFG.get("vx_sign", 1.0) * vmax, 0.0, 0.0)
                ws = [clamp(w, -wmax, wmax) for w in ws]
                for h, w in zip(self.wheels, ws):
                    sim.setJointTargetVelocity(h, w)

                sim.wait(dt)

            # Stop at edge
            self.stop_base()
            sim.wait(0.1)

            # Optional: back off a bit so only the cube falls, not the robot
            backoff_speed = CFG.get("edge_backoff_speed", -0.06)
            backoff_s = CFG.get("edge_backoff_s", 0.3)
            if backoff_s > 0:
                t_end = sim.getSimulationTime() + backoff_s
                while not sim.getSimulationStopping() and sim.getSimulationTime() < t_end:
                    ws = self.wheels_from_twist(CFG.get("vx_sign", 1.0) * backoff_speed, 0.0, 0.0)
                    ws = [clamp(w, -wmax, wmax) for w in ws]
                    for h, w in zip(self.wheels, ws):
                        sim.setJointTargetVelocity(h, w)
                    sim.wait(dt)

                self.stop_base()
                sim.wait(0.1)
                
    def turn_to_face_target(self, target_handle, label="Turn to face target"):
        with StepLogger(label):
            dt = CFG["dt"]
            tol = math.radians(CFG["face_tol_deg"])
            wmax = CFG["wheel_speed_max"]

            stable_needed = 10
            stable_count = 0

            while not sim.getSimulationStopping():
                rel = sim.getObjectPosition(target_handle, self.base_ref)  # target in base frame 
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

                omega = clamp(CFG["ang_kp"] * err, -CFG["ang_max"], CFG["ang_max"])
                omega *= CFG["omega_sign"]

                ws = self.wheels_from_twist(0.0, 0.0, omega)
                ws = [clamp(w, -wmax, wmax) for w in ws]
                for h, w in zip(self.wheels, ws):
                    sim.setJointTargetVelocity(h, w)

                sim.wait(dt)

            self.stop_base()
            sim.wait(0.1)
            
    def drive_to_target(self, target_handle, stop_distance=0.15, label="Drive to target"):
        with StepLogger(label):
            dt = CFG["dt"]
            vmax = CFG["approach_speed"]
            wmax = CFG["wheel_speed_max"]
            gate = math.radians(CFG["drive_angle_gate_deg"])

            band = CFG.get("stop_band", 0.03)
            stable_needed = CFG.get("stop_stable_steps", 10)
            stable = 0

            t_start = sim.getSimulationTime()
            max_time = CFG.get("max_drive_time", 60.0)

            while not sim.getSimulationStopping():
                if sim.getSimulationTime() - t_start > max_time:
                    log_info("  [drive_to_target] timeout reached -> stopping")
                    break

                # Target position in base frame (x,y,z relative to youBot_ref) 
                rel = sim.getObjectPosition(target_handle, self.base_ref)  # 
                forward, lateral = self.forward_lateral(rel)

                # planar distance to target in base frame
                dist = math.hypot(forward, lateral)

                if dist <= (stop_distance + band):
                    stable += 1
                    self.stop_base()
                    if stable >= stable_needed:
                        log_info(f"  [drive_to_target] arrived: dist={dist:.3f}m")
                        break
                    sim.wait(dt)
                    continue
                else:
                    stable = 0

                err_ang = math.atan2(lateral, forward)

                vx = 0.0
                if abs(err_ang) <= gate:
                    err_lin = dist - stop_distance
                    v = clamp(CFG["approach_kp"] * err_lin, 0.0, vmax)
                    vx = CFG.get("vx_sign", 1.0) * v

                omega = clamp(CFG["ang_kp"] * err_ang, -CFG["ang_max"], CFG["ang_max"])
                omega *= CFG["omega_sign"]

                ws = self.wheels_from_twist(vx, 0.0, omega)
                ws = [clamp(w, -wmax, wmax) for w in ws]
                for h, w in zip(self.wheels, ws):
                    sim.setJointTargetVelocity(h, w)

                sim.wait(dt)

            self.stop_base()
            sim.wait(0.1)

    
    def return_base_to_world_origin(self):
        with StepLogger("Return base to world origin"):
            # face home then drive to it
            self.turn_to_face_target(self.home_dummy, label="Face home (0,0)")
            self.drive_to_target(self.home_dummy, stop_distance=0.05, label="Drive to home (0,0)")

    
    def reset_robot_to_world_origin(self):
        with StepLogger("Reset robot to world origin (0,0)"):
            # Always stop wheels so nothing keeps pushing after teleport:
            self.stop_base()

            # Keep the current z (height) so you don't spawn inside the floor:
            p = sim.getObjectPosition(self.model, sim.handle_world)  # [x y z] [6](https://manual.coppeliarobotics.com/en/sim/simGetObjectPosition.htm)
            z = p[2]

            x0, y0 = CFG.get("reset_base_xy", [0.0, 0.0])
            yaw = deg2rad(CFG.get("reset_base_yaw_deg", 0.0))

            # Identity quaternion for yaw-only rotation around world Z:
            # easiest: set orientation via Euler then build pose with identity quaternion:
            # BUT setObjectPose needs quaternion; simplest is yaw=0 (identity) by default.
            #
            # If you MUST set yaw!=0, you can use sim.setObjectOrientation instead (below).
            pose = [x0, y0, z, 0.0, 0.0, 0.0, 1.0]  # [x y z qx qy qz qw] [2](https://manual.coppeliarobotics.com/en/sim/simSetObjectPose.htm)
            sim.setObjectPose(self.model, pose, sim.handle_world)     # [2](https://manual.coppeliarobotics.com/en/sim/simSetObjectPose.htm)

            # If you want yaw reset explicitly (and yaw != 0), do it via Euler:
            # sim.setObjectOrientation(self.model, [0.0, 0.0, yaw], sim.handle_world) [4](https://manual.coppeliarobotics.com/en/sim/simSetObjectOrientation.htm)

            self.stop_base()
            sim.wait(0.05)

                
    

    def drop_cuboid_off_edge(self):
        with StepLogger("Drop cuboid off edge"):
            sim.wait(CFG.get("edge_drop_pause_s", 0.2))
            self.open_gripper()
            sim.wait(0.2)

            # Slower reverse (configurable)
            reverse_s = CFG.get("edge_reverse_s", 1.0)
            reverse_v = CFG.get("edge_reverse_speed", -0.04)

            t_end = sim.getSimulationTime() + reverse_s
            while sim.getSimulationTime() < t_end and not sim.getSimulationStopping():
                ws = self.wheels_from_twist(CFG.get("vx_sign", 1.0) * reverse_v, 0.0, 0.0)
                ws = [clamp(w, -CFG["wheel_speed_max"], CFG["wheel_speed_max"]) for w in ws]
                for h, w in zip(self.wheels, ws):
                    sim.setJointTargetVelocity(h, w)
                sim.wait(CFG["dt"])

            self.stop_base()






    def get_object_strict(self, path: str, label="Object"):
        """
        Robust object fetch:
        - If the provided string is a proper object path, use it directly.
        - Otherwise try common path forms.
        sim.getObject expects object paths/aliases as described in the docs.
        """
        # If user gave a correct path prefix, just try it:
        if path and path[0] in ["/", ".", ":"]:
            h = sim.getObject(path, {"noError": True})
            if h != -1:
                return h
            raise RuntimeError(f"{label} not found at path '{path}'")

        # Otherwise try to interpret "path" as alias in useful ways:
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

    def get_cuboid_pos(self):
        return sim.getObjectPosition(self.cuboid, -1)

    def cuboid_vector_in_base_frame(self):
        # Cuboid coords expressed in youBot_ref frame:
        return sim.getObjectPosition(self.cuboid, self.base_ref)

    def forward_lateral(self, rel):
        # rel = [x,y,z] in base frame
        if CFG["forward_axis"] == "x":
            forward = rel[0]
            lateral = rel[1]
        else:  # "y"
            forward = rel[1]
            lateral = rel[0]

        forward *= CFG["forward_sign"]
        return forward, lateral

    def heading_error(self, rel):
        forward, lateral = self.forward_lateral(rel)
        # angle to target relative to forward axis
        return math.atan2(lateral, forward)

    def get_clearance_front_to_cuboid(self, threshold=2.0):
        result, distData, objPair = sim.checkDistance(self.base_ref, self.cuboid, threshold)
        if result <= 0:
            # If threshold>0 and result==0 -> distance > threshold (so "far away")
            return float('inf')
        return distData[-1]  # distance value

    # -------------------------------------------------------------------------
    # Base movement
    # -------------------------------------------------------------------------

    def wheels_from_twist(self, vx, vy, omega):
        r = CFG["wheel_radius"]
        L = CFG["lx_plus_ly"]

        # wheel order: fl, rl, rr, fr (matching CFG["wheel_joint_paths"])
        w_fl = (vx - vy - L * omega) / r
        w_rl = (vx + vy - L * omega) / r
        w_rr = (vx - vy + L * omega) / r
        w_fr = (vx + vy + L * omega) / r

        ws = [w_fl, w_rl, w_rr, w_fr]

        # Apply per-wheel direction flips:
        ws = [d*w for d, w in zip(CFG["wheel_dir"], ws)]

        return ws

    def stop_base(self):
        for h in self.wheels:
            sim.setJointTargetVelocity(h, 0.0)

    def turn_to_face_cuboid(self):
        with StepLogger("Turn to face cuboid"):
            dt = CFG["dt"]
            tol = math.radians(CFG["face_tol_deg"])
            wmax = CFG["wheel_speed_max"]

            # Require being inside tolerance for a short streak
            stable_needed = 10
            stable_count = 0

            log_every = 25
            k = 0

            while not sim.getSimulationStopping():
                rel = self.cuboid_vector_in_base_frame()
                err = self.heading_error(rel)

                if k % log_every == 0:
                    log_info(
                        f"  face: rel=({rel[0]:.3f},{rel[1]:.3f}) "
                        f"heading_err_deg={math.degrees(err):.2f}"
                    )
                k += 1

                # Deadband + stability count
                if abs(err) <= tol:
                    stable_count += 1
                    if stable_count >= stable_needed:
                        break
                    # hold still while stabilizing
                    self.stop_base()
                    sim.wait(dt)
                    continue
                else:
                    stable_count = 0

                # Gentle P controller
                omega = clamp(CFG["ang_kp"] * err, -CFG["ang_max"], CFG["ang_max"])
                omega *= CFG["omega_sign"]

                ws = self.wheels_from_twist(0.0, 0.0, omega)
                ws = [clamp(w, -wmax, wmax) for w in ws]
                for h, w in zip(self.wheels, ws):
                    sim.setJointTargetVelocity(h, w)

                sim.wait(dt)

            self.stop_base()
            sim.wait(0.1)

    def drive_forward_to_stop(self, stop_distance):
        with StepLogger(f"Drive forward to stop at {stop_distance:.2f}m"):
            dt = CFG["dt"]
            vmax = CFG["approach_speed"]
            wmax = CFG["wheel_speed_max"]
            gate = math.radians(CFG["drive_angle_gate_deg"])

            band = CFG.get("stop_band", 0.03)
            stable_needed = CFG.get("stop_stable_steps", 10)
            stable = 0

            t_start = sim.getSimulationTime()
            max_time = CFG.get("max_drive_time", 60.0)

            log_every = 25
            k = 0

            while not sim.getSimulationStopping():
                # Safety timeout
                if sim.getSimulationTime() - t_start > max_time:
                    log_info("  [drive] timeout reached -> stopping")
                    break

                rel = self.cuboid_vector_in_base_frame()
                forward, lateral = self.forward_lateral(rel)

                # --- TRUE stop signal: minimum clearance ---
                clearance = self.get_clearance_front_to_cuboid(threshold=5.0)

                # Stop criterion with band + stability
                if clearance <= (stop_distance + band):
                    stable += 1
                    self.stop_base()
                    if stable >= stable_needed:
                        log_info(f"  [drive] stop: clearance={clearance:.3f}m (stable {stable}/{stable_needed})")
                        break
                    sim.wait(dt)
                    continue
                else:
                    stable = 0

                # Heading error for steering
                err_ang = math.atan2(lateral, forward)

                # Defaults
                vx = 0.0

                # Only drive if roughly facing target
                if abs(err_ang) <= gate:
                    # Speed based on clearance error (NOT origin distance)
                    err_lin = clearance - stop_distance
                    v = clamp(CFG["approach_kp"] * err_lin, 0.0, vmax)
                    vx = CFG.get("vx_sign", 1.0) * v

                omega = clamp(CFG["ang_kp"] * err_ang, -CFG["ang_max"], CFG["ang_max"])
                omega *= CFG["omega_sign"]

                if k % log_every == 0:
                    log_info(
                        f"  drive: clearance={clearance:.3f} "
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
            sim.wait(0.1)

    # -------------------------------------------------------------------------
    # Arm poses
    # -------------------------------------------------------------------------
    
    def go_tucked_pose(self):
        with StepLogger("Go to tucked pose"):
            j0, j1, j2, j3 = CFG["tucked_arm_deg"]
            wrist = deg2rad(CFG.get("tucked_wrist_deg", 0.0))
            target = [deg2rad(j0), deg2rad(j1), deg2rad(j2), deg2rad(j3), wrist]
            self.move_arm_sequential(target, order=(0,1,2,3,4), label="Arm to tucked sequential")


    def get_wrist_rotation_for_cuboid(self):
        # Latch per pick (reset self._wrist_latched=None in run_pick_sequence)
        if getattr(self, "_wrist_latched", None) is not None:
            return self._wrist_latched

        # Resolve sensor handle once
        if not hasattr(self, "wrist_sensor"):
            self.wrist_sensor = sim.getObject(":/wristOrientSensor")
            if self.wrist_sensor == -1:
                raise RuntimeError("Missing Dummy 'wristOrientSensor' (:/wristOrientSensor).")

        def wrap_pi(a):
            while a > math.pi:
                a -= 2.0 * math.pi
            while a < -math.pi:
                a += 2.0 * math.pi
            return a

        def quat_to_R(qx, qy, qz, qw):
            xx, yy, zz = qx*qx, qy*qy, qz*qz
            xy, xz, yz = qx*qy, qx*qz, qy*qz
            wx, wy, wz = qw*qx, qw*qy, qw*qz
            return [
                [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
                [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
                [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)]
            ]

        def axis_world_from_pose(pose, local_axis):
            # pose = [x y z qx qy qz qw] from sim.getObjectPose
            qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
            R = quat_to_R(qx, qy, qz, qw)
            # local_axis: 0->X, 1->Y, 2->Z
            return [R[0][local_axis], R[1][local_axis], R[2][local_axis]]

        def proj_xy_unit(v):
            x, y = v[0], v[1]
            n = math.hypot(x, y)
            if n < 1e-9:
                return [1.0, 0.0]
            return [x/n, y/n]

        def signed_angle_2d(a, b):
            # rotate a -> b, both are 2D unit vectors
            cross = a[0]*b[1] - a[1]*b[0]
            dot = a[0]*b[0] + a[1]*b[1]
            return math.atan2(cross, dot)

        # Wrist limits from the joint itself
        cyclic, interval = sim.getJointInterval(self.arm[4])
        if cyclic:
            # If cyclic, it can rotate freely; fallback to full range:
            jmin, jmax = -math.pi, math.pi
        else:
            jmin = interval[0]
            jmax = interval[0] + interval[1]

        # World poses
        sens_pose = sim.getObjectPose(self.wrist_sensor, sim.handle_world)
        cube_pose = sim.getObjectPose(self.cuboid, sim.handle_world)

        # Gripper forward = sensor local X
        sens_dir = proj_xy_unit(axis_world_from_pose(sens_pose, 0))

        # Cube axes candidates: local X and local Y
        cube_dir_x = proj_xy_unit(axis_world_from_pose(cube_pose, 0))
        cube_dir_y = proj_xy_unit(axis_world_from_pose(cube_pose, 1))

        # Desired deltas to align sensor X -> cube axis
        delta_candidates = [
            signed_angle_2d(sens_dir, cube_dir_x),
            signed_angle_2d(sens_dir, cube_dir_y),
        ]

        # Also allow flipping by 180? (axis alignment is bidirectional)
        expanded = []
        for d in delta_candidates:
            expanded.append(wrap_pi(d))
            expanded.append(wrap_pi(d + math.pi))
            expanded.append(wrap_pi(d - math.pi))

        # Current wrist and best target selection
        cur = sim.getJointPosition(self.arm[4])

        best = None  # (score, desired)
        for d in expanded:
            desired = wrap_pi(cur + d)

            # score: prefer solutions inside limits; otherwise penalize distance past limits
            if desired < jmin:
                score = 10.0 + (jmin - desired)  # outside low
                desired = jmin
            elif desired > jmax:
                score = 10.0 + (desired - jmax)  # outside high
                desired = jmax
            else:
                score = abs(d)  # inside limits: smaller move is better

            if best is None or score < best[0]:
                best = (score, desired)

        desired = best[1]

        # Warn if we had to clamp hard (means base rotation needed)
        if desired <= jmin + 1e-3 or desired >= jmax - 1e-3:
            log_info(
                f"[wrist] Hit wrist limit. desired={math.degrees(desired):.1f}deg "
                f"limits=[{math.degrees(jmin):.1f},{math.degrees(jmax):.1f}]deg. "
                f"Consider rotating base/joint0 for this cuboid yaw."
            )

        self._wrist_latched = desired
        return desired

    def get_pregrip_config_rad_no_wrist(self):
        j0, j1, j2, j3 = CFG["pregrip_deg"]
        current_wrist = sim.getJointPosition(self.arm[4])
        return [deg2rad(j0), deg2rad(j1), deg2rad(j2), deg2rad(j3), current_wrist]

    def get_grip_ready_config_rad(self):
        j0, j1, j2, j3 = CFG["grip_ready_deg"]
        # keep wrist the same target already computed at pregrip:
        wrist = self.get_wrist_rotation_for_cuboid()
        return [deg2rad(j0), deg2rad(j1), deg2rad(j2), deg2rad(j3), wrist]

    # -------------------------------------------------------------------------
    # Arm movement
    # -------------------------------------------------------------------------

    
    def move_joint_smooth(self, joint, target, speed=None, joint_name="joint"):
        if speed is None:
            speed = CFG["joint_move_speed"]

        dt = CFG["dt"]
        step = speed * dt

        # Clamp to joint limits (prevents infinite waits at hard stops) 
        try:
            cyclic, interval = sim.getJointInterval(joint)  # 
            if not cyclic:
                jmin = interval[0]
                jmax = interval[0] + interval[1]
                target = clamp(target, jmin + 1e-4, jmax - 1e-4)
        except Exception:
            pass

        timeout = CFG.get("joint_move_timeout_s", 100)

        # Make stall detection MUCH less aggressive near the goal:
        stall_eps = CFG.get("joint_stall_eps", 1e-6)       # was 1e-5
        stall_steps = CFG.get("joint_stall_steps", 80)     # was 50
        eps = CFG.get("joint_eps", 1e-3)
        near_eps = 3.0 * eps                                # treat as ?close enough?

        t0 = sim.getSimulationTime()
        last = sim.getJointPosition(joint)
        stall = 0

        while not sim.getSimulationStopping():
            if sim.getSimulationTime() - t0 > timeout:
                cur = sim.getJointPosition(joint)
                raise RuntimeError(f"{joint_name} timeout cur={cur:.6f} target={target:.6f}")

            cur = sim.getJointPosition(joint)
            err = target - cur

            # Reached?
            if abs(err) <= eps:
                break

            # If we've basically stopped moving but we're already very close, snap and finish:
            if abs(cur - last) < stall_eps and abs(err) <= near_eps:
                sim.setJointTargetPosition(joint, target)
                break

            # Real stall: not moving AND still far
            if abs(cur - last) < stall_eps and abs(err) > near_eps:
                stall += 1
                if stall >= stall_steps:
                    raise RuntimeError(f"{joint_name} stalled cur={cur:.6f} target={target:.6f} err={err:.6f}")
            else:
                stall = 0

            last = cur
            sim.setJointTargetPosition(joint, cur + clamp(err, -step, step))
            sim.wait(dt)

        sim.setJointTargetPosition(joint, target)

    def move_arm_sequential(self, target_config, order=(0,1,2,3,4), label="Move arm sequential", speed=None):
        with StepLogger(label):
            for idx in order:
                self.move_joint_smooth(self.arm[idx], target_config[idx], speed=speed, joint_name=f"arm[{idx}]")
            sim.wait(0.05)


    def move_arm_interpolated(self, target_config, joints=(1,2,3), duration=1.0, dt=0.02, label="Move arm interpolated"):
        with StepLogger(label):
            # Capture start joint positions
            start = {i: sim.getJointPosition(self.arm[i]) for i in joints}
            t0 = sim.getSimulationTime()

            while not sim.getSimulationStopping():
                t = sim.getSimulationTime() - t0
                alpha = min(1.0, t / max(1e-6, duration))

                # Interpolate each joint to target
                for i in joints:
                    qi = start[i] + alpha * (target_config[i] - start[i])
                    sim.setJointTargetPosition(self.arm[i], qi)

                if alpha >= 1.0:
                    break

                sim.wait(dt)

            # Snap to exact targets at end
            for i in joints:
                sim.setJointTargetPosition(self.arm[i], target_config[i])
            sim.wait(0.05)

    # -------------------------------------------------------------------------
    # Gripper
    # -------------------------------------------------------------------------

    def get_gripper_positions(self):
        j1 = sim.getJointPosition(self.gripper[0])
        j2 = sim.getJointPosition(self.gripper[1])
        return j1, j2

    def set_gripper_targets(self, j1, j2):
        sim.setJointTargetPosition(self.gripper[0], j1)
        sim.setJointTargetPosition(self.gripper[1], j2)

    def is_grip_confirmed(self):
        j1, j2 = self.get_gripper_positions()
        tol = CFG["gripper_tol"]

        j1_goal = CFG["gripper_close_goal_j1"]  # 0.018
        j2_goal = CFG["gripper_close_goal_j2"]  # -0.032

        # Closing directions:
        # - j1 decreases to close -> confirm if at or BELOW goal (within tol)
        # - j2 increases to close -> confirm if at or ABOVE goal (within tol)
        ok1 = (j1 <= j1_goal + tol)
        ok2 = (j2 >= j2_goal - tol)

        return ok1 and ok2

    def step_close_gripper_symmetric(self):
        j1, j2 = self.get_gripper_positions()
        j1_goal = CFG["gripper_close_goal_j1"]
        j2_goal = CFG["gripper_close_goal_j2"]

        # If already at/past goal in the closing direction, don't keep stepping:
        if j1 <= j1_goal:
            j1_next = j1_goal
        else:
            j1_next = j1 + CFG["gripper_step_j1"]  # negative step
            if j1_next < j1_goal:
                j1_next = j1_goal

        if j2 >= j2_goal:
            j2_next = j2_goal
        else:
            j2_next = j2 + CFG["gripper_step_j2"]  # positive step
            if j2_next > j2_goal:
                j2_next = j2_goal

        sim.setJointTargetPosition(self.gripper[0], j1_next)
        sim.setJointTargetPosition(self.gripper[1], j2_next)

    def open_gripper(self):
        with StepLogger("Open gripper"):
            # Detach held object first:
            self.detach_object_from_grip()

            sim.setJointTargetPosition(self.gripper[0], CFG["gripper_open_j1"])
            sim.setJointTargetPosition(self.gripper[1], CFG["gripper_open_j2"])
            sim.wait(0.3)

    def close_gripper_until_confirmed(self):
        with StepLogger("Close gripper slowly until confirmed (symmetric screw-joint closure)"):
            t_start = sim.getSimulationTime()
            timeout = CFG["gripper_close_timeout"]

            log_every = 10
            k = 0

            while not sim.getSimulationStopping():
                if sim.getSimulationTime() - t_start > timeout:
                    j1, j2 = self.get_gripper_positions()
                    raise RuntimeError(
                        f"Gripper close timeout. Current j1={j1:.4f}, j2={j2:.4f} "
                        f"(goals: j1={CFG['gripper_close_goal_j1']:.4f}, j2={CFG['gripper_close_goal_j2']:.4f})"
                    )

                if self.is_grip_confirmed():
                    j1, j2 = self.get_gripper_positions()
                    log_info(f"  grip confirmed at j1={j1:.4f}, j2={j2:.4f}")

                    # Attach cuboid to gripAttach:
                    self.attach_object_to_grip(self.cuboid)
                    log_info("  attached cuboid to gripAttach")
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

        # Parent under gripper attach dummy:
        sim.setObjectParent(obj_handle, self.grip_attach, True)

        # Freeze while carried so it follows:
        try:
            sim.setObjectInt32Param(obj_handle, sim.shapeintparam_static, 1)        # static while carried [1](https://www.opttek.com/products/simwrapper/simwrapper-python/getting-started/)[2](https://forum.coppeliarobotics.com/viewtopic.php?t=10268)
            sim.setObjectInt32Param(obj_handle, sim.shapeintparam_respondable, 0)  # reduce jitter collisions [1](https://www.opttek.com/products/simwrapper/simwrapper-python/getting-started/)[2](https://forum.coppeliarobotics.com/viewtopic.php?t=10268)
        except Exception:
            pass

        # Optional: reset only the cuboid, not the whole robot:
        try:
            sim.resetDynamicObject(obj_handle)  # rebuild its dynamics representation 
        except Exception:
            pass

        self.attached_object = obj_handle

        
    


    
    def detach_object_from_grip(self):
        if self.attached_object is None:
            return

        obj = self.attached_object

        parent = self._attached_prev_parent if self._attached_prev_parent is not None else -1
        sim.setObjectParent(obj, parent, True)

        # Force dynamic again so gravity works:
        try:
            sim.setObjectInt32Param(obj, sim.shapeintparam_static, 0)        # dynamic [1](https://www.opttek.com/products/simwrapper/simwrapper-python/getting-started/)[2](https://forum.coppeliarobotics.com/viewtopic.php?t=10268)
            sim.setObjectInt32Param(obj, sim.shapeintparam_respondable, 1)   # collide again [1](https://www.opttek.com/products/simwrapper/simwrapper-python/getting-started/)[2](https://forum.coppeliarobotics.com/viewtopic.php?t=10268)
        except Exception:
            pass

        try:
            sim.resetDynamicObject(obj)  # re-add to dynamics engine 
        except Exception:
            pass

        self.attached_object = None
        self._attached_prev_parent = None
        self._attached_prev_static = None



    # -------------------------------------------------------------------------
    # Pick sequence
    # -------------------------------------------------------------------------

    def pause_step(self, seconds, label):
        with StepLogger(label):
            sim.wait(seconds)

    def locate_cuboid(self):
        with StepLogger("Locate cuboid"):
            pos = self.get_cuboid_pos()
            log_info(f"  Cuboid position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
            return pos

    def go_pregrip(self):
        with StepLogger("Go to pregrip (hover): joints 0->1->2->3, then wrist align"):
            # Phase A: move joints 0..3 only (keep wrist as-is)
            cfg = self.get_pregrip_config_rad_no_wrist()

            log_info(
                "  pregrip target (rad) [no wrist]: " +
                ", ".join([f"{v:.3f}" for v in cfg])
            )

            self.move_arm_sequential(cfg, order=(0, 1, 2, 3), label="Arm to pregrip (0-3 only)", speed=CFG["arm_speed_pregrip"])

            # Phase B: now compute wrist based on sensor<->cube difference at the pregrip pose
            self._wrist_latched = None  # allow fresh compute now that we're at pregrip
            wrist = self.get_wrist_rotation_for_cuboid()

            log_info(f"  wrist target (rad): {wrist:.3f}")

            # Move wrist only
            self.move_joint_smooth(self.arm[4], wrist, speed=CFG["arm_speed_pregrip"])

    def go_grip_ready(self):
        with StepLogger("Go to grip-ready (descend) interpolated (no end sweep)"):
            cfg = self.get_grip_ready_config_rad()
            order = CFG.get("grip_ready_order", (3,2,1))

            self.move_arm_interpolated(
                cfg,
                joints=order,
                duration=CFG.get("grip_ready_duration_s", 1.2),
                dt=CFG.get("grip_ready_dt", CFG["dt"]),
                label=f"Grip-ready interpolated joints={order}"
            )

    def return_arm_to_neutral(self):
        with StepLogger("Return arm to neutral/carry"):
                j0, j1, j2, j3 = CFG["neutral_arm_deg"]
                wrist = deg2rad(CFG.get("neutral_wrist_deg", 0.0))  # or keep current wrist below

                # If you want to keep whatever wrist you currently have:
                # wrist = sim.getJointPosition(self.arm[4])

                target = [deg2rad(j0), deg2rad(j1), deg2rad(j2), deg2rad(j3), wrist]
                self.move_arm_sequential(target, order=(0,1,2,3,4), label="Arm to neutral/carry sequential")


    def pick_and_drop_one(self):
        with StepLogger("FULL PICK SEQUENCE"):
            try:
                self._wrist_latched = None
                self.turn_to_face_target(self.cuboid, label="Face cuboid")
                self.drive_forward_to_stop(CFG["stop_distance"])

                self.go_pregrip()
                self.pause_step(CFG["pregrip_pause_s"], "Pause at pregrip")
                self.go_grip_ready()

                self.close_gripper_until_confirmed()

                self.return_arm_to_neutral()
                self.drive_to_floor_edge()
                self.drop_cuboid_off_edge()

                self.go_tucked_pose()
                self.return_base_to_world_origin()

            finally:
                # Safety cleanup: ensure we?re not stuck holding something or driving
                self.stop_base()
                self.detach_object_from_grip()



    def run_mission_loop(self):
        with StepLogger("MISSION LOOP"):
            for path in CFG["pick_targets"]:
                h = sim.getObject(path, {"noError": True})
                if h == -1:
                    log_info(f"[skip] missing {path}")
                    continue

                self.cuboid = h
                log_info(f"[mission] picking {path}")

                try:
                    self.pick_and_drop_one()
                except Exception as e:
                    log_info(f"[mission] failed on {path}: {e}")
                    # decide: continue or abort
                    continue


# -----------------------------
# CoppeliaSim entry points
# -----------------------------
def sysCall_init():
    global sim, ctrl, INIT_OK
    sim = require("sim")  # embedded scripts access API via require forum.coppeliarobotics.com/viewtopic.php?t=10288)[1](https://manual.coppeliarobotics.com/en/accessingSceneObjects.htm)
    INIT_OK = False

    ctrl = YouBotPickController()

    try:
        ctrl.init_handles()
        INIT_OK = True
    except Exception as e:
        # Avoid crashing the wrapper: log and keep INIT_OK False
        log_info(f"[FATAL] Init failed: {e}")
        try:
            sim.addLog(sim.verbosity_errors, f"[FATAL] Init failed: {e}")
        except Exception:
            pass


def sysCall_thread():
    # Threaded scripts can safely block/yield with sim.wait [4](https://forum.coppeliarobotics.com/viewtopic.php?t=10288)
    if not INIT_OK:
        log_info("[ABORT] Init failed; not running sequence.")
        return

    sim.wait(0.2)
    ctrl.run_mission_loop()


def sysCall_cleanup():
    try:
        ctrl.stop_base()
    except Exception:
        pass

