import json
import math
import socket
import threading
import time
from dataclasses import dataclass, replace
from typing import Optional, Tuple

from pymavlink import mavutil


# =========================
# CONFIG
# =========================

UDP_IP = "127.0.0.1"
UDP_PORT = 5050

MAVLINK_CONN = "udpin:127.0.0.1:14551"

POSITION_RATE_HZ = 10
POSITION_TOLERANCE_M = 0.10
YAW_TOLERANCE_DEG = 5.0
MOVE_TIMEOUT = 15.0

# How old before a detection is considered stale and invalid (seconds)
DETECTION_MAX_AGE_S = 0.5

# --- VELOCITY CONTROLLER TUNING ---
MAX_FLIGHT_SPEED_M_S = 0.5
KP_POS = 1.2

# --- HARDWARE OFFSETS ---
CAM_OFFSET_RIGHT_M = -0.25
CAM_OFFSET_DOWN_M = -0.10
CAM_YAW_OFFSET_DEG = -10.0

NO_DETECTION_DIST = 999.0

# =========================
# HITBOX / GATE DEFAULTS
# =========================
# Drone hitbox expressed as (forward_length_m, width_m, height_m)
DRONE_HITBOX_M: Tuple[float, float, float] = (0.2286, 0.2286, 0.2286)  # 9 in = 0.2286 m cube
# Safety margin applied on each side of the hitbox (1 inch default)
HITBOX_MARGIN_M: float = 0.0254  # 1 inch in meters
# Default gate aperture dimensions (fallback if vision doesn't supply them)
GATE_DEFAULT_WIDTH_M: float = 1.0
GATE_DEFAULT_HEIGHT_M: float = 1.0
# How many short observe samples must all report a fit before committing
HITBOX_CONFIRM_SAMPLES: int = 3
HITBOX_CONFIRM_SAMPLE_DURATION_S: float = 0.2


# =========================
# DATA TYPES
# =========================


@dataclass
class VehicleState:
    """Latest MAVLink-derived local pose snapshot for the drone."""
    n: float = 0.0
    e: float = 0.0
    d: float = 0.0
    yaw_rad: float = 0.0
    have_local_position: bool = False
    have_attitude: bool = False


@dataclass
class GateDetection:
    """One gate observation coming from the vision pipeline over UDP."""
    timestamp: float
    dist: float
    forward: float
    right: float
    down: float
    roll: float
    pitch: float
    yaw_deg: float


@dataclass
class LocalTarget:
    """Position and yaw target expressed in local NED coordinates."""
    n: float
    e: float
    d: float
    yaw_rad: float


# =========================
# GEOMETRY / SMALL HELPERS
# =========================

def wrap_pi(angle: float) -> float:
    """Normalize any angle into the [-pi, pi) range."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0

def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi

def body_to_local(forward: float, right: float, down: float, yaw_rad: float):
    dn = math.cos(yaw_rad) * forward - math.sin(yaw_rad) * right
    de = math.sin(yaw_rad) * forward + math.cos(yaw_rad) * right
    dd = down
    return dn, de, dd

def local_forward_vector(yaw_rad: float):
    return math.cos(yaw_rad), math.sin(yaw_rad)

def is_valid_detection(det: Optional[GateDetection]) -> bool:
    if det is None:
        return False
    if det.dist == NO_DETECTION_DIST:
        return False
    # detection timestamp must be recent
    try:
        if (time.time() - det.timestamp) > DETECTION_MAX_AGE_S:
            return False
    except Exception:
        # if timestamp missing or invalid, treat as invalid
        return False
    return True


class NavigationController:
    """Owns MAVLink I/O, vehicle state, and reusable flight primitives."""
    def __init__(
        self,
        mavlink_conn: str,
    ):
        self.mavlink_conn = mavlink_conn

        self.master: Optional[mavutil.mavfile] = None

        self._running = threading.Event()
        self._vehicle = VehicleState()
        self._vehicle_lock = threading.Lock()
        # Control whether vertical movement commands are allowed. When False,
        # `send_velocity_and_yaw_target` will send zero vertical velocity.
        self._vertical_enabled = True
        self._vertical_lock = threading.Lock()
        self._mavlink_thread: Optional[threading.Thread] = None

    @property
    def running(self) -> bool:
        return self._running.is_set()

    # =========================
    # LIFECYCLE
    # =========================

    def start(self):
        """Connect to MAVLink, start the reader thread, and enter offboard."""
        if self.running:
            return

        # Clear any stale state so repeated missions do not reuse old pose flags.
        with self._vehicle_lock:
            self._vehicle = VehicleState()

        print("[*] Connecting to MAVLink...")
        self.master = mavutil.mavlink_connection(self.mavlink_conn, source_system=254)
        self.master.wait_heartbeat()
        print("[*] MAVLink heartbeat received")

        self._running.set()
        self._mavlink_thread = threading.Thread(
            target=self._mavlink_reader,
            daemon=True,
            name="nav-mavlink-reader",
        )
        self._mavlink_thread.start()

        self.wait_for_vehicle_state()
        self.enable_offboard_control()

    def stop(self):
        """Stop background work. The reader thread exits on its next loop."""
        self._running.clear()

        if self.master is not None:
            close = getattr(self.master, "close", None)
            if callable(close):
                close()
            self.master = None

        if self._mavlink_thread is not None and self._mavlink_thread.is_alive():
            self._mavlink_thread.join(timeout=1.0)

    def run_mission(self, mission):
        """Run one mission object against this controller with managed lifecycle."""
        self.start()
        try:
            mission.start()
            mission.run(self)
        finally:
            mission.stop()
            self.stop()

    # =========================
    # VEHICLE STATE ACCESS
    # =========================

    def get_vehicle_snapshot(self) -> VehicleState:
        with self._vehicle_lock:
            # Copy under the lock so callers can read a stable snapshot
            # without holding the shared-state lock themselves.
            return replace(self._vehicle)

    def _mavlink_reader(self):
        """Continuously fold incoming MAVLink messages into VehicleState."""
        if self.master is None:
            return

        while self.running:
            msg = self.master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue

            msg_type = msg.get_type()
            with self._vehicle_lock:
                if msg_type == "LOCAL_POSITION_NED":
                    self._vehicle.n = float(msg.x)
                    self._vehicle.e = float(msg.y)
                    self._vehicle.d = float(msg.z)
                    self._vehicle.have_local_position = True
                elif msg_type == "ATTITUDE":
                    self._vehicle.yaw_rad = float(msg.yaw)
                    self._vehicle.have_attitude = True

    # =========================
    # FLIGHT PRIMITIVES
    # =========================

    def wait_for_vehicle_state(self):
        """Block until both local position and attitude have been observed."""
        print("[*] Waiting for LOCAL_POSITION_NED and ATTITUDE...")
        while self.running:
            state = self.get_vehicle_snapshot()
            if state.have_local_position and state.have_attitude:
                print("[*] Vehicle state ready")
                return
            time.sleep(0.1)

    def enable_offboard_control(self):
        """Prime PX4 with a few setpoints, then request OFFBOARD mode."""
        print("[*] Requesting Offboard Control...")
        for _ in range(10):
            current_yaw = self.get_vehicle_snapshot().yaw_rad
            self.send_velocity_and_yaw_target(0.0, 0.0, 0.0, current_yaw)
            time.sleep(0.1)

        try:
            assert self.master is not None
            self.master.set_mode("OFFBOARD")
            print("[*] Successfully entered Offboard mode!")
        except Exception as exc:
            print(f"[!] Failed to set mode: {exc}")

    def send_velocity_and_yaw_target(self, vn: float, ve: float, vd: float, yaw_rad: float):
        """Send one MAVLink local-NED velocity command with an absolute yaw target."""
        if self.master is None:
            raise RuntimeError("MAVLink connection not started")

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        # If vertical commands are disabled (e.g. during a pass-through), force vd=0.
        with self._vertical_lock:
            if not self._vertical_enabled:
                vd_to_send = 0.0
            else:
                vd_to_send = vd

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0,
            0,
            0,
            vn,
            ve,
            vd_to_send,
            0,
            0,
            0,
            yaw_rad,
            0,
        )

    def set_vertical_enabled(self, enabled: bool) -> None:
        """Enable or disable sending vertical velocity commands.

        When disabled, `send_velocity_and_yaw_target` will send `vd=0.0`.
        This is thread-safe and intended for short critical sections such as
        pass-through maneuvers.
        """
        with self._vertical_lock:
            self._vertical_enabled = bool(enabled)

    def move_to_target(
        self,
        final_target: LocalTarget,
        label: str,
        *,
        max_speed_m_s: float = MAX_FLIGHT_SPEED_M_S,
        timeout_s: float = MOVE_TIMEOUT,
    ) -> bool:
        """Drive to a local target with per-move speed and timeout limits."""
        if not math.isfinite(max_speed_m_s) or max_speed_m_s <= 0.0:
            raise ValueError("max_speed_m_s must be a positive finite number")
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be a positive finite number")

        print(
            f"[*] Moving to {label}: "
            f"N={final_target.n:.2f}, E={final_target.e:.2f}, "
            f"D={final_target.d:.2f}, Yaw={rad_to_deg(final_target.yaw_rad):.1f} deg, "
            f"MaxSpeed={max_speed_m_s:.2f}m/s"
        )

        start_time = time.time()
        period = 1.0 / POSITION_RATE_HZ

        while self.running:
            state = self.get_vehicle_snapshot()
            err_n = final_target.n - state.n
            err_e = final_target.e - state.e
            err_d = final_target.d - state.d
            yaw_err = wrap_pi(final_target.yaw_rad - state.yaw_rad)

            dist_xyz = math.sqrt(err_n**2 + err_e**2 + err_d**2)
            if dist_xyz < POSITION_TOLERANCE_M and abs(rad_to_deg(yaw_err)) < YAW_TOLERANCE_DEG:
                print(f"[*] Reached {label}")
                self.send_velocity_and_yaw_target(0.0, 0.0, 0.0, final_target.yaw_rad)
                return True

            if time.time() - start_time > timeout_s:
                print(f"[!] Timeout moving to {label}. Proceeding anyway.")
                self.send_velocity_and_yaw_target(0.0, 0.0, 0.0, final_target.yaw_rad)
                return False

            vn = KP_POS * err_n
            ve = KP_POS * err_e
            vd = KP_POS * err_d

            cmd_speed = math.sqrt(vn**2 + ve**2 + vd**2)
            if cmd_speed > max_speed_m_s:
                vn = (vn / cmd_speed) * max_speed_m_s
                ve = (ve / cmd_speed) * max_speed_m_s
                vd = (vd / cmd_speed) * max_speed_m_s

            self.send_velocity_and_yaw_target(vn, ve, vd, final_target.yaw_rad)
            time.sleep(period)

        return False

    def turn_around_180(self):
        """Hold position and command a 180-degree yaw change."""
        print("[*] 0 Gate Detections! Turning 180 degrees...")
        state = self.get_vehicle_snapshot()
        target = LocalTarget(
            n=state.n,
            e=state.e,
            d=state.d,
            yaw_rad=wrap_pi(state.yaw_rad + math.pi),
        )
        self.move_to_target(target, "180 Degree Turnaround")

    def land(self):
        """Send a land command after first stabilizing with a zero-velocity setpoint."""
        hold_yaw = self.get_vehicle_snapshot().yaw_rad
        self.send_velocity_and_yaw_target(0.0, 0.0, 0.0, hold_yaw)
        time.sleep(0.5)

        try:
            assert self.master is not None
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            print("[*] Land command sent successfully.")
        except Exception as exc:
            print(f"[!] Failed to send land command: {exc}")

class Mission:
    """Minimal mission interface used by NavigationController.run_mission()."""
    def start(self):
        pass

    def run(self, nav: NavigationController):
        raise NotImplementedError

    def visual_servo_approach(
        self,
        nav: NavigationController,
        det: Optional[GateDetection] = None,
        *,
        max_correction_m: float = 0.25,
        kp: float = 0.8,
        duration_s: float = 2.0,
        sample_interval: float = 0.1,
        max_speed_override: float = 0.20,
        dist_tolerance: float = 1.0,
    ) -> bool:
        """Conservative visual-servo that recenters the drone on the observed gate.

        Safety-first design principles:
        - Abort if the detection disappears or distance changes dramatically.
        - Clamp commanded speeds below `max_speed_override` and `MAX_FLIGHT_SPEED_M_S`.
        - Send a zero-velocity setpoint on exit to avoid leaving the vehicle moving.
        - Non-destructive: does not change mission state or lasting offsets.
        Returns True if the routine executed (even if it aborted early), False on invalid inputs.
        """
        if det is None:
            det = self.get_latest_detection_snapshot()

        if not is_valid_detection(det):
            print("[!] visual_servo_approach: no valid initial detection, skipping")
            return False

        initial_dist = det.dist
        start_t = time.time()
        hold_yaw = nav.get_vehicle_snapshot().yaw_rad

        period = max(0.01, float(sample_interval))
        max_speed = min(max_speed_override, MAX_FLIGHT_SPEED_M_S)

        try:
            while nav.running and (time.time() - start_t) < float(duration_s):
                current = self.get_latest_detection_snapshot()
                if not is_valid_detection(current):
                    print("[!] visual_servo_approach: detection lost, aborting servo")
                    break

                # If distance changes too much, assume we've switched targets or moved; abort
                if abs(current.dist - initial_dist) > float(dist_tolerance):
                    print("[!] visual_servo_approach: large distance change, aborting servo")
                    break

                # Body-frame errors: apply camera yaw offset to translate camera-frame
                # forward/right into body-forward/right before using offsets.
                theta = deg_to_rad(self.cam_yaw_offset_deg)
                f_b = current.forward * math.cos(theta) - current.right * math.sin(theta)
                r_b = current.forward * math.sin(theta) + current.right * math.cos(theta)

                err_right = r_b + self.cam_offset_right_m
                err_down = current.down + self.cam_offset_down_m

                # P controller in meters -> m/s
                body_v_forward = 0.0
                body_v_right = kp * err_right
                body_v_down = kp * err_down

                mag = math.sqrt(body_v_forward**2 + body_v_right**2 + body_v_down**2)
                if mag > 0.0 and mag > max_speed:
                    scale = max_speed / mag
                    body_v_forward *= scale
                    body_v_right *= scale
                    body_v_down *= scale

                # Convert body velocities to local NED using the current vehicle yaw.
                vn, ve, vd = body_to_local(body_v_forward, body_v_right, body_v_down, hold_yaw)

                # Command yaw aligned to the observed gate yaw (with camera yaw offset applied)
                try:
                    yaw_cmd = wrap_pi(hold_yaw + deg_to_rad(current.yaw_deg + self.cam_yaw_offset_deg))
                except Exception:
                    yaw_cmd = hold_yaw

                # Send conservative velocity + yaw setpoint
                try:
                    nav.send_velocity_and_yaw_target(vn, ve, vd, yaw_cmd)
                except Exception as exc:
                    print(f"[!] visual_servo_approach: failed to send setpoint: {exc}")
                    break

                time.sleep(period)

        finally:
            # Always send a zero-velocity setpoint to leave vehicle stable.
            try:
                nav.send_velocity_and_yaw_target(0.0, 0.0, 0.0, hold_yaw)
            except Exception:
                pass

        return True
    def stop(self):
        pass


class GateMission(Mission):
    """Base class for missions that consume gate detections over UDP."""
    def __init__(
        self,
        udp_ip: str = UDP_IP,
        udp_port: int = UDP_PORT,
        *,
        cam_offset_right_m: float = CAM_OFFSET_RIGHT_M,
        cam_offset_down_m: float = CAM_OFFSET_DOWN_M,
        cam_yaw_offset_deg: float = CAM_YAW_OFFSET_DEG,
    ):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.cam_offset_right_m = cam_offset_right_m
        self.cam_offset_down_m = cam_offset_down_m
        self.cam_yaw_offset_deg = cam_yaw_offset_deg

        self._running = threading.Event()
        self._latest_detection: Optional[GateDetection] = None
        self._detection_lock = threading.Lock()
        self._udp_thread: Optional[threading.Thread] = None
        # Debounce settings to avoid rapid flipping between detections
        self._debounce_candidate: Optional[GateDetection] = None
        self._debounce_count: int = 0
        self._debounce_required: int = 3  # consecutive frames required to accept a candidate
        self._debounce_pos_thresh_m: float = 0.2  # lateral/vertical change below this is considered same
        self._debounce_dist_improve_m: float = 0.2  # immediate accept if new detection is this much closer

    @property
    def running(self) -> bool:
        return self._running.is_set()

    def start(self):
        """Start the background UDP listener that receives gate detections."""
        if self.running:
            return

        with self._detection_lock:
            self._latest_detection = None

        self._running.set()
        self._udp_thread = threading.Thread(
            target=self._udp_gate_listener,
            daemon=True,
            name="gate-mission-udp-listener",
        )
        self._udp_thread.start()

    def stop(self):
        """Stop the UDP listener loop."""
        self._running.clear()

        if self._udp_thread is not None and self._udp_thread.is_alive():
            self._udp_thread.join(timeout=1.0)

    def get_latest_detection_snapshot(self) -> Optional[GateDetection]:
        with self._detection_lock:
            if self._latest_detection is None:
                return None
            # Like vehicle snapshots, callers get a copy instead of the shared object.
            return replace(self._latest_detection)

    def _maybe_update_detection(self, new_det: GateDetection) -> None:
        """Conservative debounce: only replace _latest_detection when it's clearly
        better or when the same candidate appears for a few consecutive frames.
        Runs under no lock; it will acquire the detection lock internally.
        """
        if new_det is None:
            return

        with self._detection_lock:
            cur = self._latest_detection

            # If we have no current detection, accept immediately.
            if cur is None:
                self._latest_detection = replace(new_det)
                self._debounce_candidate = None
                self._debounce_count = 0
                return

            # If new detection is substantially closer, accept immediately.
            if new_det.dist + 0.0 < (cur.dist - self._debounce_dist_improve_m):
                self._latest_detection = replace(new_det)
                self._debounce_candidate = None
                self._debounce_count = 0
                return

            # If new detection is near the current one (small lateral/vertical change), refresh.
            lateral_diff = abs(new_det.right - cur.right)
            vertical_diff = abs(new_det.down - cur.down)
            if lateral_diff <= self._debounce_pos_thresh_m and vertical_diff <= self._debounce_pos_thresh_m:
                # update stored detection to the average for smoothing
                avg = GateDetection(
                    timestamp=time.time(),
                    dist=(new_det.dist + cur.dist) / 2.0,
                    forward=(new_det.forward + cur.forward) / 2.0,
                    right=(new_det.right + cur.right) / 2.0,
                    down=(new_det.down + cur.down) / 2.0,
                    roll=(new_det.roll + cur.roll) / 2.0,
                    pitch=(new_det.pitch + cur.pitch) / 2.0,
                    yaw_deg=(new_det.yaw_deg + cur.yaw_deg) / 2.0,
                )
                self._latest_detection = avg
                self._debounce_candidate = None
                self._debounce_count = 0
                return

            # Otherwise, consider it a candidate and require consecutive frames to accept.
            if self._debounce_candidate is None:
                self._debounce_candidate = new_det
                self._debounce_count = 1
                return

            # If candidate roughly matches previous candidate, increment counter.
            cand = self._debounce_candidate
            if abs(new_det.dist - cand.dist) < 0.5 and abs(new_det.right - cand.right) < 0.5:
                self._debounce_count += 1
            else:
                # different candidate, restart counting
                self._debounce_candidate = new_det
                self._debounce_count = 1

            if self._debounce_count >= self._debounce_required:
                self._latest_detection = replace(self._debounce_candidate)
                self._debounce_candidate = None
                self._debounce_count = 0

    def _udp_gate_listener(self):
        """Listen for the latest vision packet and keep only the newest gate."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.udp_ip, self.udp_port))
        sock.settimeout(0.2)

        print(f"[*] Listening for gate telemetry on UDP {self.udp_ip}:{self.udp_port}")

        try:
            while self.running:
                try:
                    data, _ = sock.recvfrom(4096)
                    payload = json.loads(data.decode("utf-8"))
                    gates = payload.get("gates", [])
                    if not gates:
                        continue

                    gate = gates[0]
                    det = GateDetection(
                        timestamp=time.time(),
                        dist=float(gate[0]),
                        forward=float(gate[1]),
                        right=float(gate[2]),
                        down=float(gate[3]),
                        roll=float(gate[4]),
                        pitch=float(gate[5]),
                        yaw_deg=float(gate[6]),
                    )

                    # Use debounce logic to avoid rapid flipping between similar detections.
                    try:
                        self._maybe_update_detection(det)
                    except Exception as exc:
                        # Never let the UDP thread crash for a non-critical logic error.
                        print(f"[!] debounce update error: {exc}")

                except socket.timeout:
                    continue
                except OSError:
                    if self.running:
                        print("[!] UDP gate listener socket error.")
                    return
                except Exception as exc:
                    print(f"[!] UDP gate listener error: {exc}")
        finally:
            sock.close()

    def observe_gate(self, nav: NavigationController, duration: float = 1.0) -> Optional[GateDetection]:
        """Hover in place, sample detections for a short window, then average them."""
        print(f"[*] Observing gate for {duration}s...")

        samples = []
        t0 = time.time()
        hold_yaw = nav.get_vehicle_snapshot().yaw_rad

        while nav.running and time.time() - t0 < duration:
            nav.send_velocity_and_yaw_target(0.0, 0.0, 0.0, hold_yaw)

            det = self.get_latest_detection_snapshot()
            if is_valid_detection(det):
                samples.append(det)

            time.sleep(0.04)

        if not samples:
            print("[!] No valid gate detections seen.")
            return None

        avg = GateDetection(
            timestamp=time.time(),
            dist=sum(d.dist for d in samples) / len(samples),
            forward=sum(d.forward for d in samples) / len(samples),
            right=sum(d.right for d in samples) / len(samples),
            down=sum(d.down for d in samples) / len(samples),
            roll=sum(d.roll for d in samples) / len(samples),
            pitch=sum(d.pitch for d in samples) / len(samples),
            yaw_deg=sum(d.yaw_deg for d in samples) / len(samples),
        )

        print(f"[*] Lock Acquired: Dist={avg.dist:.2f}m, Yaw={avg.yaw_deg:.1f}deg")
        return avg

    def detection_to_gate_local(self, det: GateDetection, state: VehicleState):
        """Convert a camera-relative gate detection into local NED gate pose."""
        # Rotate camera-frame translation into body-frame using camera yaw offset,
        # then apply the fixed camera lever-arm offsets before projecting into local NED.
        theta = deg_to_rad(self.cam_yaw_offset_deg)
        f_b = det.forward * math.cos(theta) - det.right * math.sin(theta)
        r_b = det.forward * math.sin(theta) + det.right * math.cos(theta)

        corrected_right = r_b + self.cam_offset_right_m
        corrected_down = det.down + self.cam_offset_down_m

        dn, de, dd = body_to_local(f_b, corrected_right, corrected_down, state.yaw_rad)
        gate_n = state.n + dn
        gate_e = state.e + de
        gate_d = state.d + dd

        corrected_yaw_deg = det.yaw_deg + self.cam_yaw_offset_deg
        gate_yaw = wrap_pi(state.yaw_rad + deg_to_rad(corrected_yaw_deg))

        return gate_n, gate_e, gate_d, gate_yaw

    def hitbox_fits_gate(self, det: GateDetection, hitbox_dims: tuple = DRONE_HITBOX_M, margin_m: float = HITBOX_MARGIN_M) -> bool:
        """Return True if the configured hitbox (plus margin) fits the observed gate aperture.

        This is a conservative axis-aligned check using available gate dimensions. If the
        vision detection does not include explicit aperture sizes, fall back to module
        defaults `GATE_DEFAULT_WIDTH_M` and `GATE_DEFAULT_HEIGHT_M`.
        """
        # If the detector includes explicit aperture fields, prefer them. Otherwise
        # fall back to conservative defaults.
        gate_width = getattr(det, "width", None) or GATE_DEFAULT_WIDTH_M
        gate_height = getattr(det, "height", None) or GATE_DEFAULT_HEIGHT_M

        # hitbox_dims is (forward_length, width, height)
        _, hb_width, hb_height = hitbox_dims

        lateral_ok = gate_width >= (hb_width + 2.0 * margin_m)
        vertical_ok = gate_height >= (hb_height + 2.0 * margin_m)

        print(f"[*] Hitbox check: gate_w={gate_width:.2f}, gate_h={gate_height:.2f}, hb_w={hb_width:.2f}, hb_h={hb_height:.2f}, margin={margin_m:.3f} => lateral_ok={lateral_ok}, vertical_ok={vertical_ok}")
        return lateral_ok and vertical_ok

    def confirm_hitbox_fits(self, nav: NavigationController, det: GateDetection, samples: int = HITBOX_CONFIRM_SAMPLES, sample_duration: float = HITBOX_CONFIRM_SAMPLE_DURATION_S) -> bool:
        """Confirm the hitbox fit across several short observe samples to avoid
        transient false positives/negatives from noisy detections."""
        for _ in range(samples):
            obs = self.observe_gate(nav, duration=sample_duration)
            if not obs:
                return False
            if not self.hitbox_fits_gate(obs):
                return False
        return True

    def build_standoff_target(
        self,
        nav: NavigationController,
        det: GateDetection,
        standoff_m: float,
    ) -> LocalTarget:
        """Build a target that stops in front of the gate by the given standoff."""
        state = nav.get_vehicle_snapshot()
        gate_n, gate_e, gate_d, gate_yaw = self.detection_to_gate_local(det, state)
        f_n, f_e = local_forward_vector(gate_yaw)

        return LocalTarget(
            n=gate_n - standoff_m * f_n,
            e=gate_e - standoff_m * f_e,
            d=gate_d,
            yaw_rad=gate_yaw,
        )

    def build_pass_through_target(
        self,
        nav: NavigationController,
        det: GateDetection,
        pass_dist_m: float,
    ) -> LocalTarget:
        """Build a target that carries the drone through and past the gate."""
        state = nav.get_vehicle_snapshot()
        gate_n, gate_e, gate_d, gate_yaw = self.detection_to_gate_local(det, state)
        f_n, f_e = local_forward_vector(gate_yaw)

        return LocalTarget(
            n=gate_n + pass_dist_m * f_n,
            e=gate_e + pass_dist_m * f_e,
            d=gate_d,
            yaw_rad=gate_yaw,
        )

    def perform_pass_through(self, nav: NavigationController, det: GateDetection, pass_dist_m: float, *, max_speed_m_s: float = 0.15, label: str = "Through The Gate!") -> bool:
        """Helper that disables vertical commands, executes the pass-through move, and restores vertical control.

        Returns the boolean result from `move_to_target`.
        """
        target = self.build_pass_through_target(nav, det, pass_dist_m)
        try:
            nav.set_vertical_enabled(False)
            return nav.move_to_target(target, label, max_speed_m_s=max_speed_m_s)
        finally:
            nav.set_vertical_enabled(True)

    def run(self, nav: NavigationController):
        raise NotImplementedError