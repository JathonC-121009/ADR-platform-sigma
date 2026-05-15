import socket
import json
import time
import math
import threading
import signal
import sys
from dataclasses import dataclass
from typing import Optional, List

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

# --- VELOCITY CONTROLLER TUNING ---
MAX_FLIGHT_SPEED_M_S = 0.5   # Maximum speed cap
KP_POS = 1.2                 # Proportional Gain (Higher = more aggressive correction)

# --- HARDWARE OFFSETS ---
CAM_OFFSET_RIGHT_M = -0.25   # Negative = shift drone Left | Positive = shift drone Right
CAM_OFFSET_DOWN_M  = -0.10   # Negative = shift drone Up   | Positive = shift drone Down
CAM_YAW_OFFSET_DEG = -10.0   # Negative = Yaw Left | Positive = Yaw Right

NO_DETECTION_DIST = 999.0


# =========================
# DATA TYPES
# =========================

@dataclass
class VehicleState:
    n: float = 0.0
    e: float = 0.0
    d: float = 0.0
    yaw_rad: float = 0.0
    have_local_position: bool = False
    have_attitude: bool = False


@dataclass
class GateDetection:
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
    n: float
    e: float
    d: float
    yaw_rad: float


# =========================
# SHARED STATE
# =========================

vehicle = VehicleState()
vehicle_lock = threading.Lock()

latest_detection: Optional[GateDetection] = None
detections_log: List[GateDetection] = []
detection_lock = threading.Lock()

running = True


# =========================
# UTILS
# =========================

def wrap_pi(angle: float) -> float:
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
    return det is not None and det.dist != NO_DETECTION_DIST


# =========================
# UDP GATE LISTENER
# =========================

def udp_gate_listener():
    global latest_detection

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.2)

    print(f"[*] Listening for gate telemetry on UDP {UDP_IP}:{UDP_PORT}")

    while running:
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

            with detection_lock:
                latest_detection = det
                detections_log.append(det)

        except socket.timeout:
            continue
        except Exception as e:
            pass


# =========================
# MAVLINK & OFFBOARD CONTROL
# =========================

print("[*] Connecting to MAVLink...")
master = mavutil.mavlink_connection(MAVLINK_CONN, source_system=254)
master.wait_heartbeat()
print("[*] MAVLink heartbeat received")


def mavlink_reader():
    while running:
        msg = master.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue

        msg_type = msg.get_type()

        with vehicle_lock:
            if msg_type == "LOCAL_POSITION_NED":
                vehicle.n = float(msg.x)
                vehicle.e = float(msg.y)
                vehicle.d = float(msg.z)
                vehicle.have_local_position = True

            elif msg_type == "ATTITUDE":
                vehicle.yaw_rad = float(msg.yaw)
                vehicle.have_attitude = True


def wait_for_vehicle_state():
    print("[*] Waiting for LOCAL_POSITION_NED and ATTITUDE...")
    while running:
        with vehicle_lock:
            if vehicle.have_local_position and vehicle.have_attitude:
                print("[*] Vehicle state ready")
                return
        time.sleep(0.1)


def enable_offboard_control():
    print("[*] Requesting Offboard Control...")
    for _ in range(10):
        with vehicle_lock:
            current_yaw = vehicle.yaw_rad
        send_velocity_and_yaw_target(0.0, 0.0, 0.0, current_yaw)
        time.sleep(0.1)

    try:
        master.set_mode('OFFBOARD') 
        print("[*] Successfully entered Offboard mode!")
    except Exception as e:
        print(f"[!] Failed to set mode: {e}")


def send_velocity_and_yaw_target(vn: float, ve: float, vd: float, yaw_rad: float):
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
        0, 0, 0,    
        vn, ve, vd, 
        0, 0, 0,    
        yaw_rad, 0  
    )


def move_to_target(final_target: LocalTarget, label: str):
    """ Closed-Loop Proportional (P) Velocity Controller """
    print(f"[*] Moving to {label}: N={final_target.n:.2f}, E={final_target.e:.2f}, D={final_target.d:.2f}, Yaw={rad_to_deg(final_target.yaw_rad):.1f} deg")

    start_time = time.time()
    period = 1.0 / POSITION_RATE_HZ

    while running:
        with vehicle_lock:
            err_n = final_target.n - vehicle.n
            err_e = final_target.e - vehicle.e
            err_d = final_target.d - vehicle.d
            yaw_err = wrap_pi(final_target.yaw_rad - vehicle.yaw_rad)

        dist_xyz = math.sqrt(err_n**2 + err_e**2 + err_d**2)

        if dist_xyz < POSITION_TOLERANCE_M and abs(rad_to_deg(yaw_err)) < YAW_TOLERANCE_DEG:
            print(f"[*] Reached {label}")
            send_velocity_and_yaw_target(0.0, 0.0, 0.0, final_target.yaw_rad)
            return True

        if time.time() - start_time > MOVE_TIMEOUT:
            print(f"[!] Timeout moving to {label}. Proceeding anyway.")
            send_velocity_and_yaw_target(0.0, 0.0, 0.0, final_target.yaw_rad)
            return False

        vn = KP_POS * err_n
        ve = KP_POS * err_e
        vd = KP_POS * err_d

        cmd_speed = math.sqrt(vn**2 + ve**2 + vd**2)
        if cmd_speed > MAX_FLIGHT_SPEED_M_S:
            vn = (vn / cmd_speed) * MAX_FLIGHT_SPEED_M_S
            ve = (ve / cmd_speed) * MAX_FLIGHT_SPEED_M_S
            vd = (vd / cmd_speed) * MAX_FLIGHT_SPEED_M_S

        send_velocity_and_yaw_target(vn, ve, vd, final_target.yaw_rad)
        
        time.sleep(period)


def turn_around_180():
    """ Holds current XYZ position and spins exactly 180 degrees """
    print("[*] 0 Gate Detections! Turning 180 degrees...")
    with vehicle_lock:
        target_n = vehicle.n
        target_e = vehicle.e
        target_d = vehicle.d
        target_yaw = wrap_pi(vehicle.yaw_rad + math.pi) # Add 180 degrees
    
    target = LocalTarget(target_n, target_e, target_d, target_yaw)
    move_to_target(target, "180 Degree Turnaround")


def land_drone():
    """ Sends the universal MAVLink command to initiate a landing """
    print("\n==============================")
    print("[*] MISSION COMPLETE (8 GATES). INITIATING LANDING!")
    print("==============================")
    
    # Send 0 velocity command just to stabilize before land
    with vehicle_lock:
        send_velocity_and_yaw_target(0.0, 0.0, 0.0, vehicle.yaw_rad)
    time.sleep(0.5)

    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("[*] Land command sent successfully.")
    except Exception as e:
        print(f"[!] Failed to send land command: {e}")


# =========================
# MISSION LOGIC
# =========================

def get_vehicle_snapshot():
    with vehicle_lock:
        return VehicleState(vehicle.n, vehicle.e, vehicle.d, vehicle.yaw_rad, vehicle.have_local_position, vehicle.have_attitude)

def get_latest_detection_snapshot() -> Optional[GateDetection]:
    with detection_lock:
        return latest_detection


def observe_gate(duration: float = 1.0) -> Optional[GateDetection]:
    """ Hover perfectly still for X seconds and average all gate detections """
    print(f"[*] Observing gate for {duration}s...")

    samples: List[GateDetection] = []
    t0 = time.time()
    
    with vehicle_lock:
        hold_yaw = vehicle.yaw_rad

    while running and time.time() - t0 < duration:
        send_velocity_and_yaw_target(0.0, 0.0, 0.0, hold_yaw)
        
        det = get_latest_detection_snapshot()
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


def detection_to_gate_local(det: GateDetection, state: VehicleState):
    corrected_right = det.right + CAM_OFFSET_RIGHT_M
    corrected_down = det.down + CAM_OFFSET_DOWN_M
    
    dn, de, dd = body_to_local(det.forward, corrected_right, corrected_down, state.yaw_rad)

    gate_n = state.n + dn
    gate_e = state.e + de
    gate_d = state.d + dd

    corrected_yaw_deg = det.yaw_deg + CAM_YAW_OFFSET_DEG
    gate_yaw = wrap_pi(state.yaw_rad + deg_to_rad(corrected_yaw_deg))
    
    return gate_n, gate_e, gate_d, gate_yaw


def build_standoff_target(det: GateDetection, standoff_m: float) -> LocalTarget:
    state = get_vehicle_snapshot()
    gate_n, gate_e, gate_d, gate_yaw = detection_to_gate_local(det, state)
    f_n, f_e = local_forward_vector(gate_yaw)

    target_n = gate_n - standoff_m * f_n
    target_e = gate_e - standoff_m * f_e
    target_d = gate_d 

    return LocalTarget(target_n, target_e, target_d, gate_yaw)


def build_pass_through_target(det: GateDetection, pass_dist_m: float) -> LocalTarget:
    state = get_vehicle_snapshot()
    gate_n, gate_e, gate_d, gate_yaw = detection_to_gate_local(det, state)
    f_n, f_e = local_forward_vector(gate_yaw)

    target_n = gate_n + pass_dist_m * f_n
    target_e = gate_e + pass_dist_m * f_e
    target_d = gate_d 

    return LocalTarget(target_n, target_e, target_d, gate_yaw)


def run_gate_mission():
    gate_count = 0

    while running and gate_count < 8:
        print("\n==============================")
        print(f"[*] Looking for Gate {gate_count + 1} of 8")
        print("==============================")

        # ---------------------------
        # STAGE 1: 3 Meters
        # ---------------------------
        gate = observe_gate(duration=1.0)
        if not gate:
            # If we see 0 gates at Stage 1, we must be at the end of the course. Turn around!
            turn_around_180()
            continue
        
        target_3m = build_standoff_target(gate, standoff_m=3.0)
        move_to_target(target_3m, "3m Standoff")

        # ---------------------------
        # STAGE 2: 2 Meters
        # ---------------------------
        gate = observe_gate(duration=1.0)
        if not gate:
            print("[!] Lost gate at 3m. Restarting.")
            continue
        
        target_2m = build_standoff_target(gate, standoff_m=2.0)
        move_to_target(target_2m, "2m Standoff")

        # ---------------------------
        # STAGE 3: 1 Meter
        # ---------------------------
        gate = observe_gate(duration=1.0)
        if not gate:
            print("[!] Lost gate at 2m. Restarting.")
            continue
        
        target_1m = build_standoff_target(gate, standoff_m=1.0)
        move_to_target(target_1m, "1m Standoff")

        # ---------------------------
        # STAGE 4: Fly Through
        # ---------------------------
        gate = observe_gate(duration=0.5)
        if not gate:
            print("[!] Lost gate right before pass. Restarting.")
            continue
        
        pass_target = build_pass_through_target(gate, pass_dist_m=1.5)
        move_to_target(pass_target, "Through The Gate!")

        gate_count += 1
        print(f"[*] Successfully navigated Gate {gate_count}!")

    # Loop exits when gate_count hits 8
    if gate_count >= 8:
        land_drone()


# =========================
# SHUTDOWN
# =========================

def shutdown(sig=None, frame=None):
    global running