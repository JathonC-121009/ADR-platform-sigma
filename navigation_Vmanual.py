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
POSITION_TOLERANCE_M = 0.20
YAW_TOLERANCE_DEG = 5.0
MOVE_TIMEOUT = 15.0

# How fast the drone is allowed to fly (meters per second)
MAX_FLIGHT_SPEED_M_S = 0.5 
# How fast the drone is allowed to yaw (degrees per second)
MAX_YAW_SPEED_DEG_S = 20.0 

NO_DETECTION_DIST = 999.0

CAM_OFFSET_RIGHT_M = -0.15   # Drone is too far right? Try a negative number like -0.15m
CAM_OFFSET_DOWN_M  = 0.10    # Drone is too far up? Try a positive number like 0.10m (Down is +Z)

# If the drone yaws slightly crooked to the gate, counter-rotate it here.
CAM_OFFSET_YAW_DEG = -3.0    # Try +/- degrees until it lines up perfectly straight


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
    """ Converts Camera/Body coordinates (+X Fwd, +Y Right, +Z Down) to Earth NED """
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

            # IMPORTANT: only use first gate index (the closest one)
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
            target = LocalTarget(vehicle.n, vehicle.e, vehicle.d, vehicle.yaw_rad)
        send_local_position_target(target)
        time.sleep(0.1)

    try:
        master.set_mode('OFFBOARD') 
        print("[*] Successfully entered Offboard mode!")
    except Exception as e:
        print(f"[!] Failed to set mode: {e}")


def send_local_position_target(target: LocalTarget):
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
        target.n, target.e, target.d,
        0, 0, 0, 0, 0, 0,
        target.yaw_rad, 0,
    )


def move_to_target(final_target: LocalTarget, label: str):
    """
    Moves to a target using a "Virtual Leash" to strictly control maximum speed.
    This interpolates the setpoint smoothly over time.
    """
    print(f"[*] Moving to {label}: N={final_target.n:.2f}, E={final_target.e:.2f}, D={final_target.d:.2f}, Yaw={rad_to_deg(final_target.yaw_rad):.1f} deg")

    start_time = time.time()
    period = 1.0 / POSITION_RATE_HZ
    
    max_pos_step = MAX_FLIGHT_SPEED_M_S * period
    max_yaw_step = deg_to_rad(MAX_YAW_SPEED_DEG_S) * period

    # Start our virtual target exactly where the drone is currently located
    with vehicle_lock:
        cmd_n, cmd_e, cmd_d, cmd_yaw = vehicle.n, vehicle.e, vehicle.d, vehicle.yaw_rad

    while running:
        # 1. Inch the virtual target towards the final destination
        dn = final_target.n - cmd_n
        de = final_target.e - cmd_e
        dd = final_target.d - cmd_d
        dist_to_final = math.sqrt(dn**2 + de**2 + dd**2)

        if dist_to_final > max_pos_step:
            cmd_n += (dn / dist_to_final) * max_pos_step
            cmd_e += (de / dist_to_final) * max_pos_step
            cmd_d += (dd / dist_to_final) * max_pos_step
        else:
            cmd_n, cmd_e, cmd_d = final_target.n, final_target.e, final_target.d

        # 2. Inch the yaw towards the final yaw
        yaw_err = wrap_pi(final_target.yaw_rad - cmd_yaw)
        if abs(yaw_err) > max_yaw_step:
            cmd_yaw += math.copysign(max_yaw_step, yaw_err)
        else:
            cmd_yaw = final_target.yaw_rad

        # 3. Send the step command
        step_target = LocalTarget(cmd_n, cmd_e, cmd_d, cmd_yaw)
        send_local_position_target(step_target)

        # 4. Check if actual drone has arrived at final destination
        with vehicle_lock:
            act_dn = final_target.n - vehicle.n
            act_de = final_target.e - vehicle.e
            act_dd = final_target.d - vehicle.d
            act_yaw_err = wrap_pi(final_target.yaw_rad - vehicle.yaw_rad)
            
        act_dist = math.sqrt(act_dn**2 + act_de**2 + act_dd**2)

        if act_dist < POSITION_TOLERANCE_M and abs(rad_to_deg(act_yaw_err)) < YAW_TOLERANCE_DEG:
            print(f"[*] Reached {label}")
            return True

        if time.time() - start_time > MOVE_TIMEOUT:
            print(f"[!] Timeout moving to {label}. Proceeding anyway.")
            return False

        time.sleep(period)


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
    """ Hold position for X seconds and average all valid gate detections """
    print(f"[*] Observing gate for {duration}s...")

    samples: List[GateDetection] = []
    t0 = time.time()
    
    # Grab current position to hold still
    hold_state = get_vehicle_snapshot()
    hold_target = LocalTarget(hold_state.n, hold_state.e, hold_state.d, hold_state.yaw_rad)

    while running and time.time() - t0 < duration:
        send_local_position_target(hold_target) # Keeps offboard alive
        
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
    # 1. APPLY PHYSICAL CAMERA OFFSETS
    # We adjust the AI's raw camera perception to account for the physical mounting errors
    corrected_right = det.right + CAM_OFFSET_RIGHT_M
    corrected_down = det.down + CAM_OFFSET_DOWN_M
    corrected_yaw_deg = det.yaw_deg + CAM_OFFSET_YAW_DEG

    # 2. TRANSFORM BODY TO EARTH (NED)
    dn, de, dd = body_to_local(det.forward, corrected_right, corrected_down, state.yaw_rad)

    gate_n = state.n + dn
    gate_e = state.e + de
    gate_d = state.d + dd

    # 3. APPLY YAW OFFSET TO FINAL GATE ORIENTATION
    gate_yaw = wrap_pi(state.yaw_rad + deg_to_rad(corrected_yaw_deg))

    return gate_n, gate_e, gate_d, gate_yaw


def build_standoff_target(det: GateDetection, standoff_m: float) -> LocalTarget:
    state = get_vehicle_snapshot()
    gate_n, gate_e, gate_d, gate_yaw = detection_to_gate_local(det, state)
    f_n, f_e = local_forward_vector(gate_yaw)

    # 1. Back up by standoff distance along the gate's forward vector
    target_n = gate_n - standoff_m * f_n
    target_e = gate_e - standoff_m * f_e
    
    # 2. Perfect Height Matching: target_d is set EXACTLY to gate_d
    target_d = gate_d 

    return LocalTarget(target_n, target_e, target_d, gate_yaw)


def build_pass_through_target(det: GateDetection, pass_dist_m: float) -> LocalTarget:
    state = get_vehicle_snapshot()
    gate_n, gate_e, gate_d, gate_yaw = detection_to_gate_local(det, state)
    f_n, f_e = local_forward_vector(gate_yaw)

    # 1. Push completely past the gate
    target_n = gate_n + pass_dist_m * f_n
    target_e = gate_e + pass_dist_m * f_e
    
    # 2. Perfect Height Matching
    target_d = gate_d 

    return LocalTarget(target_n, target_e, target_d, gate_yaw)


def run_gate_mission():
    gate_count = 0

    while running:
        print("\n==============================")
        print(f"[*] Looking for Gate {gate_count + 1}")
        print("==============================")

        # ---------------------------
        # STAGE 1: 3 Meters
        # ---------------------------
        gate = observe_gate(duration=1.0)
        if not gate:
            continue # If no gate, restart loop
        
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
        gate = observe_gate(duration=0.5) # Quick half-second final check
        if not gate:
            print("[!] Lost gate right before pass. Restarting.")
            continue
        
        # Calculate a point 1.5 meters beyond the gate
        pass_target = build_pass_through_target(gate, pass_dist_m=1.5)
        move_to_target(pass_target, "Through The Gate!")

        gate_count += 1
        print(f"[*] Successfully navigated Gate {gate_count}!")


# =========================
# SHUTDOWN
# =========================

def shutdown(sig=None, frame=None):
    global running
    print("\n[*] Shutting down")
    running = False
    time.sleep(0.2)
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)


# =========================
# MAIN
# =========================

if __name__ == "__main__":
    threading.Thread(target=udp_gate_listener, daemon=True).start()
    threading.Thread(target=mavlink_reader, daemon=True).start()

    wait_for_vehicle_state()
    
    enable_offboard_control()

    print("[*] Starting Multi-Stage Gate Mission")
    run_gate_mission()