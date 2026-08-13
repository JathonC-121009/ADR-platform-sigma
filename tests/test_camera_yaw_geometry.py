import sys
import types
import math

# Stub pymavlink so navigation can import in the test environment.
pymavlink = types.ModuleType("pymavlink")
# minimal mavutil/mavlink constants used at import time
mavlink = types.SimpleNamespace(
    POSITION_TARGET_TYPEMASK_X_IGNORE=0,
    POSITION_TARGET_TYPEMASK_Y_IGNORE=0,
    POSITION_TARGET_TYPEMASK_Z_IGNORE=0,
    POSITION_TARGET_TYPEMASK_AX_IGNORE=0,
    POSITION_TARGET_TYPEMASK_AY_IGNORE=0,
    POSITION_TARGET_TYPEMASK_AZ_IGNORE=0,
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE=0,
    MAV_FRAME_LOCAL_NED=0,
)
mavutil = types.SimpleNamespace(mavlink=mavlink)
setattr(pymavlink, "mavutil", mavutil)
sys.modules["pymavlink"] = pymavlink

from navigation.navigation import GateMission, GateDetection, VehicleState


def test_detection_to_gate_local_rotates_translation():
    gm = GateMission()
    # set a nonzero camera yaw offset and known lever-arm
    gm.cam_yaw_offset_deg = -10.0
    gm.cam_offset_right_m = -0.25
    gm.cam_offset_down_m = -0.10

    # Vehicle at origin facing north (yaw = 0)
    state = VehicleState(n=0.0, e=0.0, d=0.0, yaw_rad=0.0)

    # A gate detected straight ahead in camera frame at forward=3.0m, right=0.0m
    det = GateDetection(timestamp=math.floor(0.0), dist=3.0, forward=3.0, right=0.0, down=0.0, roll=0.0, pitch=0.0, yaw_deg=0.0)

    gate_n, gate_e, gate_d, gate_yaw = gm.detection_to_gate_local(det, state)

    theta = math.radians(gm.cam_yaw_offset_deg)
    expected_r_b = det.forward * math.sin(theta) + det.right * math.cos(theta)
    expected_e = state.e + expected_r_b + gm.cam_offset_right_m

    assert math.isclose(gate_e, expected_e, rel_tol=1e-3, abs_tol=1e-3), f"Expected E={expected_e}, got {gate_e}"


if __name__ == '__main__':
    test_detection_to_gate_local_rotates_translation()
    print('OK')
