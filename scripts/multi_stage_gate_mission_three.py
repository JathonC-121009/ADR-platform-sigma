import argparse

from navigation import (
    CAM_OFFSET_DOWN_M,
    CAM_OFFSET_RIGHT_M,
    CAM_YAW_OFFSET_DEG,
    MAVLINK_CONN,
    NavigationController,
)
from navigation.missions import MultiStageGateLevelThreeMission


def parse_args():
    parser = argparse.ArgumentParser(description="Run the multi-stage gate mission")
    parser.add_argument(
        "--camera-right-offset-m",
        type=float,
        default=CAM_OFFSET_RIGHT_M,
        help=f"camera right offset in meters (default: {CAM_OFFSET_RIGHT_M})",
    )
    parser.add_argument(
        "--camera-down-offset-m",
        type=float,
        default=CAM_OFFSET_DOWN_M,
        help=f"camera down offset in meters (default: {CAM_OFFSET_DOWN_M})",
    )
    parser.add_argument(
        "--camera-yaw-offset-deg",
        type=float,
        default=CAM_YAW_OFFSET_DEG,
        help=f"camera yaw offset in degrees (default: {CAM_YAW_OFFSET_DEG})",
    )
    parser.add_argument(
        "--hitbox-size-inches",
        type=float,
        default=9.0,
        help="hitbox cube side length in inches (default: 9)",
    )
    parser.add_argument(
        "--hitbox-margin-inches",
        type=float,
        default=1.0,
        help="safety margin per side in inches (default: 1)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    import navigation as _nav
    size_m = args.hitbox_size_inches * 0.0254
    margin_m = args.hitbox_margin_inches * 0.0254
    _nav.DRONE_HITBOX_M = (size_m, size_m, size_m)
    _nav.HITBOX_MARGIN_M = margin_m

    nav = NavigationController(MAVLINK_CONN)
    mission = MultiStageGateLevelThreeMission(
        cam_offset_right_m=args.camera_right_offset_m,
        cam_offset_down_m=args.camera_down_offset_m,
        cam_yaw_offset_deg=args.camera_yaw_offset_deg,
    )
    try:
        print("[*] Starting Level 3 Multi-Stage Gate Mission")
        nav.run_mission(mission)
    except KeyboardInterrupt:
        print("\n[*] Shutdown requested by user.")
        nav.stop()
