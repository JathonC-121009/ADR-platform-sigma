import argparse
from navigation import MAVLINK_CONN, NavigationController, SingleGateMission


def parse_args():
    parser = argparse.ArgumentParser(description="Run the single-gate mission")
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
    mission = SingleGateMission()
    try:
        print("[*] Starting Single-Gate Receding-Horizon Mission")
        nav.run_mission(mission)
    except KeyboardInterrupt:
        print("\n[*] Shutdown requested by user.")
        nav.stop()
