import sys

from navigation import MAVLINK_CONN, MultiStageGateMission, NavigationController


if __name__ == "__main__":
    nav = NavigationController(MAVLINK_CONN)
    mission = MultiStageGateMission()
    try:
        print("[*] Starting Multi-Stage Gate Mission")
        nav.run_mission(mission)
    except KeyboardInterrupt:
        print("\n[*] Shutdown requested by user.")
        nav.stop()
