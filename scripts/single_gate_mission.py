from navigation import MAVLINK_CONN, NavigationController, SingleGateMission


if __name__ == "__main__":
    nav = NavigationController(MAVLINK_CONN)
    mission = SingleGateMission()
    try:
        print("[*] Starting Single-Gate Receding-Horizon Mission")
        nav.run_mission(mission)
    except KeyboardInterrupt:
        print("\n[*] Shutdown requested by user.")
        nav.stop()
