import argparse
import math
from dataclasses import replace
from ..navigation import (
    GateDetection,
    GateMission,
    LocalTarget,
    MAVLINK_CONN,
    NavigationController,
    deg_to_rad,
    wrap_pi,
    local_forward_vector,
)

PASS_ALTITUDE_MARGIN_M = 0.10
GATE_REUSE_RADIUS_M = 1.5
MAX_GATES = 4
SEARCH_OFFSETS_DEG = (0.0, 15.0, 30.0, 45.0, 60.0, -15.0, -30.0) # for right
# SEARCH_OFFSETS_DEG = (0.0, -15.0, -30.0, -45.0, -60.0, 15.0, 30.0) # for left
SEARCH_DWELL_S = 1.5

class MultiStageGateLevelThreeMission(GateMission):

    def __init__(self, *args, max_gates: int = MAX_GATES, pass_altitude_margin_m: float = PASS_ALTITUDE_MARGIN_M, **kwargs):
        super().__init__(*args, **kwargs)
        self.max_gates = max_gates
        self.pass_altitude_margin_m = pass_altitude_margin_m
        self._last_passed_local = None

    def build_standoff_target(self, nav: NavigationController, det: GateDetection, standoff_m: float) -> LocalTarget:
        target = super().build_standoff_target(nav, det, standoff_m)
        return replace(target, d=target.d + self.pass_altitude_margin_m)

    def build_pass_through_target(self, nav: NavigationController, det: GateDetection, pass_dist_m: float) -> LocalTarget:
        target = super().build_pass_through_target(nav, det, pass_dist_m)
        return replace(target, d=target.d + self.pass_altitude_margin_m)

    def _is_last_passed_gate(self, det: GateDetection, nav: NavigationController) -> bool:
        if self._last_passed_local is None:
            return False

        gate_n, gate_e, gate_d, _ = self.detection_to_gate_local(det, nav.get_vehicle_snapshot())
        pn, pe, pd = self._last_passed_local
        distance = math.hypot(gate_n - pn, gate_e - pe, gate_d - pd)
        return distance < GATE_REUSE_RADIUS_M

    def scan_for_gate(self, nav: NavigationController, dwell_s: float = SEARCH_DWELL_S):
        start_yaw = nav.get_vehicle_snapshot().yaw_rad
        for offset_deg in SEARCH_OFFSETS_DEG:
            target_yaw = wrap_pi(start_yaw + deg_to_rad(offset_deg))
            nav.send_velocity_and_yaw_target(0.0, 0.0, 0.0, target_yaw)
        
            gate = self.observe_gate(nav, duration=dwell_s)
            if gate and not self._is_last_passed_gate(gate, nav):
                return gate
        return None

        
        # start_yaw = nav.get_vehicle_snapshot().yaw_rad
        # for offset_deg in SEARCH_OFFSETS_DEG:
            # target_yaw = wrap_pi(start_yaw + deg_to_rad(offset_deg))
            # nav.send_velocity_and_yaw_target(0.0, 0.0, 0.0, target_yaw)

            # gate = self.observe_gate(nav, duration=dwell_s)
            # if gate and not self._is_last_passed_gate(gate, nav):
                # return gate
        # return None


    def recover_gate(self, nav: NavigationController, last_gate, stage_standoff_m: float):
        if last_gate is None:
            return None

        retreat_standoff = min(stage_standoff_m + 1.0, 3.0)
        recover_target = self.build_standoff_target(nav, last_gate, standoff_m=retreat_standoff)
        nav.move_to_target(recover_target, "Recover to previous standoff")

        gate = self.observe_gate(nav, duration=2.0)
        if gate and not self._is_last_passed_gate(gate, nav):
            return gate
        return None

    def _acquire_next_gate(self, nav: NavigationController):
        for _ in range(3):
            gate = self.scan_for_gate(nav)
            if gate and not self._is_last_passed_gate(gate, nav):
                return gate
        return None

    def run(self, nav: NavigationController):
        gate_count = 0
        last_gate = None

        while nav.running:
            if self.max_gates and gate_count >= self.max_gates:
                print(f"[*] Completed {gate_count} gates; landing.")
                nav.land()
                return

            print("\n==============================")
            print(f"[*] Looking for Gate {gate_count + 1}")
            print("==============================")

            gate = self._acquire_next_gate(nav)
            if not gate:
                print("[!] No gate in sight. Holding position without moving."); continue
            # refine: short conservative approach so the gate is more centered for final observe
            state = nav.get_vehicle_snapshot()
            gate_n, gate_e, gate_d, gate_yaw = self.detection_to_gate_local(gate, state)
            f_n, f_e = local_forward_vector(gate_yaw)

            approach_dist = 1.5  # meters; tune 1.0–2.0 for your setup
            approach_target = LocalTarget(
                n=gate_n - approach_dist * f_n,
                e=gate_e - approach_dist * f_e,
                d=gate_d,
                yaw_rad=gate_yaw,
            )
            nav.move_to_target(approach_target, "Approach Gate", max_speed_m_s=0.1)

            # refine observation from the new pose
            gate = self.observe_gate(nav, duration=2.5)
            if not gate:
                gate = self.recover_gate(nav, last_gate, stage_standoff_m=3.0)
                if not gate:
                    print("[!] Lost gate during approach. Restarting.")
                    continue
            last_gate = gate

            target_3m = self.build_standoff_target(nav, gate, standoff_m=3.0)
            nav.move_to_target(target_3m, "3m Standoff")

            gate = self.observe_gate(nav, duration=2.5)
            if not gate:
                gate = self.recover_gate(nav, last_gate, stage_standoff_m=3.0)
            if not gate:
                print("[!] Lost gate at 3m. Restarting.")
                continue
            last_gate = gate

            target_2m = self.build_standoff_target(nav, gate, standoff_m=2.0)
            nav.move_to_target(target_2m, "2m Standoff")

            gate = self.observe_gate(nav, duration=2.5)
            if not gate:
                gate = self.recover_gate(nav, last_gate, stage_standoff_m=2.0)
            if not gate:
                print("[!] Lost gate at 2m. Restarting.")
                continue
            last_gate = gate

            target_1m = self.build_standoff_target(nav, gate, standoff_m=1.0)
            nav.move_to_target(target_1m, "1m Standoff")

            gate = self.observe_gate(nav, duration=2.5)
            if not gate:
                gate = self.recover_gate(nav, last_gate, stage_standoff_m=1.0)
            if not gate:
                print("[!] Lost gate right before pass. Restarting.")
                continue

            gate_n, gate_e, gate_d, _ = self.detection_to_gate_local(gate, nav.get_vehicle_snapshot())
            self._last_passed_local = (gate_n, gate_e, gate_d)

            # Verify hitbox fits before committing to pass
            if not self.confirm_hitbox_fits(nav, gate):
                print("[!] Hitbox doesn't fit through the gate. Attempting recovery.")
                gate = self.recover_gate(nav, last_gate, stage_standoff_m=1.0)
                if not gate:
                    print("[!] Could not find a safe approach. Restarting.")
                    continue

            pass_target = self.build_pass_through_target(nav, gate, pass_dist_m=1.5)
            nav.move_to_target(pass_target, "Through The Gate!")

            gate_count += 1
            print(f"[*] Successfully navigated Gate {gate_count}!")


def parse_args():
    parser = argparse.ArgumentParser(description="Run the level-3 box gate mission")
    parser.add_argument(
        "--max-gates",
        type=int,
        default=MAX_GATES,
        help=f"stop and land after this many gates (default: {MAX_GATES})",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    nav = NavigationController(MAVLINK_CONN)
    mission = MultiStageGateLevelThreeMission(max_gates=args.max_gates)
    try:
        print("[*] Starting Level-3 Box Gate Mission")
        nav.run_mission(mission)
    except KeyboardInterrupt:
        print("\n[*] Shutdown requested by user.")
        nav.stop()