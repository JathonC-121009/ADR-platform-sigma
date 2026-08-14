import math
import time

from ..navigation import GateMission, NavigationController, wrap_pi, deg_to_rad, LocalTarget

class MultiStageGateMission(GateMission):
    NUM_GATES = 4

    """Default race mission that approaches each gate in shrinking stages."""

    def run(self, nav: NavigationController):
        last_gate = None
        """Fly the 3m -> 2m -> 1m -> pass-through sequence for up to 8 gates."""
        gate_count = 0

        while nav.running and gate_count < self.NUM_GATES:
            print("\n==============================")
            print(f"[*] Looking for Gate {gate_count + 1} of {self.NUM_GATES}")
            print("==============================")

            gate = self.observe_gate(nav, duration=2.5)
            if not gate:
                gate = self.try_recover_gate(nav, last_gate, stage_standoff_m=3.0)
                if not gate:
                    print("[!] Lost gate at 3m. Restarting.")
                    continue
            last_gate = gate

            target_3m = self.build_standoff_target(nav, gate, standoff_m=3.0)
            nav.move_to_target(target_3m, "3m Standoff", max_speed_m_s= 0.15)

            gate = self.observe_gate(nav, duration=2.5)
            if not gate:
                gate = self.try_recover_gate(nav, last_gate, stage_standoff_m=3.0)
                if not gate:
                    print("[!] Lost gate at 3m. Restarting.")
                    continue
            last_gate = gate

            target_2m = self.build_standoff_target(nav, gate, standoff_m=2.0)
            nav.move_to_target(target_2m, "2m Standoff", max_speed_m_s= 0.15)

            gate = self.observe_gate(nav, duration=2.5)
            if not gate:
                gate = self.try_recover_gate(nav, last_gate, stage_standoff_m=2.0)
                if not gate:
                    print("[!] Lost gate at 2m. Restarting.")
                    continue
            last_gate = gate

            target_1m = self.build_standoff_target(nav, gate, standoff_m=1.0)
            nav.move_to_target(target_1m, "1m Standoff", max_speed_m_s= 0.15)

            gate = self.observe_gate(nav, duration=2.5)
            if not gate:
                gate = self.try_recover_gate(nav, last_gate, stage_standoff_m=1.0)
                if not gate:
                    print("[!] Lost gate right before pass. Restarting.")
                    continue
            last_gate = gate

            # Verify the hitbox fits the gate before committing to the pass.
            if not self.confirm_hitbox_fits(nav, gate):
                print("[!] Hitbox doesn't fit through the gate. Attempting local recovery.")
                recovered = self.try_recover_gate(nav, last_gate, stage_standoff_m=1.0)
                if not recovered:
                    print("[!] Could not find a safe approach. Restarting.")
                    continue
                gate = recovered

            # Use centralized helper that disables vertical commands during pass.
            self.perform_pass_through(nav, gate, pass_dist_m=1.5, max_speed_m_s=0.15)

            gate_count += 1
            print(f"[*] Successfully navigated Gate {gate_count}!")

        if gate_count >= 8:
            print("[!] Mission complete; landing disabled. Please land the vehicle manually.")


    def try_recover_gate(self, nav, last_gate, stage_standoff_m):
        if last_gate is None:
            return False

        print("[*] Trying local recovery from last known gate")
        recovered_gate = self.scan_for_gate(nav, last_gate)
        if recovered_gate:
            return recovered_gate

        print("[*] Local scan failed, backing off one stage")
        retreat_standoff = min(stage_standoff_m + 1.0, 3.0)
        recover_target = self.build_standoff_target(nav, last_gate, standoff_m=retreat_standoff)
        nav.move_to_target(recover_target, "recover to previous standoff")
        return None

    def scan_for_gate(self, nav, last_gate):
        hold_yaw = nav.get_vehicle_snapshot().yaw_rad
        for offset_deg in (0.0, 15.0, -15.0, 30.0, -30.0):
            target_yaw = wrap_pi(hold_yaw + deg_to_rad(offset_deg))
            nav.send_velocity_and_yaw_target(0.0, 0.0, 0.0, target_yaw)
            gate = self.observe_gate(nav, duration=2.5)
            if gate:
                return gate
        return None