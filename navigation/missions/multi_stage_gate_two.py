from ..navigation import GateMission, NavigationController, wrap_pi, deg_to_rad, LocalTarget
import math
import time


class MultiStageGateMissionLevelTwo(GateMission):

    """Default race mission that approaches each gate in shrinking stages."""

    NUM_GATES = 4

    def run(self, nav: NavigationController):
        last_gate = None
        """Fly the 3m -> 2m -> 1m -> pass-through sequence for up to 8 gates."""
        gate_count = 0

        expect_gate_direction = True  # true means next gate is on right, false means next
                                      # gate is on left. change the starting boolean value
                                      # depending on how the course is

        while nav.running and gate_count < self.NUM_GATES:

            print("\n==============================")
            print(f"[*] Looking for Gate {gate_count + 1} of {self.NUM_GATES}")
            print("==============================")

            gate = self.scan_for_gate(nav, None, expect_gate_direction)
            if not gate:
                gate = self.try_recover_gate(nav, None, stage_standoff_m=3.0)
                if not gate:
                    print("[!] Lost gate at 3m. Restarting.")
                    continue
            last_gate = gate

            next_gate_target = self.build_standoff_target(nav, gate, standoff_m=3.0)
            nav.move_to_target(next_gate_target, "Move to next gate", max_speed_m_s= 0.1)

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

            # Verify hitbox fits before committing to pass
            if not self.confirm_hitbox_fits(nav, gate):
                print("[!] Hitbox doesn't fit through the gate. Attempting local recovery.")
                recovered = self.try_recover_gate(nav, last_gate, stage_standoff_m=1.0)
                if not recovered:
                    print("[!] Could not find a safe approach. Restarting.")
                    continue
                gate = recovered

            # Run a short, conservative visual-servo to recentre on the gate before passing.
            # This is safety-first: small corrections, bounded speed, aborts if detection lost.
            try:
                self.visual_servo_approach(
                    nav,
                    gate,
                    max_correction_m=0.20,
                    kp=0.8,
                    duration_s=1.5,
                    sample_interval=0.08,
                    max_speed_override=0.12,
                )
            except Exception as exc:
                print(f"[!] visual_servo_approach raised: {exc} (continuing to pass)")

            # Perform a pass-through using the centralized helper which disables
            # vertical commands during the pass for all missions.
            self.perform_pass_through(nav, gate, pass_dist_m=1.5, max_speed_m_s=0.15)

            gate_count += 1
            print(f"[*] Successfully navigated Gate {gate_count}!")
            expect_gate_direction = not expect_gate_direction

            '''
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

            self.perform_pass_through(nav, gate, pass_dist_m=1.5, max_speed_m_s=0.15)

            gate_count += 1
            print(f"[*] Successfully navigated Gate {gate_count}!")
            expect_gate_direction = not expect_gate_direction

            '''

        if gate_count >= self.NUM_GATES:
            print("[!] Mission complete; landing disabled. Please land the vehicle manually.")


    def try_recover_gate(self, nav, last_gate, stage_standoff_m):
        if last_gate is None:
            return False

        print("[*] Trying local recovery from last known gate")
        recovered_gate = self.scan_for_gate(nav, last_gate, None)
        if recovered_gate:
            return recovered_gate

        print("[*] Local scan failed, backing off one stage")
        retreat_standoff = min(stage_standoff_m + 1.0, 3.0)
        recover_target = self.build_standoff_target(nav, last_gate, standoff_m=retreat_standoff)
        nav.move_to_target(recover_target, "recover to previous standoff")
        return None

    def scan_for_gate(self, nav, last_gate, expected_side: bool):  # to be changed for level 2
        # Compare against True/False explicitly: try_recover_gate passes None to
        # mean "no side preference", and a plain `not expected_side` would send
        # that down the left-hand sweep instead of the forward one below.
        if expected_side is True:   # next gate is on the right
            hold_yaw = nav.get_vehicle_snapshot().yaw_rad
            for offset_deg in (0.0, 15.0, 30.0, 45.0, -15.0):
                target_yaw = wrap_pi(hold_yaw + deg_to_rad(offset_deg))
                nav.send_velocity_and_yaw_target(0.0, 0.0, 0.0, target_yaw)
                gate = self.observe_gate(nav, duration=1.5)
                if gate:
                    return gate
            return None
        elif expected_side is False:   # next gate is on the left
            hold_yaw = nav.get_vehicle_snapshot().yaw_rad
            for offset_deg in (0.0, -15.0, -30.0, -45.0, 15.0):
                target_yaw = wrap_pi(hold_yaw + deg_to_rad(offset_deg))
                nav.send_velocity_and_yaw_target(0.0, 0.0, 0.0, target_yaw)
                gate = self.observe_gate(nav, duration=1.5)
                if gate:
                    return gate
            return None
        else:  # just normal observe forward
            hold_yaw = nav.get_vehicle_snapshot().yaw_rad
            for offset_deg in (0.0, 15.0, -15.0, 30.0, -30.0):
                target_yaw = wrap_pi(hold_yaw + deg_to_rad(offset_deg))
                nav.send_velocity_and_yaw_target(0.0, 0.0, 0.0, target_yaw)
                gate = self.observe_gate(nav, duration=2.5)
                if gate:
                    return gate
            return None

        # hold_yaw = nav.get_vehicle_snapshot().yaw_rad
        # for offset_deg in (0.0, 15.0, -15.0, 30.0, -30.0):
            # target_yaw = wrap_pi(hold_yaw + deg_to_rad(offset_deg))
            # nav.send_velocity_and_yaw_target(0.0, 0.0, 0.0, target_yaw)
            # gate = self.observe_gate(nav, duration=2.5)
            # if gate:
                # return gate
        # return None