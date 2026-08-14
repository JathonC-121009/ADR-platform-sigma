"""Receding-horizon mission for approaching and crossing one gate."""

import math
from typing import Tuple

from ..navigation import (
    GateDetection,
    GateMission,
    LocalTarget,
    NavigationController,
    VehicleState,
)


def plan_standoff_horizon(
    distance_m: float,
    commit_distance_m: float,
    step_size_m: float,
    horizon_steps: int,
) -> Tuple[float, ...]:
    """Return the next decreasing standoff distances in a short horizon.

    Each planned move is no larger than ``step_size_m`` and the horizon never
    asks the vehicle to move closer than ``commit_distance_m``.  The mission
    executes only the first target before measuring the gate again.
    """
    if commit_distance_m < 0.0:
        raise ValueError("commit_distance_m must be non-negative")
    if step_size_m <= 0.0:
        raise ValueError("step_size_m must be positive")
    if horizon_steps <= 0:
        raise ValueError("horizon_steps must be positive")

    if distance_m <= commit_distance_m:
        return ()

    standoffs = []
    predicted_distance = distance_m
    for _ in range(horizon_steps):
        predicted_distance = max(commit_distance_m, predicted_distance - step_size_m)
        standoffs.append(predicted_distance)
        if predicted_distance <= commit_distance_m:
            break

    return tuple(standoffs)


def limit_target_step(
    current: VehicleState,
    target: LocalTarget,
    max_step_m: float,
) -> LocalTarget:
    """Limit an approach target to a maximum 3D displacement."""
    if max_step_m <= 0.0:
        raise ValueError("max_step_m must be positive")

    delta_n = target.n - current.n
    delta_e = target.e - current.e
    delta_d = target.d - current.d
    distance = math.sqrt(delta_n**2 + delta_e**2 + delta_d**2)

    if distance <= max_step_m:
        return target

    scale = max_step_m / distance
    return LocalTarget(
        n=current.n + delta_n * scale,
        e=current.e + delta_e * scale,
        d=current.d + delta_d * scale,
        yaw_rad=target.yaw_rad,
    )


class SingleGateMission(GateMission):
    """Approach one gate with small replanned moves, cross it, and land.

    This is an MPC-style receding-horizon controller rather than a numerical
    optimizer: it predicts a few safe standoff targets, applies the first one,
    and uses a new vision observation to rebuild the horizon.
    """

    def __init__(
        self,
        *args,
        step_size_m: float = 0.25,
        horizon_steps: int = 2,
        commit_distance_m: float = 1.0,
        pass_distance_m: float = 1.5,
        observation_duration_s: float = 0.5,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)

        # Validate all planner settings at construction time.
        plan_standoff_horizon(
            commit_distance_m + step_size_m,
            commit_distance_m,
            step_size_m,
            horizon_steps,
        )
        if pass_distance_m <= 0.0:
            raise ValueError("pass_distance_m must be positive")
        if observation_duration_s <= 0.0:
            raise ValueError("observation_duration_s must be positive")

        self.step_size_m = step_size_m
        self.horizon_steps = horizon_steps
        self.commit_distance_m = commit_distance_m
        self.pass_distance_m = pass_distance_m
        self.observation_duration_s = observation_duration_s

    def _build_horizon(self, gate: GateDetection) -> Tuple[float, ...]:
        return plan_standoff_horizon(
            distance_m=gate.dist,
            commit_distance_m=self.commit_distance_m,
            step_size_m=self.step_size_m,
            horizon_steps=self.horizon_steps,
        )

    def run(self, nav: NavigationController):
        """Replan until 1 m away, then cross using the last visible gate pose."""
        print("[*] Starting single-gate receding-horizon approach")

        while nav.running:
            gate = self.observe_gate(nav, duration=self.observation_duration_s)
            if gate is None:
                print("[!] Gate unavailable. Holding position and trying again.")
                continue

            horizon = self._build_horizon(gate)
            if horizon:
                horizon_text = ", ".join(f"{distance:.2f}m" for distance in horizon)
                print(
                    f"[*] Gate distance {gate.dist:.2f}m; "
                    f"planned standoffs: {horizon_text}"
                )

                # Receding-horizon behavior: execute only the first prediction,
                # then observe the gate and solve the short plan again.
                next_standoff = horizon[0]
                desired_target = self.build_standoff_target(
                    nav,
                    gate,
                    standoff_m=next_standoff,
                )
                target = limit_target_step(
                    current=nav.get_vehicle_snapshot(),
                    target=desired_target,
                    max_step_m=self.step_size_m,
                )
                nav.move_to_target(target, f"MPC standoff {next_standoff:.2f}m")
                continue

            # Do not depend on vision after committing to the crossing. The
            # current detection is the final gate pose used for pass-through.
            print(
                f"[*] Within {self.commit_distance_m:.2f}m; "
                "committing to gate pass"
            )
            # Confirm hitbox fit before committing to the crossing
            if not self.confirm_hitbox_fits(nav, gate):
                print("[!] Hitbox doesn't fit through the gate. Holding and retrying.")
                continue

            crossed = self.perform_pass_through(nav, gate, pass_dist_m=self.pass_distance_m, max_speed_m_s=0.15, label="through the gate")
            if not crossed:
                print("[!] Pass-through move timed out; landing at current position.")

            if nav.running:
                print("[!] Mission complete; landing disabled. Please land the vehicle manually.")
            return
