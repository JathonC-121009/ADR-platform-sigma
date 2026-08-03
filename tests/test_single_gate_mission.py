import sys
import types
import unittest


fake_mavutil = types.ModuleType("pymavlink.mavutil")
fake_mavutil.mavfile = object
fake_pymavlink = types.ModuleType("pymavlink")
fake_pymavlink.mavutil = fake_mavutil
sys.modules.setdefault("pymavlink", fake_pymavlink)
sys.modules.setdefault("pymavlink.mavutil", fake_mavutil)

from navigation.missions.single_gate import (  # noqa: E402
    SingleGateMission,
    limit_target_step,
    plan_standoff_horizon,
)
from navigation.navigation import GateDetection, LocalTarget, VehicleState  # noqa: E402


def detection(distance):
    return GateDetection(
        timestamp=0.0,
        dist=distance,
        forward=distance,
        right=0.0,
        down=0.0,
        roll=0.0,
        pitch=0.0,
        yaw_deg=0.0,
    )


class PlannerTests(unittest.TestCase):
    def test_horizon_uses_small_steps_and_stops_at_commit_distance(self):
        self.assertEqual(
            plan_standoff_horizon(2.0, 1.0, 0.25, 2),
            (1.75, 1.5),
        )
        self.assertEqual(
            plan_standoff_horizon(1.1, 1.0, 0.25, 2),
            (1.0,),
        )

    def test_horizon_is_empty_at_or_inside_commit_distance(self):
        self.assertEqual(plan_standoff_horizon(1.0, 1.0, 0.25, 2), ())
        self.assertEqual(plan_standoff_horizon(0.8, 1.0, 0.25, 2), ())

    def test_approach_target_is_limited_in_three_dimensions(self):
        current = VehicleState(n=1.0, e=2.0, d=-1.0)
        desired = LocalTarget(n=4.0, e=6.0, d=-1.0, yaw_rad=0.3)

        limited = limit_target_step(current, desired, max_step_m=0.25)

        displacement = (
            (limited.n - current.n) ** 2
            + (limited.e - current.e) ** 2
            + (limited.d - current.d) ** 2
        ) ** 0.5
        self.assertAlmostEqual(displacement, 0.25)
        self.assertAlmostEqual(limited.yaw_rad, desired.yaw_rad)


class FakeNavigation:
    def __init__(self):
        self.running = True
        self.moves = []
        self.landed = False
        self.state = VehicleState()

    def move_to_target(self, target, label):
        self.moves.append((target, label))
        if isinstance(target, LocalTarget):
            self.state.n = target.n
            self.state.e = target.e
            self.state.d = target.d
            self.state.yaw_rad = target.yaw_rad
        return True

    def get_vehicle_snapshot(self):
        return self.state

    def land(self):
        self.landed = True
        self.running = False


class ScriptedSingleGateMission(SingleGateMission):
    def __init__(self, observations, **kwargs):
        super().__init__(**kwargs)
        self.observations = iter(observations)

    def observe_gate(self, nav, duration=1.0):
        return next(self.observations)

    def build_standoff_target(self, nav, det, standoff_m):
        return LocalTarget(n=standoff_m, e=0.0, d=0.0, yaw_rad=0.0)

    def build_pass_through_target(self, nav, det, pass_dist_m):
        return ("pass", pass_dist_m)


class SingleGateMissionTests(unittest.TestCase):
    def test_replans_after_each_small_move_then_passes_and_lands(self):
        mission = ScriptedSingleGateMission(
            [detection(1.5), detection(1.22), detection(1.0)],
            step_size_m=0.25,
            horizon_steps=2,
        )
        nav = FakeNavigation()

        mission.run(nav)

        self.assertEqual(
            [target for target, _ in nav.moves],
            [
                LocalTarget(n=0.25, e=0.0, d=0.0, yaw_rad=0.0),
                LocalTarget(n=0.5, e=0.0, d=0.0, yaw_rad=0.0),
                ("pass", 1.5),
            ],
        )
        self.assertTrue(nav.landed)


if __name__ == "__main__":
    unittest.main()
