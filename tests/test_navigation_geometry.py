import math
import sys
import types
import unittest


fake_mavutil = types.ModuleType("pymavlink.mavutil")
fake_mavutil.mavfile = object
fake_pymavlink = types.ModuleType("pymavlink")
fake_pymavlink.mavutil = fake_mavutil
sys.modules.setdefault("pymavlink", fake_pymavlink)
sys.modules.setdefault("pymavlink.mavutil", fake_mavutil)

from navigation.navigation import (  # noqa: E402
    GateDetection,
    NO_DETECTION_DIST,
    body_to_local,
    deg_to_rad,
    is_valid_detection,
    rad_to_deg,
    wrap_pi,
)


class NavigationGeometryTests(unittest.TestCase):
    def test_degree_and_radian_conversion_round_trip(self):
        self.assertAlmostEqual(rad_to_deg(deg_to_rad(123.4)), 123.4)

    def test_wrap_pi_normalizes_angles(self):
        self.assertAlmostEqual(wrap_pi(3 * math.pi), -math.pi)
        self.assertAlmostEqual(wrap_pi(-3 * math.pi), -math.pi)
        self.assertAlmostEqual(wrap_pi(math.pi / 2), math.pi / 2)

    def test_body_to_local_rotates_horizontal_axes(self):
        north, east, down = body_to_local(2.0, 1.0, -0.5, math.pi / 2)

        self.assertAlmostEqual(north, -1.0)
        self.assertAlmostEqual(east, 2.0)
        self.assertAlmostEqual(down, -0.5)

    def test_missing_detection_sentinel_is_invalid(self):
        missing = GateDetection(
            timestamp=0.0,
            dist=NO_DETECTION_DIST,
            forward=0.0,
            right=0.0,
            down=0.0,
            roll=0.0,
            pitch=0.0,
            yaw_deg=0.0,
        )
        detected = GateDetection(
            timestamp=0.0,
            dist=3.0,
            forward=3.0,
            right=0.0,
            down=0.0,
            roll=0.0,
            pitch=0.0,
            yaw_deg=0.0,
        )

        self.assertFalse(is_valid_detection(None))
        self.assertFalse(is_valid_detection(missing))
        self.assertTrue(is_valid_detection(detected))


if __name__ == "__main__":
    unittest.main()
