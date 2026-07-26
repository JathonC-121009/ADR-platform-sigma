import importlib.util
import unittest
from dataclasses import dataclass
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "vision" / "flask_streaming.py"
)
SPEC = importlib.util.spec_from_file_location(
    "flask_streaming_under_test",
    MODULE_PATH,
)
flask_streaming = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(flask_streaming)
FlaskCameraServer = flask_streaming.FlaskCameraServer


class DummyCamera:
    def get_latest_jpg(self):
        return b"raw-jpeg"

    def get_latest_processed_jpg(self):
        return b"processed-jpeg"

    def is_camera_running(self):
        return False


@dataclass
class DummyCameraApp:
    name: str
    camera: DummyCamera


class FlaskCameraServerTests(unittest.TestCase):
    def setUp(self):
        self.server = FlaskCameraServer(
            [
                DummyCameraApp("camera0", DummyCamera()),
                DummyCameraApp("camera1", DummyCamera()),
            ]
        )
        self.client = self.server.app.test_client()

    def test_index_lists_every_camera_stream(self):
        response = self.client.get("/")

        self.assertEqual(response.status_code, 200)
        self.assertIn(b"/camera0/video_feed", response.data)
        self.assertIn(b"/camera0/processed_feed", response.data)
        self.assertIn(b"/camera1/video_feed", response.data)
        self.assertIn(b"/camera1/processed_feed", response.data)

    def test_camera_routes_are_registered(self):
        rules = {rule.rule for rule in self.server.app.url_map.iter_rules()}

        self.assertIn("/camera0/video_feed", rules)
        self.assertIn("/camera0/processed_feed", rules)
        self.assertIn("/camera1/video_feed", rules)
        self.assertIn("/camera1/processed_feed", rules)

    def test_mjpeg_response_contains_frame_headers_and_bytes(self):
        states = iter((True, False))
        response = self.server._mjpeg_response(
            lambda: b"jpeg-bytes",
            lambda: next(states),
        )

        body = b"".join(response.response)

        self.assertIn(b"Content-Type: image/jpeg", body)
        self.assertIn(b"Content-Length: 10", body)
        self.assertIn(b"jpeg-bytes", body)


if __name__ == "__main__":
    unittest.main()
