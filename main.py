import signal
import sys

from camera_app import create_camera_app
from flask_streaming import FlaskCameraServer


CAMERA_CONFIGS = [
    {
        "name": "camera0",
        "device": "/dev/video0",
        "width": 1280,
        "height": 720,
        "fps": 60,
    },
    # To add another camera, duplicate this block and change name/device:
    {
        "name": "camera1",
        "device": "/dev/video2",
        "width": 1280,
        "height": 720,
        "fps": 60,
    },
]

camera_apps = [create_camera_app(config) for config in CAMERA_CONFIGS]
server = FlaskCameraServer(camera_apps)


def shutdown(sig=None, frame=None):
    for camera_app in camera_apps:
        camera_app.stop()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    for camera_app in camera_apps:
        camera_app.start()

    server.run(host="0.0.0.0", port=5000)
