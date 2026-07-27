import signal
import sys

from vision import FlaskCameraServer, create_camera_app


CAMERA_CONFIGS = [
    {
        "name": "camera0",
        "device": "/dev/video0", # Change to /dev/video2 if your camera mounts there
        "width": 1280,
        "height": 720,
        "fps": 60,
    },
    # COMMENT THIS OUT FOR NOW
    # {
    #     "name": "camera1",
    #     "device": "/dev/video2",
    #     "width": 1280,
    #     "height": 720,
    #     "fps": 60,
    # },
]


def main():
    camera_apps = [create_camera_app(config) for config in CAMERA_CONFIGS]
    server = FlaskCameraServer(camera_apps)

    def shutdown(sig=None, frame=None):
        for camera_app in camera_apps:
            camera_app.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    for camera_app in camera_apps:
        camera_app.start()

    server.run(host="0.0.0.0", port=5000)


if __name__ == "__main__":
    main()
