import threading
from dataclasses import dataclass

from .gstreamer_class import CameraStream
from .opencv_processing import OpenCVProcessing


@dataclass
class CameraApp:
    name: str
    camera: CameraStream
    processor: OpenCVProcessing
    processing_thread: threading.Thread | None = None

    def start(self):
        self.camera.start()
        self.processing_thread = threading.Thread(
            target=self.processor.cv_processing_loop,
            daemon=True,
            name=f"{self.name}-opencv",
        )
        self.processing_thread.start()

    def stop(self):
        self.camera.stop()


def create_camera_app(config):
    camera = CameraStream(
        device=config["device"],
        width=config["width"],
        height=config["height"],
        fps=config["fps"],
    )
    return CameraApp(
        name=config["name"],
        camera=camera,
        processor=OpenCVProcessing(camera),
    )
