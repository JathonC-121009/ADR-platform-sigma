import cv2
from gstreamer_class import CameraStream

class OpenCVProcessing():
    
    def __init__(self, cam: CameraStream):
        self.cam = cam
    
    def cv_processing_loop(self):
        while self.cam.is_camera_running():
            frame = self.cam.get_latest_cv_frame()
            if frame is None:
                continue
            
            # do processing here
            # example overlay
            cv2.putText(
                frame,
                "Processed OpenCV output",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (255, 255, 255),
                2,
            )

            # Encode processed frame for Flask
            ok, buffer = cv2.imencode(".jpg", frame)
            if ok:
                self.cam.set_latest_processed_jpg(buffer.tobytes())
