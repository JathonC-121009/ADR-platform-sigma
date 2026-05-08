import time

from flask import Flask, Response


class FlaskCameraServer:
    def __init__(self, camera_apps):
        self.camera_apps = camera_apps
        self.app = Flask(__name__)
        self._register_routes()

    def run(self, host="0.0.0.0", port=5000):
        self.app.run(
            host=host,
            port=port,
            threaded=True,
            use_reloader=False,
        )

    def _register_routes(self):
        self.app.add_url_rule("/", endpoint="index", view_func=self.index)

        for camera_app in self.camera_apps:
            self._register_camera_routes(camera_app)

    def _register_camera_routes(self, camera_app):
        prefix = f"/{camera_app.name}"

        self.app.add_url_rule(
            f"{prefix}/video_feed",
            endpoint=f"{camera_app.name}_video_feed",
            view_func=lambda: self._mjpeg_response(
                camera_app.camera.get_latest_jpg,
                camera_app.camera.is_camera_running,
            ),
        )

        self.app.add_url_rule(
            f"{prefix}/processed_feed",
            endpoint=f"{camera_app.name}_processed_feed",
            view_func=lambda: self._mjpeg_response(
                camera_app.camera.get_latest_processed_jpg,
                camera_app.camera.is_camera_running,
            ),
        )

    def _mjpeg_response(self, frame_getter, is_running):
        def generate():
            last_sent = None

            while is_running():
                frame = frame_getter()

                if frame is not None and frame is not last_sent:
                    last_sent = frame

                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                        + frame
                        + b"\r\n"
                    )

                time.sleep(1 / 30)

        return Response(
            generate(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    def index(self):
        stream_sections = "\n".join(
            f"""
            <section>
              <h1>{camera_app.name} Raw GStreamer MJPEG Stream</h1>
              <img src="/{camera_app.name}/video_feed" style="width:45vw;">

              <h1>{camera_app.name} Processed OpenCV Stream</h1>
              <img src="/{camera_app.name}/processed_feed" style="width:45vw;">
            </section>
            """
            for camera_app in self.camera_apps
        )

        return f"""
        <html>
          <body style="background:#111;color:white;text-align:center;">
            {stream_sections}
          </body>
        </html>
        """
