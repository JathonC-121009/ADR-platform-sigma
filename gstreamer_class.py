#!/usr/bin/env python3
import sys
import threading
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst
import numpy as np

class CameraStream:
    def __init__(self, width=1280, height=720, fps=60, device="/dev/video0"):
        self.width = width
        self.height = height
        self.fps = fps
        self.device = device
        
        self.is_running = False

        self.latest_jpg = None # latest from gstreamer flask uses this as well
        self.latest_cv_frame = None # the frame that gets converted into bgr for opencv to do processing
        self.latest_processed_jpg = None # opencv takes the bgr frame does processing and spits out jpg for flask 
                                    # note this is separate form the gstreamer flask thing        
        
        self.jpg_lock = threading.Lock()
        self.cv_lock = threading.Lock()
        self.processed_jpg_lock = threading.Lock()
        
        Gst.init(None)
        
        PIPELINE = f"""
        v4l2src device={self.device}
        ! image/jpeg,width={self.width},height={self.height},framerate={self.fps}/1
        ! queue max-size-buffers=1 leaky=downstream
        ! jpegparse
        ! tee name=t

        t.
        ! queue max-size-buffers=1 leaky=downstream
        ! appsink name=jpeg_sink emit-signals=true sync=false max-buffers=1 drop=true

        t.
        ! queue max-size-buffers=1 leaky=downstream
        ! jpegdec
        ! videoconvert
        ! video/x-raw,format=BGR,width={self.width},height={self.height}
        ! appsink name=cv_sink emit-signals=true sync=false max-buffers=1 drop=true
        """
        
        self.pipeline = Gst.parse_launch(PIPELINE)
        self.jpeg_sink = self.pipeline.get_by_name("jpeg_sink")
        self.cv_sink = self.pipeline.get_by_name("cv_sink")
        
        self.jpeg_sink.connect("new-sample", self.on_jpeg_sample)
        self.cv_sink.connect("new-sample", self.on_cv_sample)
        
    # ---------- thread-safe accessors ----------
 
    def set_latest_jpg(self, jpg_bytes):
        with self.jpg_lock:
            self.latest_jpg = jpg_bytes

    def get_latest_jpg(self):
        with self.jpg_lock:
            return self.latest_jpg

    def set_latest_cv_frame(self, frame):
        with self.cv_lock:
            self.latest_cv_frame = frame

    def get_latest_cv_frame(self):
        with self.cv_lock:
            if self.latest_cv_frame is None:
                return None
            return self.latest_cv_frame.copy()

    def set_latest_processed_jpg(self, jpg_bytes):
        with self.processed_jpg_lock:
            self.latest_processed_jpg = jpg_bytes

    def get_latest_processed_jpg(self):
        with self.processed_jpg_lock:
            return self.latest_processed_jpg
        
    def is_camera_running(self):
        return self.is_running
        
    # ---------- gstreamer callbacks ----------

    def on_jpeg_sample(self, sink):
        
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        ok, map_info = buf.map(Gst.MapFlags.READ)

        if not ok:
            return Gst.FlowReturn.ERROR

        try:
            jpg_bytes = bytes(map_info.data)

            with self.jpg_lock:
                self.latest_jpg = jpg_bytes

        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK


    def on_cv_sample(self, sink):

        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        ok, map_info = buf.map(Gst.MapFlags.READ)

        if not ok:
            return Gst.FlowReturn.ERROR

        try:
            frame = np.frombuffer(map_info.data, dtype=np.uint8)

            # BGR frame from GStreamer
            frame = frame.reshape((self.height, self.width, 3)).copy()

            with self.cv_lock:
                self.latest_cv_frame = frame

        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

    # ---------- lifecycle ----------

    def start(self):
        self.is_running = True
        ret = self.pipeline.set_state(Gst.State.PLAYING)

        if ret == Gst.StateChangeReturn.FAILURE:
            print("Failed to start GStreamer pipeline", file=sys.stderr)
            self.stop()

    def stop(self, sig=None, frame=None):
        self.is_running = False
        self.pipeline.set_state(Gst.State.NULL)
