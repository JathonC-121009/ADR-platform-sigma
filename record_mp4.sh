#!/usr/bin/env bash

OUT="capture_$(date +%Y%m%d_%H%M%S).mp4"

gst-launch-1.0 -e \
  v4l2src device=/dev/video0 ! \
  "image/jpeg,width=1280,height=720,framerate=60/1" ! \
  queue max-size-buffers=1 leaky=downstream ! \
  jpegparse ! \
  jpegdec ! \
  videoconvert ! \
  x264enc tune=zerolatency speed-preset=veryfast bitrate=8000 key-int-max=60 ! \
  h264parse ! \
  mp4mux ! \
  filesink location="$OUT"
