#!/usr/bin/env bash
set -e

if [ $# -ne 1 ]; then
  echo "Usage: $0 <video_device_number>"
  echo "Example: $0 0   # records from /dev/video0"
  echo "Example: $0 2   # records from /dev/video2"
  exit 1
fi

DEV="$1"
OUT="captures/capture_dev${DEV}_$(date +%Y%m%d_%H%M%S).mkv"

mkdir -p captures

gst-launch-1.0 -e \
  v4l2src device="/dev/video${DEV}" ! \
  "image/jpeg,width=1280,height=720,framerate=60/1" ! \
  queue max-size-buffers=1 leaky=downstream ! \
  jpegparse ! \
  matroskamux ! \
  filesink location="$OUT"
