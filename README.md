# ADR Platform

ADR Platform is an experimental camera and video-processing foundation for
autonomous drone-racing projects. It captures MJPEG video from one or more
Linux V4L2 cameras with GStreamer, makes frames available to OpenCV, and serves
raw and processed streams through Flask.

> [!WARNING]
> This is research software, not a safety-certified flight system. Test changes
> without propellers first, use an appropriate flight-test area, and keep a
> manual failsafe available.

## Features

- Multiple configurable V4L2 cameras
- Low-latency GStreamer pipelines with bounded, leaky queues
- Thread-safe access to raw and OpenCV frames
- Browser-viewable raw and processed MJPEG streams
- A helper script for recording camera output to Matroska

## Requirements

- Linux with V4L2 camera devices
- Python 3.10 or newer
- Cameras that can produce MJPEG at the configured resolution and frame rate
- GStreamer 1.x and its Python bindings

The default configuration expects cameras at `/dev/video0` and `/dev/video2`,
each producing 1280×720 MJPEG at 60 FPS. Adjust `CAMERA_CONFIGS` in `main.py`
for your hardware.

## Installation

Install the system packages on Debian, Ubuntu, or Raspberry Pi OS:

```bash
sudo apt update
sudo apt install \
  python3-venv \
  python3-gi \
  gir1.2-gstreamer-1.0 \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly
```

Clone the project and create a virtual environment that can access the
system-installed GStreamer bindings:

```bash
git clone https://github.com/ctrl-alt-delete101/ADR-platform.git
cd ADR-platform
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Usage

List the available camera devices:

```bash
ls -l /dev/video*
```

Update `CAMERA_CONFIGS` in `main.py`, then start the server:

```bash
python main.py
```

Open `http://localhost:5000` on the machine running the server. For each
configured camera, these endpoints are also available:

- `/<camera-name>/video_feed` — raw MJPEG stream
- `/<camera-name>/processed_feed` — OpenCV-processed MJPEG stream

The development server listens on all network interfaces. It has no
authentication or encryption, so only run it on a trusted network or place it
behind an authenticated reverse proxy.

To record a camera directly with GStreamer:

```bash
./record.sh 0
```

Recordings are written to `captures/`, which is intentionally ignored by Git.

## Development

Run the hardware-independent tests:

```bash
python -m unittest discover -s tests -v
```

Hardware behavior varies by camera and platform. When reporting a problem,
include your operating system, Python and GStreamer versions, camera model,
device path, and supported V4L2 formats.

## Contributing and security

Contributions are welcome; see [CONTRIBUTING.md](CONTRIBUTING.md). Please report
security issues privately as described in [SECURITY.md](SECURITY.md).

## License

This project is available under the [MIT License](LICENSE).
