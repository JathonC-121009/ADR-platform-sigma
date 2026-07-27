# ADR Platform

ADR Platform is experimental edge software for autonomous drone racing. It
combines Hailo-accelerated gate detection, fisheye-camera pose estimation,
browser-viewable video, UDP telemetry, and MAVLink mission control on a
Raspberry Pi 5.

> [!CAUTION]
> This is research software, not a safety-certified flight system. The mission
> code can send movement and landing commands to an aircraft. Develop against
> simulation first, test without propellers, keep a manual failsafe available,
> and follow local aviation and radio-control rules.

## System overview

```text
V4L2 camera or video
        |
        v
GStreamer + OpenCV --> Hailo-10H YOLO --> gate pose + web stream
                                             |
                                      UDP 127.0.0.1:5050
                                             |
                                             v
                                  navigation mission controller
                                             |
                                  MAVLink local-NED commands
```

The vision pipeline detects up to three gates, refines their image corners,
estimates 3D pose with `solvePnP`, and publishes rows in this format:

```text
[distance, forward, right, down, roll, pitch, yaw]
```

Positions are in metres, angles are in degrees, and `999.0` marks a missing
detection.

## Repository layout

- `assets/` — model documentation and camera calibrations
- `vision/` — capture, inference, pose estimation, telemetry, and Flask streams
- `navigation/` — current modular MAVLink controller and mission framework
- `scripts/` — supported entry points and recording helper
- `debug/` — Hailo and UDP telemetry diagnostics
- Top-level `navigation*.py` files — legacy prototypes retained for reference

## Hardware and software

The current configuration targets:

- Raspberry Pi 5 running 64-bit Raspberry Pi OS
- Raspberry Pi AI HAT+ 2 with a Hailo-10H accelerator
- A V4L2 camera capable of 1280×720 MJPEG at 60 FPS
- A MAVLink-compatible autopilot or simulator
- Python 3.10 or newer

## Installation

Install the Hailo-10H runtime using the current
[Raspberry Pi AI software instructions](https://www.raspberrypi.com/documentation/computers/ai.html):

```bash
sudo apt update
sudo apt install dkms hailo-h10-all
sudo reboot
```

After rebooting, verify the accelerator:

```bash
hailortcli fw-control identify
```

Install the remaining system dependencies:

```bash
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
system-installed Hailo and GStreamer Python bindings:

```bash
git clone https://github.com/ctrl-alt-delete101/ADR-platform.git
cd ADR-platform
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Configuration

Review these values before running anything:

1. In `scripts/run_vision.py`, set the camera device, resolution, and frame
   rate in `CAMERA_CONFIGS`.
2. Supply a Hailo-10H model at `assets/best.hef`. This local model is ignored
   by Git and is not distributed with the repository.
3. In `vision/opencv_processing.py`, set `USE_VIDEO_FILE`,
   `VIDEO_FILE_PATH`, and `FLIP_CAMERA`. Live-camera mode is the default.
4. Confirm that the selected calibration YAML matches the physical camera.
5. In `navigation/navigation.py`, verify `MAVLINK_CONN`, camera offsets,
   controller gains, speed limits, tolerances, and timeouts for your vehicle.

The included calibration files are hardware-specific and should not be assumed
to fit another camera, lens, resolution, or mounting arrangement.

## Running the vision pipeline

Start vision and telemetry:

```bash
python -m scripts.run_vision
```

Open `http://localhost:5000` on the Raspberry Pi. The Flask development server
listens on all interfaces and has no authentication or TLS. Do not expose it
directly to the public internet.

To inspect telemetry without connecting an autopilot:

```bash
python -m debug.telemetry_reader
```

To record camera input:

```bash
./scripts/record.sh 0
```

Recordings are written to `captures/`, which is ignored by Git.

## Running a mission

Only proceed after validating the vision output, coordinate frames, calibration,
offsets, and MAVLink connection in a simulator:

```bash
python -m scripts.multi_stage_gate_mission
```

The mission approaches each detected gate at staged distances and then commands
a pass-through target. Treat the default controller values as development
examples, not safe settings for an arbitrary vehicle.

## Model and calibration provenance

The compiled model is intentionally excluded from the repository. Users must
supply a model they are licensed to use. The calibration files need separate
provenance and redistribution review; see
[assets/README.md](assets/README.md). Assets are not automatically covered by
the source-code license.

## Development

Run the hardware-independent checks:

```bash
python -m unittest discover -s tests -v
python -m compileall -q debug navigation scripts vision
bash -n scripts/record.sh
```

Contributions are welcome; see [CONTRIBUTING.md](CONTRIBUTING.md). Report
security issues privately as described in [SECURITY.md](SECURITY.md).

## License

Unless otherwise noted, the source code is available under the
[MIT License](LICENSE). Third-party dependencies and repository assets retain
their own licensing terms.
