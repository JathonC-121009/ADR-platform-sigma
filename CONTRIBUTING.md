# Contributing

Thanks for helping improve ADR Platform.

## Before opening an issue

- Search existing issues for the same symptom or proposal.
- For camera problems, verify the device path and supported modes with
  `v4l2-ctl --list-formats-ext -d /dev/videoN` when `v4l2-ctl` is available.
- Remove credentials, private network details, recorded video, and other
  sensitive data from logs and attachments.

Bug reports should include:

- Operating system and hardware
- Python, GStreamer, and OpenCV versions
- Camera model and V4L2 device path
- The configured resolution and frame rate
- Minimal reproduction steps and the complete error message

Use GitHub's private vulnerability-reporting flow instead of a public issue
for security problems; see [SECURITY.md](SECURITY.md).

## Development setup

Follow the installation instructions in [README.md](README.md), then run:

```bash
python -m unittest discover -s tests -v
python -m py_compile \
  camera_app.py \
  flask_streaming.py \
  gstreamer_class.py \
  main.py \
  opencv_processing.py
```

The unit tests do not require camera hardware. Please also test camera or
pipeline changes on relevant hardware when possible and describe that testing
in the pull request.

## Pull requests

Keep each pull request focused on one change. Explain the motivation, note any
hardware assumptions, add or update tests where practical, and update the
documentation when behavior or setup changes.

By contributing, you agree that your contribution may be distributed under
the repository's MIT License and that you have the right to submit it.
