# Contributing

Thanks for helping improve ADR Platform.

## Before opening an issue

- Search existing issues for the same symptom or proposal.
- Reproduce flight-control problems in simulation whenever possible.
- Remove credentials, private network details, recorded video, vehicle
  identifiers, and other sensitive data from logs and attachments.
- Use GitHub's private vulnerability-reporting flow instead of a public issue
  for security problems; see [SECURITY.md](SECURITY.md).

Bug reports should include the operating system, Python and HailoRT versions,
camera and accelerator hardware, calibration used, MAVLink simulator or
autopilot, relevant configuration, reproduction steps, and complete errors.

## Development setup

Follow [README.md](README.md), then run:

```bash
python -m unittest discover -s tests -v
python -m compileall -q debug navigation scripts vision
bash -n scripts/record.sh
```

The unit tests do not require an accelerator, camera, or autopilot. Changes to
vision or navigation behavior should also be tested on appropriate hardware or
in simulation, and that validation should be described in the pull request.

## Pull requests

Keep each pull request focused on one change. Explain the motivation and safety
impact, document hardware assumptions, add or update tests where practical, and
update the documentation when behavior or setup changes.

Do not add recordings, datasets, model weights, compiled models, or calibration
data unless the pull request documents:

- Who created the asset and who owns it
- The source dataset and upstream model or checkpoint
- The license and redistribution terms for every input and output
- The target hardware and toolchain used to produce compiled artifacts
- A checksum for the exact file

By contributing, you agree that your source-code contribution may be
distributed under the repository's MIT License and that you have the right to
submit it. Assets are accepted only under separately documented terms.
