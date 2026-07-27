# Asset provenance

The repository's MIT License covers source code unless otherwise noted. It
does not automatically grant rights to models, datasets, calibration outputs,
or other separately copyrightable assets.

## Locally supplied `best.hef`

The application expects a Hailo-10H executable at `assets/best.hef`, but that
file is intentionally excluded by `.gitignore` and is not distributed with the
repository.

Users must supply their own model and confirm that its training data, upstream
checkpoint, compiler toolchain, and resulting artifact permit their intended
use. Do not commit or redistribute the model through this repository.

## Camera calibration files

- `tevs_CAM1_fisheye_calibration.yaml`
- `tevs_CAM2_fisheye_calibration.yaml`

These files contain intrinsic calibration values for specific camera setups.
Their creator, source images, and redistribution permission must be confirmed
before public release. They should not be reused for different hardware without
performing a new calibration.
