# Asset provenance

The repository's MIT License covers source code unless otherwise noted. It
does not automatically grant rights to models, datasets, calibration outputs,
or other separately copyrightable assets.

## `best.hef`

- Format: compiled Hailo executable
- Target: Hailo-10H
- SHA-256:
  `1f2601ee94f7162277a20ce673212c4070bc4f230073f230b4f019736200bd0a`
- Training-data provenance: not yet documented
- Upstream checkpoint and license: not yet documented
- Compiler/toolchain version and terms: not yet documented
- Redistribution permission: not yet confirmed

Do not publish or redistribute this file until the maintainers have documented
the complete provenance and confirmed that every relevant license permits
redistribution of the compiled artifact.

## Camera calibration files

- `tevs_CAM1_fisheye_calibration.yaml`
- `tevs_CAM2_fisheye_calibration.yaml`

These files contain intrinsic calibration values for specific camera setups.
Their creator, source images, and redistribution permission must be confirmed
before public release. They should not be reused for different hardware without
performing a new calibration.
