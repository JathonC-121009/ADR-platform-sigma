import numpy as np
from pathlib import Path

from hailo_platform import VDevice, FormatType


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSETS_DIR = PROJECT_ROOT / "assets"

def main():
    hef_path = ASSETS_DIR / "best.hef"
    print(f"Loading {hef_path} into VDevice...")

    vdevice = VDevice()
    infer_model = vdevice.create_infer_model(str(hef_path))
    infer_model.set_batch_size(1)

    print("\n=== MODEL INPUTS ===")
    for i in infer_model.inputs:
        i.set_format_type(FormatType.UINT8)
        print(f"Name: {i.name}, Shape: {i.shape}")

    print("\n=== MODEL OUTPUTS ===")
    for o in infer_model.outputs:
        o.set_format_type(FormatType.FLOAT32)
        print(f"Name: {o.name}, Shape: {o.shape}")

    print("\nConfiguring hardware...")
    configured = infer_model.configure()
    bindings = configured.create_bindings()

    out_buffers = {}
    for o in infer_model.outputs:
        buf = np.empty(o.shape, dtype=np.float32)
        out_buffers[o.name] = buf
        bindings.output(o.name).set_buffer(buf)

    # Feed a dummy blank image
    print("Feeding dummy image...")
    img = np.zeros((1, 640, 640, 3), dtype=np.uint8)
    bindings.input(infer_model.inputs[0].name).set_buffer(img)

    print("Running inference...")
    configured.run([bindings], 1000)

    print("\n=== INFERENCE RESULTS ===")
    for name, buf in out_buffers.items():
        print(f"\nOutput Buffer: {name}")
        print(f"  Raw Shape: {buf.shape}")
        print(f"  Max value: {np.max(buf)}")
        print(f"  Min value: {np.min(buf)}")

        # Print a small slice of the data so we can see the padding
        flat = buf.flatten()
        print(f"  First 15 values: {flat[:15]}")

if __name__ == "__main__":
    main()
