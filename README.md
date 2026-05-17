
***

# 🚀 Autonomous Drone Racing Gate Detection (Edge Deployment)

This repository contains the edge-deployment software for a real-time, 60 FPS 3D gate detection pipeline designed for autonomous racing drones. 

It utilizes a custom YOLOv8 model quantized for the **Raspberry Pi AI HAT+ 2 (Hailo-10H)**, followed by highly optimized OpenCV sub-pixel math and `solvePnP` 3D kinematics to output a live telemetry matrix (Distance, X, Y, Z, Roll, Pitch, Yaw) for the drone's flight controller.

## 📦 Included Files
Ensure your project directory contains the following folders before proceeding:
* `assets/` *(Compiled model and calibration YAMLs)*
* `vision/` *(Camera, OpenCV, and Flask streaming modules)*
* `navigation/` *(Navigation controller and mission classes)*
* `scripts/` *(Runnable entry points and helper scripts)*
* `debug/` *(One-off diagnostics and telemetry readers)*
* `requirements.txt` *(Python dependencies)*

---

## 🛠️ Step 1: Install AI Hardware Drivers
This system assumes you are running a **Raspberry Pi 5** with the **Raspberry Pi AI HAT+ 2 (Hailo-10H chip)**. 

Open a terminal on your Pi (or SSH into it) and run the following commands to install the PCIe drivers and HailoRT software:

```bash
sudo apt update
sudo apt install raspberrypi-kernel-headers -y
sudo apt install hailo-h10-all h10-hailort-pcie-driver python3-h10-hailort -y
```

Add your user to the Hailo hardware group so Python can access the AI chip without `sudo`, then reboot the Pi:
```bash
sudo usermod -aG hailo $USER
sudo reboot
```

---

## 📂 Step 2: Transfer Files to the Pi
If the files are currently on your laptop, you need to copy them to the Raspberry Pi.
Open **Windows PowerShell** (or your Mac/Linux terminal) in the folder containing your files and run:

```powershell
# Replace <PI_IP_ADDRESS> and <PI_USERNAME> with your actual Pi info
ssh <PI_USERNAME>@<PI_IP_ADDRESS> "mkdir -p ~/drone_vision"
    scp -r * <PI_USERNAME>@<PI_IP_ADDRESS>:~/drone_vision/
```

---

## 🐍 Step 3: Setup Python Environment
SSH back into your Raspberry Pi. We need to create a Python Virtual Environment. 

**⚠️ CRITICAL:** You *must* use the `--system-site-packages` flag. If you don't, your virtual environment will not be able to see the Hailo-10H drivers we installed in Step 1!

```bash
cd ~/drone_vision

# Create virtual environment with system packages enabled
python3 -m venv camvenv --system-site-packages

# Activate the environment
source camvenv/bin/activate

# Install required Python libraries
pip install -r requirements.txt
```

---

## ⚙️ Step 4: Verify Configuration
Before running, open `vision/opencv_processing.py` and ensure the configuration toggles at the very top of the file match your setup:

```python
# ==========================================
# --- CONFIGURATION TOGGLES ---
# ==========================================
USE_VIDEO_FILE = False  # Set to False to use the live hardware camera
VIDEO_FILE_PATH = "F3video1_Flipped.mkv" # Ignored if USE_VIDEO_FILE is False
FLIP_CAMERA = True      # Set to True if your camera is mounted upside down on the drone
# ==========================================
```

---

## ▶️ Step 5: Run the Vision System
With the virtual environment activated, simply run the main script:

```bash
# Ensure you are in the directory and the venv is active
cd ~/drone_vision
source camvenv/bin/activate

# Run the app
python -m scripts.run_vision
```

You should see terminal output confirming the camera calibration loaded, the Hailo-10H model initialized, and the Flask server starting.

---

## 📺 Step 6: View the Live Telemetry Feed
To view the real-time AI processing, open a web browser on your laptop (must be on the same WiFi network as the Pi) and navigate to:

**`http://<PI_IP_ADDRESS>:5000`**

### Understanding the HUD (Heads-Up Display)
On the live stream, you will see the detected gates and an overlay matrix in the top left corner:
* **Green Rows:** An active, 3D-tracked gate.
* **Red Rows:** Empty padding rows (displays `999.00`).
* **Format:** `[Distance, X(Fwd), Y(Rgt), Z(Dwn), Roll, Pitch, Yaw]`
* **Units:** Meters and Degrees.

*Note: The kinematics are pre-mapped to the standard Drone FRD (Forward-Right-Down) coordinate frame.*
