# X3_ROS2_ws — Robot Setup & Docker Deployment Guide

This guide covers how to set up a new Yahboom ROS Master X3 robot, build the Docker container, configure GPU libraries, and launch the workspace for the first time.

---

## Hardware & Software Prerequisites

### Robot Platform
- **Robot:** Yahboom ROS Master X3
- **SoM:** Jetson Orin Nano / NX
- **JetPack:** 6.2.1 (Ubuntu 22.04)
- **CUDA:** 12.6 | **TensorRT:** 10.3 | **cuDNN:** 9.3

### Required Host Software
The following must be installed on the robot's host system (outside Docker) before proceeding.

**Docker Engine**
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add your user to the docker group so you don't need sudo
sudo usermod -aG docker $USER
newgrp docker
```

**NVIDIA Container Toolkit**

Required for GPU access inside the container.
```bash
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

**Git**
```bash
sudo apt install -y git
```

---

## Cloning the Repository

```bash
cd ~
git clone https://github.com/<your-org>/X3_ROS2_ws.git
```

The workspace directory `~/X3_ROS2_ws` is volume-mounted into the container at runtime, so the Docker image itself does not contain workspace source code. This means you can update your ROS2 packages and rebuild inside the container without ever rebuilding the image. Edits to the repo inside the container is reflected on the host machine.

> **Submodules:** The container entrypoint (`ros_entrypoint.sh`) automatically initializes git submodules (e.g. `rplidar_ros`) on first start. If you need to do this manually beforehand:
> ```bash
> cd ~/X3_ROS2_ws
> git submodule update --init --recursive
> ```

---

## Step 1 — Build the Docker Image

From the repo root, run the build script:

```bash
cd ~/X3_ROS2_ws
bash robot_ws/build.sh
```

This builds the image tagged `ros2_humble_img` from the `Dockerfile`. It installs all system and ROS2 dependencies, including:

- ROS2 Humble base packages (xacro, robot_localization, joint/robot state publishers)
- Orbbec Astra Pro Plus camera dependencies (libuvc, libusb, image_transport, etc.)
- Python packages: `pyserial`, `numpy`, `matplotlib`, `opencv-python`, `stable-baselines3`
- The Rosmaster serial driver (`rosmaster_driver_install`)

The CUDA 12.6 toolkit path is baked into `LD_LIBRARY_PATH` and `PATH` inside the image.

> **Build time:** Expect 10–20 minutes on first build due to apt package downloads and the libuvc source build.

---

## Step 2 — Copy GPU Libraries

Before starting the container for the first time, run the GPU library setup script. This copies TensorRT and cuDNN shared libraries from the host into a `gpu_libs/` folder tracked inside the repo, making them available to the container at runtime without requiring a broad host mount.

```bash
bash robot_ws/setup_gpu_libs.sh
```

What this script does:
- Copies TensorRT and cuDNN `.so` libraries from host system paths (e.g. `/usr/lib/aarch64-linux-gnu/`) into `robot_ws/gpu_libs/`
- Uses `cp -L` to dereference symlinks, so the actual library binaries are stored (not broken symlink pointers)
- The `gpu_libs/` directory is committed to the repo and included in the container's `LD_LIBRARY_PATH`

> **Re-run this script if you update JetPack, TensorRT, or cuDNN on the host.**

---

## Step 3 — Start the Container

```bash
bash robot_ws/start_container.sh
```

This launches a container named `ros2_humble` from the `ros2_humble_img` image with the following configuration:

| Option | Purpose |
|---|---|
| `--restart unless-stopped` | Container restarts automatically on reboot |
| `--net=host` | Robot's ROS2 topics are visible on the network (required for RViz2 on dev machine) |
| `--privileged` | Full device access (serial ports, USB cameras, LiDAR) |
| `-v /dev:/dev` | Passes through all device nodes |
| `-v /usr/local/cuda-12.6` | Mounts CUDA toolkit from host (read-only) |
| `-v ~/X3_ROS2_ws:/X3_ROS2_ws` | Mounts workspace at runtime |
| `NVIDIA_VISIBLE_DEVICES=all` | Exposes all GPU devices to the container |
| `NVIDIA_DRIVER_CAPABILITIES=all` | Enables all NVIDIA capabilities (compute, graphics, video) |
| `TZ=Canada/Atlantic` | Sets the container timezone |

---

## Step 4 — Build the ROS2 Workspace

Enter the running container and build:

```bash
docker exec -it ros2_humble bash

# Inside the container:
cd /X3_ROS2_ws
colcon build --symlink-install
source install/setup.bash
```

The `--symlink-install` flag means Python nodes can be edited in place without rebuilding.

---

## Step 5 — WiFi Configuration (Lab / Headless Setup)

To have the robot connect to your lab WiFi automatically on startup (and suppress any hotspot autoconnect), use `nmcli` on the host:

```bash
# Connect to lab WiFi
sudo nmcli device wifi connect "<SSID>" password "<password>"

# Disable hotspot autoconnect if present
sudo nmcli connection modify "<hotspot-connection-name>" connection.autoconnect no
```

---

## ONNX Runtime GPU (for Inference Nodes)

The face detection and other inference nodes require ONNX Runtime with GPU support. The standard PyPI wheel does **not** support Jetson/aarch64 — use the Jetson AI Lab prebuilt wheel instead:

```bash
# Inside the container:
pip3 install onnxruntime-gpu \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
```

To verify GPU execution providers are available after install:
```python
import onnxruntime as ort
print(ort.get_available_providers())
# Should include 'TensorrtExecutionProvider' and 'CUDAExecutionProvider'
```

> A cosmetic warning about `/sys/class/drm/card1/device/vendor` may appear on startup — this is a known Tegra quirk and does not affect inference functionality.

> **Baking into the image (optional):** To avoid downloading this wheel on every fresh container, you can pre-download it and install offline:
> ```bash
> # On host or container:
> pip3 download onnxruntime-gpu \
>   --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
>   -d robot_ws/pip_cache/
> ```
> Then add a `COPY` + `RUN pip3 install` step to the Dockerfile pointing at the cached `.whl` file.

---

## Launching Nodes

**Important — Teleop / keyboard input nodes** must be launched with `ros2 run` in a TTY-attached terminal session, not via `ros2 launch`. The `ros2 launch` process spawns child processes that do not inherit a TTY, which breaks `termios`-based keyboard input:

```bash
# Correct — TTY attached
docker exec -it ros2_humble ros2 run <package> <teleop_node>

# Incorrect for keyboard nodes
ros2 launch <package> teleop.launch.py
```

For all other nodes, launch files work normally:
```bash
docker exec -it ros2_humble bash -c \
  "source /X3_ROS2_ws/install/setup.bash && ros2 launch <package> <launch_file>"
```

---

## RViz2 on a Remote Dev Machine

RViz2 runs on a separate machine connected to the same lab network. Because the container uses `--net=host`, all ROS2 topics are automatically discoverable via DDS multicast.

Ensure the dev machine has the robot's URDF packages installed and sourced (for `package://` URI mesh resolution in RViz2), and that `ROS_DOMAIN_ID` matches between the robot and dev machine if you have customized it.

---

## Troubleshooting

**Container doesn't see the GPU**
- Confirm `nvidia-container-toolkit` is installed and Docker was restarted after configuration.
- Run `tegrastats` on the host to verify the GPU is healthy.

**Serial port / LiDAR not found**
- Confirm the `ros` user is in the `dialout` group (already set in the Dockerfile).
- Check `ls /dev/ttyUSB*` or `ls /dev/ttyACM*` on the host and inside the container.

**Camera USB device not accessible**
- The `--privileged` flag and `-v /dev:/dev` mount should handle this. If permissions are still denied, check `udev` rules on the host for the Orbbec device.

**Submodules missing (e.g. rplidar_ros)**
- The entrypoint runs `git submodule update --init --recursive` automatically, but only if the workspace is mounted. Confirm `~/X3_ROS2_ws` is present on the host before starting the container.

**rqt plugin discovery errors on dev machine**
- This is a known `rqt` cache issue. Clear the cache and relaunch:
  ```bash
  rm -rf ~/.config/ros.org
  rqt --force-discover
  ```