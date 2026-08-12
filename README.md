# Insta360 ROS 2 EquiLib Driver

A minimal ROS 2 driver for an Insta360 camera. It contains only camera capture,
H.264 decoding, and the CUDA/CPU PyTorch + PyEquiLib stitcher.

It uses the existing workspace's `jazzy360` Pixi environment; do not create a
separate Python environment. see **[parent repository](https://github.com/avasalya/pixi_insta360_ros2_jazzy_driver)**  for installation procedure.


# Insta360 ROS2 Jazzy Driver with Pixi Environment

A ROS driver for the Insta360 cameras. This driver is tested on Ubuntu 24.04 with ROS2 Jazzy on old classic RTX1080Ti. The driver has also been verified on the Insta360 X2 and X3 cameras. The following resolutions are available, all at 30 FPS.
- 3840 x 1920
- 2560 x 1280
- 2304 x 1152
- 1920 x 960
- 1152 x 1152 (default)

you can update `video_resolution` in parameter config file.

```bash
 src/insta360_ros2_equilib_driver/launch/bringup.launch.xml
 ```

# Installation

To use this driver, you need the latest Insta360 SDK (post-April 23, 2025), which can be requested via their [official website](https://insta360.com/sdk/home). For additional instructions, refer to this [post](https://github.com/ai4ce/insta360_ros_driver/issues/10#issuecomment-3371481987).

> ⚠️ **Note:** Do not manually clone or build this submodule directly. This package is managed within a `pixi` ecosystem to avoid environment conflicts. Also Please make you use the latest SDK. This package works with the SDK posted after April 23, 2025**


## Please follow the installation guide in the **[parent repository](https://github.com/avasalya/pixi_insta360_ros2_jazzy_driver)**.

```bash
# Clone with submodules
git clone -b equilib --recurse-submodules https://github.com/avasalya/pixi_insta360_ros2_jazzy_driver
cd pixi_insta360_ros2_jazzy_driver
```

**Add dependencies:**
Then, the Insta360 libraries need to be installed as follows:
- add the <code>camera</code> and <code>stream</code> header files inside the <code>include</code> directory
- add the <code>libCameraSDK.so</code> library under the <code>lib</code> directory.

**Build:** From the parent repository root, run:
```bash
# Setup: install dependencies and build
pixi run -e jazzy360 setup
```

# Setup Insta360 Camera

**make sure the camera is set to dual-lens (360°) mode**

Additionally, **ensure the camera's USB mode is set to Android**:
1. On the camera, swipe down the screen to the main menu
2. Go to Settings
3. Set USB Mode to **Android** (not Webcam or other modes)
4. This is required for the ROS driver to properly detect and communicate with the camera (see [Issue #4](https://github.com/ai4ce/insta360_ros_driver/issues/4))

The Insta360 requires sudo privilege to be accessed via USB. To compensate for this, a udev configuration can be automatically created that will only request for sudo once. The camera can thus be setup initially via:

```bash
cd ~/pixi_insta360_ros2_jazzy_driver/src/insta360_ros_driver
./setup.sh
```
This creates a symlink  based on the vendor ID of Insta360 cameras. The symlink, in this case <code>/dev/insta</code> is used to grant permissions to the usb port used by the camera.

**Sometimes, this does not work (e.g. you see "device /dev/insta not found" or something similar). You can try entering the commands manually, since that sometimes sees success, especially for the first time.**
```
echo SUBSYSTEM=='"usb"', ATTR{manufacturer}=='"Arashi Vision"', SYMLINK+='"insta"', MODE='"0777"' | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/insta
```

# Usage
The camera provides images natively in `H.264` or `H.264_cuvid` compressed image format. We have a decoder node that

## CUDA EquiLib projections

Set `equilib:=true` to use the PyTorch/CUDA dual-fisheye stitcher. It subscribes to
`/dual_fisheye/image`, applies the calibration in `config/equilib.yaml`, publishes
`/equirectangular/image`, and optionally generates `/perspective/image` with
[EquiLib](https://github.com/haruishi43/equilib).

```bash
pixi run -e jazzy360 ros2 launch insta360_ros2_equilib_driver bringup.launch.xml \
  equilib:=true equirectangular:=true perspective:=true
```

Set `gpu: false` in `config/equilib.yaml` to use the CPU path. The `equilib` launch
mode replaces the legacy C++ equirectangular and perspective nodes, so only one
projection pipeline runs at a time.


## Included pipeline

`insta360_camera` → `/dual_fisheye/image/compressed` → `decoder` →
`/dual_fisheye/image` → `equilib_stitcher.py` → `/equirectangular/image`

The perspective output `/perspective/image` is optional.

## Build and run from the existing workspace

Place this repository in the existing workspace root, then build it with the
already configured Pixi environment:

```bash
pixi run -e jazzy360 colcon build --base-paths insta360_ros2_equilib_driver --symlink-install
pixi run -e jazzy360 ros2 launch insta360_ros2_equilib_driver bringup.launch.xml equirectangular:=true perspective:=false
```

The Pixi environment must provide ROS 2 Jazzy, FFmpeg with CUDA decoding,
OpenCV, `cv_bridge`, NumPy, PyTorch, and PyEquiLib. The existing workspace's
`pixi.toml` already provides these dependencies.

Set `gpu: false` in `config/equilib.yaml` to run the stitcher on CPU. Adjust the
same file for the fisheye calibration and output dimensions.
