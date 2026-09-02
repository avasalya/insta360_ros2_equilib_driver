# Insta360 ROS 2 CUDA Driver

ROS 2 Jazzy driver for an Insta360 X3 that captures the dual-fisheye H.264
stream, decodes it, and stitches it into a full-resolution equirectangular
image on an NVIDIA GPU.

This repository builds and runs on its own.

> [!IMPORTANT]
> This is an independent community project. It is not affiliated with or
> endorsed by Insta360. The proprietary Insta360 Camera SDK is not included.

## Pipeline

```text
Insta360 X3
  -> /dual_fisheye/image/compressed (H.264)
  -> decoder
  -> /dual_fisheye/image (bgr8)
  -> native CUDA stitcher
  -> /equirectangular/image (bgr8)
```

Installed executables:

- `insta360_ros2_cuda_driver`: uses the proprietary Camera SDK and publishes
  H.264 packets.
- `decoder`: decodes H.264 into ROS images; the launch file requests
  `h264_cuvid` hardware decoding.
- `cuda_stitcher`: performs the native C++/CUDA fisheye-to-equirectangular
  remap.
- `equilib_stitcher.py`: optional PyTorch/PyEquilib fallback.

The decoder is required with the native stitcher: the stitcher consumes
decoded images, not H.264 packets.

## Improvements over the original driver

This project extends the original AI4CE ROS driver with:

- a native C++/CUDA dual-fisheye stitcher for real-time equirectangular output;
- configurable NVIDIA NVDEC hardware decoding and preserved source timestamps;
- selectable Camera SDK streaming resolutions up to `RES_3840_1920P30`;
- automatic stitch geometry derived from the decoded frame dimensions;
- automatic camera reconnection and stalled-stream recovery;
- frame-rate, latency, dropped-frame, and NVTX performance diagnostics;
- a reproducible standalone ROS 2 Jazzy environment managed with Pixi.

The original camera capture and decoder foundations remain credited and
Apache-2.0 licensed; the new independent components are MIT licensed.

## Tested configuration

- Ubuntu 24.04 and ROS 2 Jazzy from RoboStack
- Insta360 X3 and NVIDIA RTX 2080 Ti / RTX 4070
- CUDA 12.8
- 2304 x 1152 input and equirectangular output at 30 FPS

The default CUDA target is compute capability 7.5. Override it at build time
with `-DCMAKE_CUDA_ARCHITECTURES=<value>` for another GPU.

## Prerequisites

Install an NVIDIA driver compatible with CUDA 12.8, Git, and
[Pixi](https://pixi.sh/latest/installation/). Confirm GPU access:

```bash
nvidia-smi
```

### Proprietary Insta360 Camera SDK

Create an Insta360 developer account and download the current **Camera SDK for
Linux** from the
[Insta360 Developer portal](https://www.insta360.com/developer/home). Accept
and follow Insta360's SDK license. The headers and binary are proprietary and
are intentionally excluded from Git.

This driver was validated with the Linux SDK downloaded on **2026-08-28**. The
downloaded bundle and library do not expose a reliable Camera SDK release
number. The exact tested `libCameraSDK.so` is identified by:

```text
SHA-256: d55091f120fa04632decd2230c1ebf5e0c87037d7ec01ffae089eb4a9236b2ad
ELF build ID: a874e4ec041cc097d126dc539572e908194b8d2f
```

Use an SDK published after 2025-04-23; older SDKs are not compatible with the
upstream X3 driver API. A newer SDK is suitable if it retains the API used
here, including
`LiveStreamParam::lrv_video_resulution`. Copy these files from the downloaded
SDK into the repository:


## Create the standalone Pixi environment

```bash
git clone https://github.com/avasalya/insta360_ros2_cuda_driver.git
cd insta360_ros2_cuda_driver
pixi install
```

The committed `pixi.toml` contains the minimal native driver dependencies plus
the optional Python fallback. No lockfile is committed; Pixi resolves the
environment locally.

## SDK setup

```bash
cd insta360_ros2_cuda_driver
mkdir lib include
```

Copy `include/camera` and `include/stream` from sdk and copy to `insta360_ros2_cuda_driver/include`
and copy `libCameraSDK.so` to `insta360_ros2_cuda_driver/lib`


```text
insta360_ros2_cuda_driver/
├── include/
│   ├── camera/
│   ├── stream/
│   └── insta360_ros2_cuda_driver/
│       └── cuda_stitcher.hpp
└── lib/
    └── libCameraSDK.so
```

Never force-add these files. Verify that Git ignores them:

```bash
git check-ignore include/camera/camera.h \
  include/stream/stream_types.h \
  lib/libCameraSDK.so
```

## Camera setup

Set the camera to dual-lens 360 mode and USB mode **Android**. Connect it and
install the udev rule:

```bash
./setup.sh
```

The rule grants camera access to the `plugdev` group. If needed, run
`sudo usermod -aG plugdev "$USER"`, then log out and back in.

Verify the device:

```bash
lsusb | grep 2e1a
ls -l /dev/insta
```

An X3 normally appears as `2e1a:0002 Arashi Vision Insta360 X3`. If it does
not appear, power it off, remove and reinsert the battery, turn it on, restore
Android USB mode, and reconnect it.

## Build

```bash
pixi run build
```

For a GPU other than the default compute capability 7.5, run the underlying
command with the appropriate architecture:

```bash
pixi run colcon build \
  --packages-select insta360_ros2_cuda_driver \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=89 \
  --symlink-install
```

The example `89` targets an RTX 4070.

If this workspace previously contained the package under its old name, start
a fresh terminal and remove only the stale package output:

```bash
rm -rf build/insta360_ros2_equilib_driver \
       install/insta360_ros2_equilib_driver
```

Do not source an old `install/setup.bash`; it can retain the removed package
in `CMAKE_PREFIX_PATH` and `PYTHONPATH`.

## Run

Launch the complete native CUDA pipeline:

```bash
pixi run launch
```

For headless operation:

```bash
pixi run launch-headless
```

For other launch combinations, source `install/setup.bash` inside the Pixi
environment and pass the arguments listed below.

### Optional `run360` shell function

`RUN360_VIEWER` and `RUN360_NATIVE` are convenience variables, not package
requirements. To keep the short `run360` command, add this portable version to
`~/.bashrc` and start a new shell:

```bash
run360()
{
    local viewer="${1:-true}"
    local native_cuda="${2:-true}"

    pixi run env RUN360_VIEWER="$viewer" RUN360_NATIVE="$native_cuda" \
      bash -lc '
        source install/setup.bash
        ros2 launch insta360_ros2_cuda_driver bringup.launch.xml \
          equirectangular:=true \
          perspective:=false \
          viewer:="$RUN360_VIEWER" \
          native_cuda_stitcher:="$RUN360_NATIVE"
      '
}
```

Run it from the repository root:

```bash
run360                 # viewer and native CUDA enabled
run360 false true      # headless with native CUDA enabled
```

RouDi is part of the larger Panotrack shared-memory stack, not this ROS camera
package, so the standalone helper does not start it.

## Launch arguments

| Argument | Default | Purpose |
|---|---:|---|
| `equirectangular` | `true` | Publish `/equirectangular/image` |
| `perspective` | `false` | Publish the fallback perspective view |
| `viewer` | `false` | Open `image_view` on the selected output |
| `native_cuda_stitcher` | `true` | Use the native C++/CUDA stitcher |
| `cuda_visible_devices` | `0` | Select the visible NVIDIA GPU |
| `video_resolution` | `RES_3840_1920P30` | Select the Camera SDK streaming resolution |
| `equilib_config` | package config | Override the stitcher YAML path |

The viewer selects `/equirectangular/image`, `/perspective/image`, or the raw
`/dual_fisheye/image`, according to the enabled outputs.

The driver accepts these Camera SDK resolution names:

- `RES_1152_1152P30` (default)
- `RES_1440_720P30`
- `RES_1920_960P30`
- `RES_2560_1280P30`
- `RES_2880_2880P30`
- `RES_3840_1920P30` (maximum supported by this driver)

Select a resolution at launch time:

```bash
ros2 launch insta360_ros2_cuda_driver bringup.launch.xml \
  video_resolution:=RES_2560_1280P30
```

Actual mode availability still depends on the connected camera and Camera SDK.
The `CUDA map initialized: <input> -> <output>` log reports the decoded frame
dimensions actually received by the stitcher.

## Stitcher configuration

Calibration and output settings are in `config/equilib.yaml`. The native
stitcher consumes `cx_offset`, `cy_offset`, `crop_size`, `translation`,
`rotation_deg`, `out_width`, `out_height`, `fisheye_fov_deg`, and
`publish_equirectangular`.

By default, `crop_size`, `out_width`, and `out_height` are zero, which enables
automatic sizing from the decoded dual-fisheye frame. For a standard side-by-side
frame, the stitcher uses the largest square lens crop and produces a native-size
2:1 panorama. Positive values remain available as explicit overrides. Calibrate
the remaining values for the individual camera before relying on geometric
accuracy.

## Python fallback

PyTorch and PyEquilib are already included in `pixi.toml`. Launch with
`native_cuda_stitcher:=false` to use them. Perspective output exists only in
this fallback path.

## Topics

| Topic | Type | Producer |
|---|---|---|
| `/dual_fisheye/image/compressed` | `sensor_msgs/msg/CompressedImage` | camera capture |
| `/dual_fisheye/image` | `sensor_msgs/msg/Image` (`bgr8`) | decoder |
| `/equirectangular/image` | `sensor_msgs/msg/Image` (`bgr8`) | stitcher |
| `/perspective/image` | `sensor_msgs/msg/Image` | Python fallback |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | camera capture |

Inspect rates in separate terminals:

```bash
source install/setup.bash
ros2 topic hz /dual_fisheye/image
ros2 topic hz /equirectangular/image
```

## H.264 hardware decoding

Check whether the environment's FFmpeg provides `h264_cuvid`:

```bash
pixi run ffmpeg -hide_banner -decoders | grep -E "h264_cuvid|cuvid"
```

If it is absent, install an FFmpeg build compiled with NVIDIA codec support.
Camera capture at 30 FPS alone does not prove decoded delivery; verify that
`/dual_fisheye/image` also has a nonzero rate.

## Troubleshooting

### Header not found after renaming the package

The header must be at:

```text
include/insta360_ros2_cuda_driver/cuda_stitcher.hpp
```

### Stale old-package prefix warning

If colcon reports a nonexistent `insta360_ros2_equilib_driver` path, open a
fresh terminal or clear the old workspace variables:

```bash
unset CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
```

### `non-existing PPS 0 referenced`

This commonly means the decoder subscribed after the initial H.264 SPS/PPS
data. Confirm the camera executable waits for the decoder subscriber before
starting the stream, then inspect each topic rate.

### Native stitcher reports zero input FPS

The fault is upstream of stitching. Check in order:

```bash
ros2 topic hz /dual_fisheye/image/compressed
ros2 topic hz /dual_fisheye/image
ros2 topic hz /equirectangular/image
```

### Camera SDK library not found at runtime

Confirm `lib/libCameraSDK.so` exists and inspect the executable:

```bash
ldd install/insta360_ros2_cuda_driver/lib/insta360_ros2_cuda_driver/insta360_ros2_cuda_driver \
  | grep CameraSDK
```

## Performance

Measured on an RTX 2080 Ti at 2304 x 1152:

```text
input=30.5 fps equirect=30.0 fps dropped=0
cuda=2.77 ms max_cuda=4.57 ms
```

The native path uses a precomputed coordinate map, one CUDA bilinear-remap
kernel, a non-blocking CUDA stream, pinned host output memory, a latest-frame
ROS queue, and `bgr8` end to end.

## Output examples

### Dual fisheye

![Dual fisheye](doc/fisheye.png)

### Equirectangular

![Equirectangular](doc/equirectangular.png)

### Perspective

![Perspective](doc/perspective.png)

## Credits

- Camera capture and decoder nodes are based on the Apache-2.0-licensed
  [AI4CE Insta360 ROS driver](https://github.com/ai4ce/insta360_ros_driver) and
  have been substantially modified for this project.
- The optional Python fallback uses [EquiLib](https://github.com/haruishi43/equilib).

## License

Independent original work is available under the [MIT License](LICENSE).
The derived camera and decoder nodes remain Apache-2.0; see
[third-party notices](THIRD_PARTY_NOTICES.md). The proprietary Insta360 Camera
SDK is separately licensed and is not included.
