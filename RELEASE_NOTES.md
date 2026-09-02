# v1.0.0 — 2026-09-02

First stable release of the standalone **Insta360 ROS 2 CUDA Driver**.

## Highlights

- Renamed the ROS package to `insta360_ros2_cuda_driver` to reflect its native
  CUDA pipeline.
- Added automatic camera reconnection and stalled-stream recovery.
- Added a standalone Pixi environment for ROS 2 Jazzy, CUDA 12.8, FFmpeg, and
  the optional Python fallback.
- Improved proprietary Camera SDK discovery, installation, runtime loading,
  and compatibility documentation.
- Made the target CUDA architecture configurable at build time.
- Added open-source licensing, upstream attribution, contribution guidance,
  and safer group-based USB permissions.

The default pipeline uses NVIDIA NVDEC for H.264 decoding and the native
C++/CUDA stitcher for 2304 x 1152 equirectangular output. The PyTorch/EquiLib
stitcher remains available as an optional fallback.

## Breaking change from v0.1

The package, executable, include path, and launch references changed from
`insta360_ros2_equilib_driver` to `insta360_ros2_cuda_driver`.

Rebuild in a fresh terminal and use:

```bash
pixi run build
pixi run launch
```

Remove stale v0.1 package output if it remains in an existing workspace:

```bash
rm -rf build/insta360_ros2_equilib_driver \
       install/insta360_ros2_equilib_driver
```

## SDK requirement

The proprietary Insta360 Camera SDK is not included. Download the Camera SDK
for Linux from the Insta360 Developer portal and follow the placement and
version-identification instructions in the README.

## Tested with

- Ubuntu 24.04 and ROS 2 Jazzy
- Insta360 X3
- NVIDIA RTX 2080 Ti and RTX 4070
- CUDA 12.8
- 2304 x 1152 output at 30 FPS
