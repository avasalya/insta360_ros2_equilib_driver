# v1.1.0 — 2026-09-11

Real-time standalone streaming and reproducible ffmpeg-CUDA dependency update.

## Highlights

- Added the named Pixi environment `insta360` and updated installation, build,
  and launch commands to select it explicitly.
- Added CycloneDDS shared-memory transport backed by RouDi so decoded and
  equirectangular image streams can sustain the camera's 30 FPS output.
- Added automatic RouDi startup, readiness checking, cleanup, and standalone
  shared-memory pool configuration.
- Added automatic native GPU architecture detection for Pixi builds while
  retaining compute capability 7.5 as the CMake-only fallback.
- Pinned FFmpeg and nv-codec-headers to tested commits and made their build
  tasks stop immediately on errors.
- Documented NVDEC verification, expected camera/stitcher statistics, and the
  portable `run360` helper.

## Runtime change

Use the Pixi launch tasks so CycloneDDS and RouDi are configured before ROS:

```bash
pixi install -e insta360
pixi run -e insta360 build
pixi run -e insta360 launch
pixi run -e insta360 launch-headless
```

Launching ROS directly without the bundled environment and RouDi wrapper can
reduce throughput for the large `bgr8` image topics.

## Pinned sources

- FFmpeg: `3a165c77dce7fac8c54f9f9aaab5447590433748`
- nv-codec-headers: `eddcea9e27f6b772057c9b3f87de2cc1737faffc`

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
