# Contributing

Issues and pull requests are welcome.

Do not commit the proprietary Insta360 Camera SDK, camera recordings, build
outputs, credentials, or machine-specific configuration. Keep changes focused,
document user-visible behavior, and use the package's Pixi environment for
consistent dependencies.

Before opening a pull request:

```bash
git diff --check
pixi run build
```

Hardware-dependent changes should state the camera, GPU, driver, CUDA, and SDK
versions used for testing. If the SDK has no published version identifier,
provide the download date and SHA-256 of `libCameraSDK.so` instead.
