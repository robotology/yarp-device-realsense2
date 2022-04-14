# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Added `rotateImage180` parameter to rotate the image when the camera is mounted upside down (https://github.com/robotology/yarp-device-realsense2/pull/27).

### Changed
- If all the distortion parameters are zero, explicitly specify that the image has `YARP_DISTORTION_NONE` distortion (https://github.com/robotology/yarp-device-realsense2/pull/26).
- Changed minimum required YARP version to 3.5 (https://github.com/robotology/yarp-device-realsense2/pull/26).

## [0.2.0] - 2021-05-28

### Added
- Added `analogServer` device to stream T256 pose. (See [!18](https://github.com/robotology/yarp-device-realsense2/pull/18)).

## [0.1.0] - 2021-04-27

First release of the `yarp-device-realsense2`, tested to be compatible with YARP 3.4 .
