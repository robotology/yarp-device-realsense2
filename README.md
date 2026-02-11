![YARP logo](https://raw.githubusercontent.com/robotology/yarp/master/doc/images/yarp-robot-24.png "yarp-device-realsense2")
yarp-device-realsense2
======================

This is the realsense device for [YARP](https://www.yarp.it/).
It supports the Intel® RealSense cameras.

The **Intel® RealSense™** cameras currently compatible with YARP are:
- [Depth Camera D405](https://www.intelrealsense.com/depth-camera-d405/)
- [Depth Camera D415](https://www.intelrealsense.com/depth-camera-d415/)
- [Depth Camera D435](https://www.intelrealsense.com/depth-camera-d435/)
- [Depth Camera D455](https://www.intelrealsense.com/depth-camera-d455/)
- [LiDAR Camera L515](https://www.intelrealsense.com/lidar-camera-l515/)
- [Depth Camera with IMU D435i](https://www.intelrealsense.com/depth-camera-d435i/)
- [Tracking Camera T265](https://www.intelrealsense.com/tracking-camera-t265/)

License
-------

[![License](https://img.shields.io/badge/license-BSD--3--Clause%20%2B%20others-19c2d8.svg)](https://github.com/robotology/yarp-device-realsense2/blob/master/LICENSE)

This software may be modified and distributed under the terms of the
BSD-3-Clause license. See the accompanying LICENSE file for details.

The realsense2 device uses the
[librealsense](https://github.com/IntelRealSense/librealsense) library, released
under the
[Apache-2.0](https://github.com/IntelRealSense/librealsense/blob/master/LICENSE)
license.
See the relative documentation for the terms of the license.

How to use Intel RealSense cameras as a YARP device
---------------------------------------------------

First of all, please install the `yarp-device-realsense2` following one (and just one) method listed below, and the move to the `Usage` section.

## Install with conda or pixi (easy)

You can easily the library with [`conda`](https://docs.conda.io/en/latest/) in a new conda environment with
```
conda create -n newenvname -c conda-forge yarp-device-realsense2
```

`conda` will automatically install all the supported dependencies.

To add yarp-device-realsense2 to a [`pixi`](https://pixi.sh/latest/) project, just run:

```
pixi add yarp-device-realsense2
```

## Installation from source (advanced)

If you want to use a package manager (for example `apt` on Ubuntu) that does not provide `yarp-device-realsense2` packages, you can do that
as `yarp-device-realsense2` is a fairly standard CMake project. To do that, first of all install either via a package
manager or manually the following depencies:

### Dependencies
Before proceeding further, please install the following dependencies:

- [librealsense](https://github.com/IntelRealSense/librealsense)

### YARP compilation from source

If you are using `yarp` binary package from conda-forge or you compiled `yarp` via the `robotology-superbuild`, go to the next section.

**Only** if instead you manually compiled YARP, please make sure that the following CMake flags are enabled in the YARP CMake build:
```
ENABLE_yarpmod_rgbdSensor_nws_yarp
ENABLE_yarpmod_RGBDSensorClient
ENABLE_yarpcar_mjpeg
ENABLE_yarppm_depthimage_to_rgb
```

### Build and install yarp-device-realsense2

```bash
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<installation_path> ..
make
make install
```

In order to make the device detectable, add `<installation_path>/share/yarp` to the `YARP_DATA_DIRS` environment variable of the system.

Alternatively, if `YARP` has been installed using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild), it is possible to use `<directory-where-you-downloaded-robotology-superbuild>/build/install` as the `<installation_path>`.

## Usage

### How to use a RealSense D435

#### As `yarpmanager` application
You can write a yarp application to launch the camera and see the RGB and Depth images. Here is a simple example:

```xml
<application>
    <name>simpleRealsense</name>
    <description>Description</description>
    <version>1.0</version>

    <module>
      <name>yarpdev</name>
          <parameters> --from sensors/RealSense_conf.ini </parameters>
      <node>localhost</node>
    </module>  

    <module>
      <name>yarpview</name>
      <parameters>--name /view/rgb:i</parameters>
      <node>localhost</node>
    </module>

    <module>
      <name>yarpview</name>
      <parameters>--name /view/depth:i</parameters>
      <node>localhost</node>
    </module>

    <connection>
      <from>/depthCamera/rgbImage:o</from>
      <to> /view/rgb:i</to>
      <protocol>mjpeg</protocol>
    </connection>

    <connection>
      <from>/depthCamera/depthImage:o</from>
      <to>/view/depth:i</to>
      <protocol>udp+recv.portmonitor+type.dll+file.depthimage_to_rgb</protocol>
    </connection>

</application>
```

:bulb: **NOTE:** When using the camera *locally* and, not over a network, the user can substitute the protocol used for visualizing the depth image as follows:

```xml
<protocol>unix_stream+recv.portmonitor+type.dll+file.depthimage_to_rgb</protocol>
```

:bulb: **NOTE:** When using YARP < 3.5, the final part of the `protocol` string should be changed from `+file.depthimage_to_rgb` to `+file.depthimage2`.


### From the command line

Set the `YARP_ROBOT_NAME` to either `CER01`, `CER02` or `CER03`. Then, run the command:

```bash
yarpdev --from sensors/RealSense_conf.ini
```

By setting the `YARP_ROBOT_NAME`, YARP finds the configuration file `RealSense_conf.ini` automatically.

`RealSense_conf.ini` contains the following settings:

```ini
device            deviceBundler
wrapper_device    rgbdSensor_nws_yarp
attached_device   realsense2
name              /depthCamera

[SETTINGS]
depthResolution (480 270)    #Other possible values (424 240) or (640 480)
rgbResolution   (424 240)	 #Other possible values (424 240) or (640 480)
framerate       30
enableEmitter   true
needAlignment   true
alignmentFrame  RGB

[HW_DESCRIPTION]
clipPlanes (0.2 10.0)
```

:warning: **WARNING:** the user might have to change the parameters `depthResolution` and `rgbResolution`. These parameters change depending on the firmware version installed on the used camera.

### How to use a RealSense D405

To use this model of camera you can follow the same instructions above with the following **notable** differences:

- this camera does not have an active emitter system, hence it should be disabled as not supported;
- this camera does not have separate RGB and depth sensors - it only has a depth sensor that also provides RGB images - hence only the depth resolution should be set;
- this camera does not need active software-based alignment per construction - see [here](https://github.com/IntelRealSense/librealsense/issues/11784#issuecomment-1539851902) - hence the alignment should be disabled;
- this camera is a short-range camera, hence it is suggested to provide an adequately low lower bound for the clipping planes.

Considering the above, a possible configuration file might be the following:

```ini
device            deviceBundler
wrapper_device    rgbdSensor_nws_yarp
attached_device   realsense2
name              /depthCamera

[SETTINGS]
depthResolution (640 480) # <-- Same resolution as the rgbResolution
framerate       30
alignmentFrame  None      # <-- alignment set to None
                          # <-- 'enableEmitter' removed as not supported

[HW_DESCRIPTION]
clipPlanes (0.01 10.0)    # <-- lower bound for clipping planes set to 0.01 meters
```

### How to use a RealSense D435i

:construction: UNDER CONSTRUCTION :construction:

### How to use a RealSense T265

#### From the command line

The device retrieving the  RealSense T265 data is the `multipleanalogsensorserver` and it must be run as follows:

```bash
yarpdev --device deviceBundler --wrapper_device multipleanalogsensorsserver --attached_device realsense2Tracking --name /t265 --period 10
```

:bulb: **NOTE:** the user should specify the parameters `--name` and `--period` as needed.

The data from a RealSense T265 will be streamed on the port named `/t265/measures:o` following the format described in the `SensorStreamingData` class.

The type on information currently made available are:

- the Gyroscope measures,
- the Accelerometer measures, and
- the Pose (position and orientation).

## Device documentation

This device driver exposes the `yarp::dev::IRGBDSensor` and
`yarp::dev::IFrameGrabberControls` interfaces to read the images and operate on
the available settings.
See the documentation for more details about each interface.

This device is paired with its server called `rgbdSensor_nws_yarp` to stream the
images and perform remote operations.

The configuration file is subdivided into 2 major sections called `SETTINGS` and
`HW_DESCRIPTION`.

The `SETTINGS` section is meant for read/write parameters, meaning parameters
which can be get and set by the device.
A common case of setting is the image resolution in pixel.
This setting will be read by the device and it'll be applied in the startup
phase.
If the setting fails, the device will terminate the execution with a error
message.

The `HW_DESCRIPTION` section is meant for read only parameters which describe
the hardware property of the device and cannot be provided by the device through
software API.
A common case is the 'Field Of View' property, which may or may not be supported
by the physical device.
When a property is present in the `HW_DESCRIPTION` group, the YARP
`RGBDSensorClient` will report this value when asked for and setting will be
disabled.
This group can also be used to by-pass realsense2 API in case some functionality
is not correctly working with the current device.
For example the `clipPlanes` property may return incorrect values or values
using non-standard measurement unit.
In this case using the `HW_DESCRIPTION`, a user can override the value got from
OpenNI2 API with a custom value.

**Note**: Parameters inside the `HW_DESCRIPTION` are read only, so the relative
set functionality will be disabled.

**Note**: For parameters which are neither in `SETTINGS` nor in `HW_DESCRIPTION`
groups, read / write functionality is assumed but not initial setting will be
performed. Device will start with manufacturer default values.

**Warning**: A single parameter cannot be present into both `SETTINGS` and
`HW_DESCRIPTION` groups.

**Warning** whenever more then one value is required by the setting, the values
must be in parentheses!

| YARP device name |
|:----------------:|
| `realsense2`     |

Parameters used by this device are:

| Parameter name               | SubParameter      | Type           | Read / write | Units   | Default Value | Required        | Description                                                                           | Notes                                                                 |
|:----------------------------:|:-----------------:|:--------------:|:------------:|:-------:|:-------------:|:---------------:|:-------------------------------------------------------------------------------------:|:---------------------------------------------------------------------:|
|  `stereoMode`                |     -             | bool           | Read / write |         |   false       |  No(see notes)  | Flag for using the realsense as stereo camera                                         |  This option is to use it with yarp::dev::ServerGrabber as network wrapper. The stereo images provided are raw images(yarp::sig::PixelMono) and note that not all the realsense devices have the stereo streams. |
|  `verbose`                   |     -             | bool           | Read / write |         |   false       |  No             | Flag for enabling debug prints                                                        |                                                                       |
|  `rotateImage180`            |                   | bool           | Read / write | -       |   -           |  No             | Flag for enabling rotating the image 180 degrees                                      |  Parameter useful when a camera is mounted upside down |
|  `SETTINGS`                  |     -             | group          | Read / write | -       |   -           |  Yes            | Initial setting of the device.                                                        |  Properties must be read/writable in order for setting to work        |
|                              | `rgbResolution`   | int, int       | Read / write | pixels  |   -           |  Yes            | Size of rgb image in pixels                                                           |  2 values expected as height, width                                   |
|                              | `depthResolution` | int, int       | Read / write | pixels  |   -           |  Yes            | Size of depth image in pixels                                                         |  Values are height, width                                             |
|                              | `accuracy`        | double         | Read / write | meters  |   -           |  No             | Accuracy of the device, as the depth measurement error at 1 meter distance            |  Note that only few realsense devices allows to set it                |
|                              | `framerate`       | int            | Read / Write | fps     |   30          |  No             | Framerate of the sensor                                                               |                                                                       |
|                              | `enableEmitter`   | bool           | Read / Write | -       |   true        |  No             | Flag for enabling the IR emitter(if supported by the sensor)                          |                                                                       |
|                              | `needAlignment`   | bool           | Read / Write | -       |   true        |  No             | Flag for enabling the alignment of the depth frame over the rgb frame                 |  This option is deprecated, please use alignmentFrame instead.        |
|                              | `alignmentFrame`  | string         | Read / Write | -       |   RGB         |  No             | This parameter specifies the frame to which the frames RGB and Depth will be aligned. |  The accepted values are RGB, Depth, None. This operation could be heavy, set it to None to increase the fps.|
|  `HW_DESCRIPTION`            |     -             | group          |              | -       |   -           |  Yes            | Hardware description of device property.                                              |  Read only property. Setting will be disabled                         |
|                              | `clipPlanes`      | double, double | Read / write | meters  |   -           |  No             | Minimum and maximum distance at which an object is seen by the depth sensor           |  parameter introduced mainly for simulated sensors, it can be used to set the clip planes if Openni gives wrong values |

Configuration file using `.ini` format, for using as RGBD device:

```ini
device            deviceBundler
wrapper_device    rgbdSensor_nws_yarp
attached_device   realsense2
name              /depthCamera

[SETTINGS]
depthResolution (640 480)    #Note the parentheses
rgbResolution   (640 480)
framerate       30
enableEmitter   true
alignmentFrame  RGB

[HW_DESCRIPTION]
clipPlanes (0.2 10.0)
```

Configuration file using `.ini` format, for using as stereo camera:


```ini
device            deviceBundler
wrapper_device    grabberDual
attached_device   realsense2
name              /stereoCamera
capabilities RAW
stereoMode   true

[SETTINGS]
rgbResolution   (640 480)
depthResolution   (640 480)
framerate       30
enableEmitter   false

[HW_DESCRIPTION]
clipPlanes (0.2 10.0)
```

Maintainers
--------------
This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/Nicogene.png" width="40">](https://github.com/Nicogene) | [@Nicogene](https://github.com/Nicogene) |

