![YARP logo](https://raw.githubusercontent.com/robotology/yarp/master/doc/images/yarp-robot-24.png "yarp-device-realsense2")
yarp-device-realsense2
======================

This is the realsense device for [YARP](https://www.yarp.it/).
It supports the Intel® RealSense cameras.

The **Intel® RealSense™** cameras currently compatible with YARP are:
- [Depth Camera D435](https://www.intelrealsense.com/depth-camera-d435/)
- [Depth Camera with IMU D435i](https://www.intelrealsense.com/depth-camera-d435i/)
- [Tracking Camera T256](https://www.intelrealsense.com/tracking-camera-t265/)

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

### Dependencies
Before proceeding further, please install the following dependencies:

- [librealsense](https://github.com/IntelRealSense/librealsense)

### Configure YARP

Before proceeding further, please enable the following CMake flags in YARP:
```
ENABLE_yarpmod_RGBDSensorWrapper
ENABLE_yarpmod_RGBDSensorClient
ENABLE_yarpcar_mjpeg
ENABLE_yarppm_depthimage_to_rgb
```

:bulb: **NOTE:** If you use YARP < 3.5, instead of the option `ENABLE_yarppm_depthimage_to_rgb` you should enable the option `ENABLE_yarpcar_depthimage2`.

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


#### From the command line

Set the `YARP_ROBOT_NAME` to either `CER01`, `CER02` or `CER03`. Then, run the command:

```bash
yarpdev --from sensors/RealSense_conf.ini
```

By setting the `YARP_ROBOT_NAME`, YARP finds the configuration file `RealSense_conf.ini` automatically.

`RealSense_conf.ini` contains the following settings:

```ini
device       RGBDSensorWrapper
subdevice    realsense2
name         /depthCamera

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

### How to use a RealSense D435i

:construction: UNDER CONSTRUCTION :construction:

### How to use a RealSense T256

#### From the command line

The device retrieving the  RealSense T256 data is the `multipleanalogsensorserver` and it must be run as follows:

```bash
yarpdev --device multipleanalogsensorsserver --name /t256 --period 10 --subdevice realsense2Tracking
```

:bulb: **NOTE:** the user should specify the parameters `--name` and `--period` as needed.

The data from a RealSense T256 will be streamed on the port named `/t256/measures:o` following the format described in the `SensorStreamingData` class.

The type on information currently made available are:

- the Gyroscope measures,
- the Accelerometer measures, and
- the Pose (position and orientation).

## Device documentation

This device driver exposes the `yarp::dev::IRGBDSensor` and
`yarp::dev::IFrameGrabberControls` interfaces to read the images and operate on
the available settings.
See the documentation for more details about each interface.

This device is paired with its server called `RGBDSensorWrapper` to stream the
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
device       RGBDSensorWrapper
subdevice    realsense2
name         /depthCamera

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
device       grabberDual
subdevice    realsense2
name         /stereoCamera
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
| [<img src="https://github.com/traversaro.png" width="40">](https://github.com/traversaro) | [@traversaro](https://github.com/traversaro) |

