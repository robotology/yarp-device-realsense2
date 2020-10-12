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
ENABLE_yarpcar_depthimage2
```

### Build and install yarp-device-realsense2

```bash
mkdir build
cd build
cmake ..
make
make install
```

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
      <protocol>udp+recv.portmonitor+type.dll+file.depthimage2</protocol>
    </connection>

</application>
```

:bulb: **NOTE:** When using the camera *locally* and, not over a network, the user can substitute the protocol used for visualizing the depth image as follows:

```xml
<protocol>unix_stream+recv.portmonitor+type.dll+file.depthimage2</protocol>
```

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
