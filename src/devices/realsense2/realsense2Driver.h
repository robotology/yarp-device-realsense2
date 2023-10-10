/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef REALSENSE2_DRIVER_H
#define REALSENSE2_DRIVER_H

#include <iostream>
#include <cstring>
#include <map>
#include <mutex>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IFrameGrabberControls.h>
#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/RGBDSensorParamParser.h>
#include <librealsense2/rs.hpp>
#include <rs_types.h>


class realsense2Driver :
        public yarp::dev::DeviceDriver,
        public yarp::dev::IFrameGrabberControls,
        public yarp::dev::IFrameGrabberImageRaw,
        public yarp::dev::IRGBDSensor
{
private:
    typedef yarp::sig::ImageOf<yarp::sig::PixelFloat> depthImage;
    typedef yarp::os::Stamp                           Stamp;
    typedef yarp::os::Property                        Property;
    typedef yarp::sig::FlexImage                      FlexImage;
    const std::map<std::string, rs2_rs400_visual_preset>    presetsMap{
        {"CUSTOM",              rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_CUSTOM},
        {"DEFAULT",             rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_DEFAULT},
        {"HAND",                rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HAND},
        {"HIGH_ACCURACY",       rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY},
        {"PRESET_HIGH_DENSITY", rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_DENSITY},
        {"MEDIUM_DENSITY",      rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_MEDIUM_DENSITY},
        {"REMOVE_IR_PATTERN",   rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_REMOVE_IR_PATTERN}
    };

public:
    realsense2Driver();
    ~realsense2Driver() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IRGBDSensor
    int    getRgbHeight() override;
    int    getRgbWidth() override;
    bool   getRgbSupportedConfigurations(yarp::sig::VectorOf<yarp::dev::CameraConfig> &configurations) override;
    bool   getRgbResolution(int &width, int &height) override;
    bool   setRgbResolution(int width, int height) override;
    bool   getRgbFOV(double& horizontalFov, double& verticalFov) override;
    bool   setRgbFOV(double horizontalFov, double verticalFov) override;
    bool   getRgbMirroring(bool& mirror) override;
    bool   setRgbMirroring(bool mirror) override;

    bool   getRgbIntrinsicParam(Property& intrinsic) override;
    int    getDepthHeight() override;
    int    getDepthWidth() override;
    bool   setDepthResolution(int width, int height) override;
    bool   getDepthFOV(double& horizontalFov, double& verticalFov) override;
    bool   setDepthFOV(double horizontalFov, double verticalFov) override;
    bool   getDepthIntrinsicParam(Property& intrinsic) override;
    double getDepthAccuracy() override;
    bool   setDepthAccuracy(double accuracy) override;
    bool   getDepthClipPlanes(double& nearPlane, double& farPlane) override;
    bool   setDepthClipPlanes(double nearPlane, double farPlane) override;
    bool   getDepthMirroring(bool& mirror) override;
    bool   setDepthMirroring(bool mirror) override;


    bool   getExtrinsicParam(yarp::sig::Matrix &extrinsic) override;
    bool   getRgbImage(FlexImage& rgbImage, Stamp* timeStamp = nullptr) override;
    bool   getDepthImage(depthImage& depthImage, Stamp* timeStamp = nullptr) override;
    bool   getImages(FlexImage& colorFrame, depthImage& depthFrame, Stamp* colorStamp=NULL, Stamp* depthStamp=NULL) override;

    RGBDSensor_status     getSensorStatus() override;
    std::string getLastErrorMsg(Stamp* timeStamp = NULL) override;

    //IFrameGrabberControls
    bool   getCameraDescription(CameraDescriptor *camera) override;
    bool   hasFeature(int feature, bool*   hasFeature) override;
    bool   setFeature(int feature, double  value) override;
    bool   getFeature(int feature, double* value) override;
    bool   setFeature(int feature, double  value1,  double  value2) override;
    bool   getFeature(int feature, double* value1,  double* value2) override;
    bool   hasOnOff(  int feature, bool*   HasOnOff) override;
    bool   setActive( int feature, bool    onoff) override;
    bool   getActive( int feature, bool*   isActive) override;
    bool   hasAuto(   int feature, bool*   hasAuto) override;
    bool   hasManual( int feature, bool*   hasManual) override;
    bool   hasOnePush(int feature, bool*   hasOnePush) override;
    bool   setMode(   int feature, FeatureMode mode) override;
    bool   getMode(   int feature, FeatureMode *mode) override;
    bool   setOnePush(int feature) override;

    //IFrameGrabberImageRaw
    bool getImage(yarp::sig::ImageOf<yarp::sig::PixelMono>& image) override;
    int height() const override;
    int width() const override;

protected:
    //method
    inline bool initializeRealsenseDevice();
    inline bool setParams();

    bool        getImage(FlexImage& Frame, Stamp* timeStamp, rs2::frameset& sourceFrame);
    bool        getImage(depthImage& Frame, Stamp* timeStamp, const rs2::frameset& sourceFrame);
    void        updateTransformations();
    bool        pipelineStartup();
    bool        pipelineShutdown();
    bool        pipelineRestart();
    bool        setFramerate(const int _fps);
    void        fallback();
    bool        setPreset(rs2_rs400_visual_preset preset);


    // realsense classes
    mutable std::mutex m_mutex;
    rs2::context m_ctx;
    rs2::config m_cfg;
    rs2::pipeline m_pipeline;
    rs2::pipeline_profile m_profile;
    rs2::device  m_device;
    std::vector<rs2::sensor> m_sensors;
    rs2::sensor* m_depth_sensor;
    rs2::sensor* m_color_sensor;
    rs2_intrinsics m_depth_intrin{}, m_color_intrin{}, m_infrared_intrin{};
    rs2_extrinsics m_depth_to_color{}, m_color_to_depth{};
    rs2_stream  m_alignment_stream{RS2_STREAM_COLOR};


    // Data quantization related parameters
    bool                             m_depthQuantizationEnabled{false};
    int                              m_depthDecimalNum{0};

    yarp::os::Stamp m_rgb_stamp;
    yarp::os::Stamp m_depth_stamp;
    mutable std::string m_lastError;
    yarp::dev::RGBDSensorParamParser m_paramParser;
    bool m_verbose;
    bool m_initialized;
    bool m_stereoMode;
    bool m_needAlignment;
    int m_fps;
    float m_scale;
    bool m_rotateImage180{false};
    std::vector<cameraFeature_id_t> m_supportedFeatures;
    bool m_usePreset{false};
};
#endif
