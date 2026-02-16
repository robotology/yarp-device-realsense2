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


public:
    realsense2Driver();
    ~realsense2Driver() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IRGBDSensor
    int    getRgbHeight() override;
    int    getRgbWidth() override;
    yarp::dev::ReturnValue   getRgbSupportedConfigurations(std::vector<yarp::dev::CameraConfig> &configurations) override;
    yarp::dev::ReturnValue   getRgbResolution(int &width, int &height) override;
    yarp::dev::ReturnValue   setRgbResolution(int width, int height) override;
    yarp::dev::ReturnValue   getRgbFOV(double& horizontalFov, double& verticalFov) override;
    yarp::dev::ReturnValue   setRgbFOV(double horizontalFov, double verticalFov) override;
    yarp::dev::ReturnValue   getRgbMirroring(bool& mirror) override;
    yarp::dev::ReturnValue   setRgbMirroring(bool mirror) override;

    yarp::dev::ReturnValue   getRgbIntrinsicParam(Property& intrinsic) override;
    int    getDepthHeight() override;
    int    getDepthWidth() override;
    yarp::dev::ReturnValue   setDepthResolution(int width, int height) override;
    yarp::dev::ReturnValue   getDepthResolution(int &width, int &height) override;
    yarp::dev::ReturnValue   getDepthFOV(double& horizontalFov, double& verticalFov) override;
    yarp::dev::ReturnValue   setDepthFOV(double horizontalFov, double verticalFov) override;
    yarp::dev::ReturnValue   getDepthIntrinsicParam(Property& intrinsic) override;
    yarp::dev::ReturnValue   getDepthAccuracy(double& accuracy) override;
    yarp::dev::ReturnValue   setDepthAccuracy(double accuracy) override;
    yarp::dev::ReturnValue   getDepthClipPlanes(double& nearPlane, double& farPlane) override;
    yarp::dev::ReturnValue   setDepthClipPlanes(double nearPlane, double farPlane) override;
    yarp::dev::ReturnValue   getDepthMirroring(bool& mirror) override;
    yarp::dev::ReturnValue   setDepthMirroring(bool mirror) override;


    yarp::dev::ReturnValue   getExtrinsicParam(yarp::sig::Matrix &extrinsic) override;
    yarp::dev::ReturnValue   getRgbImage(FlexImage& rgbImage, Stamp* timeStamp = nullptr) override;
    yarp::dev::ReturnValue   getDepthImage(depthImage& depthImage, Stamp* timeStamp = nullptr) override;
    yarp::dev::ReturnValue   getImages(FlexImage& colorFrame, depthImage& depthFrame, Stamp* colorStamp=NULL, Stamp* depthStamp=NULL) override;

    yarp::dev::ReturnValue  getSensorStatus(RGBDSensor_status& status) override;
    yarp::dev::ReturnValue  getLastErrorMsg(std::string& message, Stamp* timeStamp = NULL) override;

    //IFrameGrabberControls
    yarp::dev::ReturnValue   getCameraDescription(yarp::dev::CameraDescriptor& camera) override;
    yarp::dev::ReturnValue   hasFeature(yarp::dev::cameraFeature_id_t feature, bool&   hasFeature) override;
    yarp::dev::ReturnValue   setFeature(yarp::dev::cameraFeature_id_t feature, double  value) override;
    yarp::dev::ReturnValue   getFeature(yarp::dev::cameraFeature_id_t feature, double& value) override;
    yarp::dev::ReturnValue   setFeature(yarp::dev::cameraFeature_id_t feature, double  value1,  double  value2) override;
    yarp::dev::ReturnValue   getFeature(yarp::dev::cameraFeature_id_t feature, double& value1,  double& value2) override;
    yarp::dev::ReturnValue   hasOnOff(  yarp::dev::cameraFeature_id_t feature, bool&   HasOnOff) override;
    yarp::dev::ReturnValue   setActive( yarp::dev::cameraFeature_id_t feature, bool    onoff) override;
    yarp::dev::ReturnValue   getActive( yarp::dev::cameraFeature_id_t feature, bool&   isActive) override;
    yarp::dev::ReturnValue   hasAuto(   yarp::dev::cameraFeature_id_t feature, bool&   hasAuto) override;
    yarp::dev::ReturnValue   hasManual( yarp::dev::cameraFeature_id_t feature, bool&   hasManual) override;
    yarp::dev::ReturnValue   hasOnePush(yarp::dev::cameraFeature_id_t feature, bool&   hasOnePush) override;
    yarp::dev::ReturnValue   setMode(   yarp::dev::cameraFeature_id_t feature, yarp::dev::FeatureMode mode) override;
    yarp::dev::ReturnValue   getMode(   yarp::dev::cameraFeature_id_t feature, yarp::dev::FeatureMode& mode) override;
    yarp::dev::ReturnValue   setOnePush(yarp::dev::cameraFeature_id_t feature) override;

    //IFrameGrabberImageRaw
    yarp::dev::ReturnValue getImage(yarp::sig::ImageOf<yarp::sig::PixelMono>& image) override;
    int height() const override;
    int width() const override;

protected:
    //method
    inline bool initializeRealsenseDevice();
    inline bool setParams();

    bool        getImage_priv(FlexImage& Frame, Stamp* timeStamp, rs2::frameset& sourceFrame);
    bool        getImage_priv(depthImage& Frame, Stamp* timeStamp, const rs2::frameset& sourceFrame);
    void        updateTransformations();
    bool        pipelineStartup();
    bool        pipelineShutdown();
    bool        pipelineRestart();
    bool        setFramerate(const int _fps);
    void        fallback();


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
    std::vector<yarp::dev::cameraFeature_id_t> m_supportedFeatures;
};
#endif
