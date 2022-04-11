/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <cmath>
#include <algorithm>
#include <iomanip>
#include <cstdint>

#include <yarp/os/LogComponent.h>
#include <yarp/os/Value.h>
#include <yarp/sig/ImageUtils.h>

#include <librealsense2/rsutil.h>
#include "realsense2Driver.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

using namespace std;

namespace {
YARP_LOG_COMPONENT(REALSENSE2, "yarp.device.realsense2")
}


constexpr char accuracy       [] = "accuracy";
constexpr char clipPlanes     [] = "clipPlanes";
constexpr char depthRes       [] = "depthResolution";
constexpr char rgbRes         [] = "rgbResolution";
constexpr char framerate      [] = "framerate";
constexpr char enableEmitter  [] = "enableEmitter";
constexpr char needAlignment  [] = "needAlignment";
constexpr char alignmentFrame [] = "alignmentFrame";


static std::map<std::string, RGBDSensorParamParser::RGBDParam> params_map =
{
    {accuracy,       RGBDSensorParamParser::RGBDParam(accuracy,        1)},
    {clipPlanes,     RGBDSensorParamParser::RGBDParam(clipPlanes,      2)},
    {depthRes,       RGBDSensorParamParser::RGBDParam(depthRes,        2)},
    {rgbRes,         RGBDSensorParamParser::RGBDParam(rgbRes,          2)},
    {framerate,      RGBDSensorParamParser::RGBDParam(framerate,       1)},
    {enableEmitter,  RGBDSensorParamParser::RGBDParam(enableEmitter,   1)},
    {needAlignment,  RGBDSensorParamParser::RGBDParam(needAlignment,   1)},
    {alignmentFrame, RGBDSensorParamParser::RGBDParam(alignmentFrame,  1)}
};

static const std::map<std::string, rs2_stream> stringRSStreamMap {
    {"None",  RS2_STREAM_ANY},
    {"RGB",  RS2_STREAM_COLOR},
    {"Depth", RS2_STREAM_DEPTH}
};


static std::string get_device_information(const rs2::device& dev)
{

    std::stringstream ss;
    ss << "Device information: " << std::endl;

    for (int i = 0; i < static_cast<int>(RS2_CAMERA_INFO_COUNT); i++)
    {
        auto info_type = static_cast<rs2_camera_info>(i);
        ss << "  " << std::left << std::setw(20) << info_type << " : ";

        if (dev.supports(info_type))
            ss << dev.get_info(info_type) << std::endl;
        else
            ss << "N/A" << std::endl;
    }
    return ss.str();
}


static void print_supported_options(const rs2::sensor& sensor)
{
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    if (sensor.is<rs2::depth_sensor>())
        yCInfo(REALSENSE2) << "Depth sensor supports the following options:\n";
    else if (sensor.get_stream_profiles()[0].stream_type() == RS2_STREAM_COLOR)
        yCInfo(REALSENSE2) << "RGB camera supports the following options:\n";
    else
        yCInfo(REALSENSE2) << "Sensor supports the following options:\n";

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        auto option_type = static_cast<rs2_option>(i);
        //SDK enum types can be streamed to get a string that represents them

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << "  " << option_type;
            std::cout << std::endl;

            // Get a human readable description of the option
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;
        }
    }

    std::cout<<std::endl;
}

static bool isSupportedFormat(const rs2::sensor &sensor, const int width, const int height, const int fps, bool verbose = false)
{
    bool ret = false;
    std::vector<rs2::stream_profile> stream_profiles = sensor.get_stream_profiles();

    std::map<std::pair<rs2_stream, int>, int> unique_streams;
    for (auto&& sp : stream_profiles)
    {
        unique_streams[std::make_pair(sp.stream_type(), sp.stream_index())]++;
    }

    if (verbose)
    {
        std::cout << "Sensor consists of " << unique_streams.size() << " streams: " << std::endl;
        for (size_t i = 0; i < unique_streams.size(); i++)
        {
            auto it = unique_streams.begin();
            std::advance(it, i);
            std::cout << "  - " << it->first.first << " #" << it->first.second << std::endl;
        }
        std::cout << "Sensor provides the following stream profiles:" << std::endl;
    }

    //Next, we go over all the stream profiles and print the details of each one

    int profile_num = 0;
    for (const rs2::stream_profile& stream_profile : stream_profiles)
    {
        if (verbose)
        {
            rs2_stream stream_data_type = stream_profile.stream_type();

            int stream_index = stream_profile.stream_index();

            std::cout << std::setw(3) << profile_num << ": " << stream_data_type << " #" << stream_index;
        }

        // As noted, a stream is an abstraction.
        // In order to get additional data for the specific type of a
        //  stream, a mechanism of "Is" and "As" is provided:
        if (stream_profile.is<rs2::video_stream_profile>()) //"Is" will test if the type tested is of the type given
        {
            // "As" will try to convert the instance to the given type
            rs2::video_stream_profile video_stream_profile = stream_profile.as<rs2::video_stream_profile>();

            // After using the "as" method we can use the new data type
            //  for additinal operations:
            if (verbose)
            {
                std::cout << " (Video Stream: " << video_stream_profile.format() << " " <<
                video_stream_profile.width() << "x" << video_stream_profile.height() << "@ " << video_stream_profile.fps() << "Hz)";
                std::cout << std::endl;
            }

            if(video_stream_profile.width() == width && video_stream_profile.height() == height && video_stream_profile.fps() == fps)
                ret=true;
        }
        profile_num++;
    }


    return ret;
}

static bool optionPerc2Value(rs2_option option,const rs2::sensor* sensor, const float& perc, float& value)
{
    if (!sensor)
    {
        return false;
    }
    try
    {
        rs2::option_range optionRange = sensor->get_option_range(option);
        value =(float) (perc * (optionRange.max - optionRange.min) + optionRange.min);

    }
    catch (const rs2::error& e)
    {
        // Some options can only be set while the camera is streaming,
        // and generally the hardware might fail so it is good practice to catch exceptions from set_option
        yCError(REALSENSE2) << "Failed to get option " << option << " range. (" << e.what() << ")";
        return false;
    }

    return true;
}

static bool optionValue2Perc(rs2_option option,const rs2::sensor* sensor, float& perc, const float& value)
{
    if (!sensor)
    {
        return false;
    }
    try
    {
        rs2::option_range optionRange = sensor->get_option_range(option);
        perc =(float) ((value - optionRange.min) /  (optionRange.max - optionRange.min));

    }
    catch (const rs2::error& e)
    {
        // Some options can only be set while the camera is streaming,
        // and generally the hardware might fail so it is good practice to catch exceptions from set_option
        yCError(REALSENSE2) << "Failed to get option " << option << " range. (" << e.what() << ")";
        return false;
    }

    return true;
}



static bool setOption(rs2_option option,const rs2::sensor* sensor, float value)
{

    if (!sensor)
    {
        return false;
    }

    // First, verify that the sensor actually supports this option
    if (!sensor->supports(option))
    {
        yCError(REALSENSE2) << "The option" << rs2_option_to_string(option) << "is not supported by this sensor";
        return false;
    }

    // To set an option to a different value, we can call set_option with a new value
    try
    {
        sensor->set_option(option, value);
    }
    catch (const rs2::error& e)
    {
        // Some options can only be set while the camera is streaming,
        // and generally the hardware might fail so it is good practice to catch exceptions from set_option
        yCError(REALSENSE2) << "Failed to set option " << rs2_option_to_string(option) << ". (" << e.what() << ")";
        return false;
    }
    return true;
}

static bool getOption(rs2_option option,const rs2::sensor *sensor, float &value)
{
    if (!sensor)
    {
        return false;
    }

    // First, verify that the sensor actually supports this option
    if (!sensor->supports(option))
    {
        yCError(REALSENSE2) << "The option" << rs2_option_to_string(option) << "is not supported by this sensor";
        return false;
    }

    // To set an option to a different value, we can call set_option with a new value
    try
    {
        value = sensor->get_option(option);
    }
    catch (const rs2::error& e)
    {
        // Some options can only be set while the camera is streaming,
        // and generally the hardware might fail so it is good practice to catch exceptions from set_option
        yCError(REALSENSE2) << "Failed to get option " << rs2_option_to_string(option) << ". (" << e.what() << ")";
        return false;
    }
    return true;
}

static int pixFormatToCode(const rs2_format p)
{
    switch(p)
    {
    case (RS2_FORMAT_RGB8):
        return VOCAB_PIXEL_RGB;

    case (RS2_FORMAT_BGR8):
        return VOCAB_PIXEL_BGR;

    case (RS2_FORMAT_Z16):
        return VOCAB_PIXEL_MONO16;

    case (RS2_FORMAT_DISPARITY16):
        return VOCAB_PIXEL_MONO16;

    case (RS2_FORMAT_RGBA8):
        return VOCAB_PIXEL_RGBA;

    case (RS2_FORMAT_BGRA8):
        return VOCAB_PIXEL_BGRA;

    case (RS2_FORMAT_Y8):
        return VOCAB_PIXEL_MONO;

    case (RS2_FORMAT_Y16):
        return VOCAB_PIXEL_MONO16;;

    case (RS2_FORMAT_RAW16):
        return VOCAB_PIXEL_MONO16;

    case (RS2_FORMAT_RAW8):
        return VOCAB_PIXEL_MONO;
    default:
        return VOCAB_PIXEL_INVALID;

    }
}

static size_t bytesPerPixel(const rs2_format format)
{
    size_t bytes_per_pixel = 0;
    switch (format)
    {
    case RS2_FORMAT_RAW8:
    case RS2_FORMAT_Y8:
        bytes_per_pixel = 1;
        break;
    case RS2_FORMAT_Z16:
    case RS2_FORMAT_DISPARITY16:
    case RS2_FORMAT_Y16:
    case RS2_FORMAT_RAW16:
        bytes_per_pixel = 2;
        break;
    case RS2_FORMAT_RGB8:
    case RS2_FORMAT_BGR8:
        bytes_per_pixel = 3;
        break;
    case RS2_FORMAT_RGBA8:
    case RS2_FORMAT_BGRA8:
        bytes_per_pixel = 4;
        break;
    default:
        break;
    }
    return bytes_per_pixel;
}

static YarpDistortion rsDistToYarpDist(const rs2_distortion dist, const rs2_intrinsics &values)
{
    switch (dist)
    {
    case RS2_DISTORTION_BROWN_CONRADY:
        return YarpDistortion::YARP_PLUMB_BOB;
    default:
        /*
         * If the coefficient are all zero the image is undistorted. For now is set to plumb bob since all the devices that uses it are configured with plumb bob.
         * An issue will be open to fix this bug and set it to undistorted.
         */
        if (values.coeffs[0] == 0.0 && values.coeffs[1] == 0.0 && values.coeffs[2] == 0.0 && values.coeffs[3] == 0.0 && values.coeffs[4] == 0.0)
        {
            return YarpDistortion::YARP_DISTORTION_NONE;
        }
        else
        {
            return YarpDistortion::YARP_UNSUPPORTED;
        }
    }
}

static void settingErrorMsg(const string& error, bool& ret)
{
    yCError(REALSENSE2) << error.c_str();
    ret = false;
}

static bool setIntrinsic(Property& intrinsic, const rs2_intrinsics &values)
{
    yarp::sig::IntrinsicParams params;
    params.focalLengthX       = values.fx;
    params.focalLengthY       = values.fy;
    params.principalPointX    = values.ppx;
    params.principalPointY    = values.ppy;
    // distortion model
    params.distortionModel.type = rsDistToYarpDist(values.model, values);
    params.distortionModel.k1 = values.coeffs[0];
    params.distortionModel.k2 = values.coeffs[1];
    params.distortionModel.t1 = values.coeffs[2];
    params.distortionModel.t2 = values.coeffs[3];
    params.distortionModel.k3 = values.coeffs[4];
    params.toProperty(intrinsic);
    return true;
}

static bool setExtrinsicParam(Matrix &extrinsic, const rs2_extrinsics &values)
{

    if (extrinsic.cols() != 4 || extrinsic.rows() != 4)
    {
        yCError(REALSENSE2) << "Extrinsic matrix is not 4x4";
        return false;
    }

    extrinsic.eye();

    for (size_t j=0; j<extrinsic.rows() - 1; j++)
    {
        for (size_t i=0; i<extrinsic.cols() - 1; i++)
        {
            extrinsic[j][i] = values.rotation[i + j*extrinsic.cols()];
        }
    }

    extrinsic[0][3] = values.translation[0];
    extrinsic[1][3] = values.translation[1];
    extrinsic[2][3] = values.translation[2];

    return false;
}

realsense2Driver::realsense2Driver() : m_depth_sensor(nullptr), m_color_sensor(nullptr),
                                       m_paramParser(), m_verbose(false),
                                       m_initialized(false), m_stereoMode(false),
                                       m_needAlignment(true), m_fps(0),
                                       m_scale(0.0)
{
    // realsense SDK already provides them
    m_paramParser.depthIntrinsic.isOptional = true;
    m_paramParser.rgbIntrinsic.isOptional   = true;
    m_paramParser.isOptionalExtrinsic       = true;


    m_supportedFeatures.push_back(YARP_FEATURE_EXPOSURE);
    m_supportedFeatures.push_back(YARP_FEATURE_WHITE_BALANCE);
    m_supportedFeatures.push_back(YARP_FEATURE_GAIN);
    m_supportedFeatures.push_back(YARP_FEATURE_FRAME_RATE);
    m_supportedFeatures.push_back(YARP_FEATURE_SHARPNESS);
    m_supportedFeatures.push_back(YARP_FEATURE_HUE);
    m_supportedFeatures.push_back(YARP_FEATURE_SATURATION);
}

bool realsense2Driver::pipelineStartup()
{
    try
    {
        m_profile = m_pipeline.start(m_cfg);
    }
    catch (const rs2::error& e)
    {
        yCError(REALSENSE2) << "Failed to start the pipeline:"<< "(" << e.what() << ")";
        m_lastError = e.what();
        return false;
    }
    return true;
}

bool realsense2Driver::pipelineShutdown()
{
    try
    {
        m_pipeline.stop();
    }
    catch (const rs2::error& e)
    {
        yCError(REALSENSE2) << "Failed to stop the pipeline:"<< "(" << e.what() << ")";
        m_lastError = e.what();
        return false;
    }
    return true;
}

bool realsense2Driver::pipelineRestart()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!pipelineShutdown())
        return false;

    return pipelineStartup();

}

bool realsense2Driver::setFramerate(const int _fps)
{
    if (m_color_sensor && isSupportedFormat(*m_color_sensor,m_color_intrin.width, m_color_intrin.height, _fps, m_verbose) &&
        m_depth_sensor && isSupportedFormat(*m_depth_sensor,m_depth_intrin.width, m_depth_intrin.height, _fps, m_verbose)) {

        m_cfg.enable_stream(RS2_STREAM_COLOR, m_color_intrin.width, m_color_intrin.height, RS2_FORMAT_RGB8, _fps);
        m_cfg.enable_stream(RS2_STREAM_DEPTH, m_depth_intrin.width, m_depth_intrin.height, RS2_FORMAT_Z16, _fps);
    }
    else
    {
        if (m_initialized)
        {
            fallback();
        }

    }

    if (!pipelineRestart())
        return false;

    m_fps = _fps;

    updateTransformations();

    return true;

}

void realsense2Driver::fallback()
{
    m_cfg.enable_stream(RS2_STREAM_COLOR, m_color_intrin.width, m_color_intrin.height, RS2_FORMAT_RGB8, m_fps);
    m_cfg.enable_stream(RS2_STREAM_DEPTH, m_depth_intrin.width, m_depth_intrin.height, RS2_FORMAT_Z16, m_fps);
    yCWarning(REALSENSE2)<<"Format not supported, use --verbose for more details. Setting the fallback format";
    std::cout<<"COLOR: "<<m_color_intrin.width<<"x"<<m_color_intrin.height<<" fps: "<<m_fps<<std::endl;
    std::cout<<"DEPTH: "<<m_depth_intrin.width<<"x"<<m_depth_intrin.height<<" fps: "<<m_fps<<std::endl;
}

bool realsense2Driver::initializeRealsenseDevice()
{
    if (!params_map[rgbRes].isSetting || !params_map[depthRes].isSetting)
    {
        yCError(REALSENSE2)<<"Missing depthResolution or rgbResolution from [SETTINGS]";
        return false;
    }
    double colorW = params_map[rgbRes].val[0].asFloat64();
    double colorH = params_map[rgbRes].val[1].asFloat64();
    double depthW = params_map[depthRes].val[0].asFloat64();
    double depthH = params_map[depthRes].val[1].asFloat64();

    m_cfg.enable_stream(RS2_STREAM_COLOR, colorW, colorH, RS2_FORMAT_RGB8, m_fps);
    m_cfg.enable_stream(RS2_STREAM_DEPTH, depthW, depthH, RS2_FORMAT_Z16, m_fps);
    if (m_stereoMode) {
        m_cfg.enable_stream(RS2_STREAM_INFRARED, 1, colorW, colorH, RS2_FORMAT_Y8, m_fps);
        m_cfg.enable_stream(RS2_STREAM_INFRARED, 2, colorW, colorH, RS2_FORMAT_Y8, m_fps);
    }
    if (!pipelineStartup())
        return false;
    m_initialized = true;

    // Camera warmup - Dropped frames to allow stabilization
    yCInfo(REALSENSE2) << "Sensor warm-up...";
    for (int i = 0; i < 30; i++)
    {
        try
        {
            m_pipeline.wait_for_frames();
        }
        catch (const rs2::error& e)
        {
            yCError(REALSENSE2) << "m_pipeline.wait_for_frames() failed with error:"<< "(" << e.what() << ")";
            m_lastError = e.what();
        }
    }
    yCInfo(REALSENSE2) << "Device ready!";

    if (m_ctx.query_devices().size() == 0)
    {
        yCError(REALSENSE2) << "No device connected, please connect a RealSense device";

        rs2::device_hub device_hub(m_ctx);

        //Using the device_hub we can block the program until a device connects
        m_device = device_hub.wait_for_device();
    }
    else
    {
        //TODO: if more are connected?!
        // Update the selected device
        m_device = m_profile.get_device();
        if (m_verbose)
            yCInfo(REALSENSE2) << get_device_information(m_device).c_str();
    }


    // Given a device, we can query its sensors using:
    m_sensors = m_device.query_sensors();

    yCInfo(REALSENSE2) << "Device consists of" << m_sensors.size() << "sensors. More infos using --verbose option";
    if (m_verbose)
    {
        for (const auto & m_sensor : m_sensors)
        {
            print_supported_options(m_sensor);
        }
    }

    for (auto & m_sensor : m_sensors)
    {
        if (m_sensor.is<rs2::depth_sensor>())
        {
            m_depth_sensor =  &m_sensor;
            if (!getOption(RS2_OPTION_DEPTH_UNITS, m_depth_sensor, m_scale))
            {
                yCError(REALSENSE2) << "Failed to retrieve scale";
                return false;
            }
        }
        else if (m_sensor.get_stream_profiles()[0].stream_type() == RS2_STREAM_COLOR)
            m_color_sensor = &m_sensor;
    }

    // Get stream intrinsics & extrinsics
    updateTransformations();
    return true;
}

void realsense2Driver::updateTransformations()
{
    rs2::pipeline_profile pipeline_profile = m_pipeline.get_active_profile();
    rs2::video_stream_profile depth_stream_profile = rs2::video_stream_profile(pipeline_profile.get_stream(RS2_STREAM_DEPTH));
    rs2::video_stream_profile color_stream_profile = rs2::video_stream_profile(pipeline_profile.get_stream(RS2_STREAM_COLOR));

    m_depth_intrin = depth_stream_profile.get_intrinsics();
    m_color_intrin = color_stream_profile.get_intrinsics();
    m_depth_to_color = depth_stream_profile.get_extrinsics_to(color_stream_profile);
    m_color_to_depth = color_stream_profile.get_extrinsics_to(depth_stream_profile);

    if (m_stereoMode) {
        rs2::video_stream_profile infrared_stream_profile = rs2::video_stream_profile(pipeline_profile.get_stream(RS2_STREAM_INFRARED));
        m_infrared_intrin = infrared_stream_profile.get_intrinsics();
    }

}


bool realsense2Driver::setParams()
{
    bool ret = true;
    //ACCURACY
    if (params_map[accuracy].isSetting && ret)
    {
        if (!params_map[accuracy].val[0].isFloat64() )
            settingErrorMsg("Param " + params_map[accuracy].name + " is not a double as it should be.", ret);

        if (! setDepthAccuracy(params_map[accuracy].val[0].asFloat64() ) )
            settingErrorMsg("Setting param " + params_map[accuracy].name + " failed... quitting.", ret);
    }

    //CLIP_PLANES
    if (params_map[clipPlanes].isSetting && ret)
    {
        if (!params_map[clipPlanes].val[0].isFloat64() )
            settingErrorMsg("Param " + params_map[clipPlanes].name + " is not a double as it should be.", ret);

        if (!params_map[clipPlanes].val[1].isFloat64() )
            settingErrorMsg("Param " + params_map[clipPlanes].name + " is not a double as it should be.", ret);

        if (! setDepthClipPlanes(params_map[clipPlanes].val[0].asFloat64(), params_map[clipPlanes].val[1].asFloat64() ) )
            settingErrorMsg("Setting param " + params_map[clipPlanes].name + " failed... quitting.", ret);
    }

    //FRAMERATE
    if (params_map[framerate].isSetting && ret)
    {
        if (!params_map[framerate].val[0].isInt32() )
            settingErrorMsg("Param " + params_map[framerate].name + " is not a int as it should be.", ret);
        else
            m_fps = params_map[framerate].val[0].asInt32();
    }
    else
    {
        yCWarning(REALSENSE2) << "Framerate not specified... setting 30 fps by default";
        m_fps = 30;
    }

    //EMITTER
    if (params_map[enableEmitter].isSetting && ret)
    {
        Value& v = params_map[enableEmitter].val[0];

        if (!v.isBool())
        {
            settingErrorMsg("Param " + params_map[enableEmitter].name + " is not a bool as it should be.", ret);
            return false;
        }
        if(!setOption(RS2_OPTION_EMITTER_ENABLED, m_depth_sensor, (float) v.asBool()))
        {
            settingErrorMsg("Setting param " + params_map[enableEmitter].name + " failed... quitting.", ret);
        }
    }

    //ALIGNMENT
    if (params_map[needAlignment].isSetting && ret)
    {
        yCWarning(REALSENSE2) << "needAlignment parameter is deprecated, use alignmentFrame instead.";
        Value& v = params_map[needAlignment].val[0];
        if (!v.isBool())
        {
            settingErrorMsg("Param " + params_map[needAlignment].name + " is not a bool as it should be.", ret);
            return false;
        }

        m_needAlignment = v.asBool();
        m_alignment_stream = m_needAlignment ? RS2_STREAM_COLOR : RS2_STREAM_ANY;
    }

    if (params_map[alignmentFrame].isSetting && ret)
    {
        Value& v = params_map[alignmentFrame].val[0];
        auto alignmentFrameStr = v.asString();
        if (!v.isString())
        {
            settingErrorMsg("Param " + params_map[alignmentFrame].name + " is not a string as it should be.", ret);
            return false;
        }

        if (stringRSStreamMap.find(alignmentFrameStr) == stringRSStreamMap.end()) {
            settingErrorMsg("Value "+alignmentFrameStr+" not allowed for " + params_map[alignmentFrame].name + " see documentation for supported values.", ret);
            return false;
        }

        m_alignment_stream = stringRSStreamMap.at(alignmentFrameStr);
    }

    //DEPTH_RES
    if (params_map[depthRes].isSetting && ret)
    {
        Value p1, p2;
        p1 = params_map[depthRes].val[0];
        p2 = params_map[depthRes].val[1];

        if (!p1.isInt32() || !p2.isInt32() )
        {
            settingErrorMsg("Param " + params_map[depthRes].name + " is not a int as it should be.", ret);
        }

        if (! setDepthResolution(p1.asInt32(), p2.asInt32()))
        {
            settingErrorMsg("Setting param " + params_map[depthRes].name + " failed... quitting.", ret);
        }
    }

    //RGB_RES
    if (params_map[rgbRes].isSetting && ret)
    {
        Value p1, p2;
        p1 = params_map[rgbRes].val[0];
        p2 = params_map[rgbRes].val[1];

        if (!p1.isInt32() || !p2.isInt32() )
        {
            settingErrorMsg("Param " + params_map[rgbRes].name + " is not a int as it should be.", ret);
        }

        if (! setRgbResolution(p1.asInt32(), p2.asInt32()))
        {
            settingErrorMsg("Setting param " + params_map[rgbRes].name + " failed... quitting.", ret);
        }
    }

    return ret;
}


bool realsense2Driver::open(Searchable& config)
{
    std::vector<RGBDSensorParamParser::RGBDParam*> params;
    params.reserve(params_map.size());
    for (auto& p:params_map)
    {
        params.push_back(&(p.second));
    }

    if(config.check("rotateImage180")){
        m_rotateImage180 = config.find("rotateImage180").asBool();
        if (m_rotateImage180) {
            yCInfo(REALSENSE2) << "parameter rotateImage180 enabled, the image is rotated";
        }
    }
    m_verbose = config.check("verbose");
    if (config.check("stereoMode")) {
        m_stereoMode = config.find("stereoMode").asBool();
    }

    if (!m_paramParser.parseParam(config, params))
    {
        yCError(REALSENSE2) << "Failed to parse the parameters";
        return false;
    }

    if (!initializeRealsenseDevice())
    {
        yCError(REALSENSE2) << "Failed to initialize the realsense device";
        return false;
    }

    // setting Parameters
    return setParams();

}

bool realsense2Driver::close()
{
    pipelineShutdown();
    return true;
}

int realsense2Driver::getRgbHeight()
{
    return m_color_intrin.height;
}

int realsense2Driver::getRgbWidth()
{
    return m_color_intrin.width;
}

bool realsense2Driver::getRgbSupportedConfigurations(yarp::sig::VectorOf<CameraConfig> &configurations)
{
    yCWarning(REALSENSE2) << "getRgbSupportedConfigurations not implemented yet";
    return false;
}

bool realsense2Driver::getRgbResolution(int &width, int &height)
{
    width  = m_color_intrin.width;
    height = m_color_intrin.height;
    return true;
}

bool realsense2Driver::setDepthResolution(int width, int height)
{
    if (m_depth_sensor && isSupportedFormat(*m_depth_sensor, width, height, m_fps, m_verbose))
    {
        m_cfg.enable_stream(RS2_STREAM_COLOR, m_color_intrin.width, m_color_intrin.height, RS2_FORMAT_RGB8, m_fps);
        m_cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, m_fps);
    }
    else
    {
        if (m_initialized)
        {
            fallback();
            return false;
        }
    }

    if (!pipelineRestart())
        return false;

    updateTransformations();
    return true;
}

bool realsense2Driver::setRgbResolution(int width, int height)
{
    bool fail = true;
    if (m_color_sensor && isSupportedFormat(*m_color_sensor, width, height, m_fps, m_verbose)) {
        m_cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, m_fps);
        m_cfg.enable_stream(RS2_STREAM_DEPTH, m_depth_intrin.width, m_depth_intrin.height, RS2_FORMAT_Z16, m_fps);
        fail = false;
        if (m_stereoMode)
        {
            if (m_depth_sensor && isSupportedFormat(*m_depth_sensor, width, height, m_fps, m_verbose))
            {
                m_cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, m_fps);
                m_cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, m_fps);
            }
            else
            {
                fail = true;
            }
        }
    }

    if (m_initialized && fail)
    {
        fallback();
        return false;
    }

    if (!pipelineRestart())
        return false;

    updateTransformations();
    return true;
}


bool realsense2Driver::setRgbFOV(double horizontalFov, double verticalFov)
{
    // It seems to be not available...
    return false;
}

bool realsense2Driver::setDepthFOV(double horizontalFov, double verticalFov)
{
    // It seems to be not available...
    return false;
}

bool realsense2Driver::setDepthAccuracy(double accuracy)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    bool ok = setOption(RS2_OPTION_DEPTH_UNITS, m_depth_sensor, accuracy);
    if (ok) {
        m_scale = accuracy;
    }
    return ok;
}

bool realsense2Driver::getRgbFOV(double &horizontalFov, double &verticalFov)
{
    float fov[2];
    rs2_fov(&m_color_intrin, fov);
    horizontalFov = fov[0];
    verticalFov   = fov[1];
    return true;
}

bool realsense2Driver::getRgbMirroring(bool& mirror)
{
    yCWarning(REALSENSE2) << "Mirroring not supported";
    return false;
}

bool realsense2Driver::setRgbMirroring(bool mirror)
{
    yCWarning(REALSENSE2) << "Mirroring not supported";
    return false;
}

bool realsense2Driver::getRgbIntrinsicParam(Property& intrinsic)
{
    return setIntrinsic(intrinsic, m_color_intrin);
}

int  realsense2Driver::getDepthHeight()
{
    return m_depth_intrin.height;
}

int  realsense2Driver::getDepthWidth()
{
    return m_depth_intrin.width;
}

bool realsense2Driver::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    float fov[2];
    rs2_fov(&m_depth_intrin, fov);
    horizontalFov = fov[0];
    verticalFov   = fov[1];
    return true;
}

bool realsense2Driver::getDepthIntrinsicParam(Property& intrinsic)
{
    return setIntrinsic(intrinsic, m_depth_intrin);;
}

double realsense2Driver::getDepthAccuracy()
{
    float accuracy = 0.0;
    if (getOption(RS2_OPTION_DEPTH_UNITS, m_depth_sensor, accuracy))
    {
        return accuracy;
    }
    return 0.0;
}

bool realsense2Driver::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    if (params_map[clipPlanes].isDescription)
    {
        nearPlane = params_map[clipPlanes].val[0].asFloat64();
        farPlane  = params_map[clipPlanes].val[1].asFloat64();
        return true;
    }

    bool ret  = getOption(RS2_OPTION_MIN_DISTANCE, m_depth_sensor, (float&) nearPlane);
    ret &= getOption(RS2_OPTION_MAX_DISTANCE, m_depth_sensor, (float&) farPlane);
    return ret;
}

bool realsense2Driver::setDepthClipPlanes(double nearPlane, double farPlane)
{
    if (params_map[clipPlanes].isDescription)
    {
        return false;
    }
    bool ret  = setOption(RS2_OPTION_MIN_DISTANCE, m_depth_sensor, nearPlane);
    ret      &= setOption(RS2_OPTION_MAX_DISTANCE, m_depth_sensor, farPlane);
    return ret;
}

bool realsense2Driver::getDepthMirroring(bool& mirror)
{
    yCWarning(REALSENSE2) << "Mirroring not supported";
    return false;
}

bool realsense2Driver::setDepthMirroring(bool mirror)
{
    yCWarning(REALSENSE2) << "Mirroring not supported";
    return false;
}

bool realsense2Driver::getExtrinsicParam(Matrix& extrinsic)
{
    return setExtrinsicParam(extrinsic, m_depth_to_color);
}

bool realsense2Driver::getRgbImage(FlexImage& rgbImage, Stamp* timeStamp)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    rs2::frameset data;
    try
    {
        data = m_pipeline.wait_for_frames();
    }
    catch (const rs2::error& e)
    {
        yCError(REALSENSE2) << "m_pipeline.wait_for_frames() failed with error:"<< "(" << e.what() << ")";
         m_lastError = e.what();
        return false;
    }
    if (m_alignment_stream == RS2_STREAM_DEPTH)
    {
        rs2::align align(m_alignment_stream);
        data = align.process(data);
    }
    return getImage(rgbImage, timeStamp, data);
}

bool realsense2Driver::getDepthImage(ImageOf<PixelFloat>& depthImage, Stamp* timeStamp)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    rs2::frameset data;
    try
    {
        data = m_pipeline.wait_for_frames();
    }
    catch (const rs2::error& e)
    {
        yCError(REALSENSE2) << "m_pipeline.wait_for_frames() failed with error:"<< "(" << e.what() << ")";
        m_lastError = e.what();
        return false;
    }
    if (m_alignment_stream == RS2_STREAM_COLOR)
    {
        rs2::align align(m_alignment_stream);
        data = align.process(data);
    }
    return getImage(depthImage, timeStamp, data);
}

bool realsense2Driver::getImage(FlexImage& Frame, Stamp *timeStamp, rs2::frameset &sourceFrame)
{
    rs2::video_frame color_frm = sourceFrame.get_color_frame();
    rs2_format format = color_frm.get_profile().format();

    int pixCode = pixFormatToCode(format);
    size_t mem_to_wrt = color_frm.get_width() * color_frm.get_height() * bytesPerPixel(format);

    if (pixCode == VOCAB_PIXEL_INVALID)
    {
        yCError(REALSENSE2) << "Pixel Format not recognized";
        return false;
    }

    Frame.setPixelCode(pixCode);
    Frame.resize(m_color_intrin.width, m_color_intrin.height);

    if ((size_t) Frame.getRawImageSize() != mem_to_wrt)
    {
        yCError(REALSENSE2) << "Device and local copy data size doesn't match";
        return false;
    }
    if (m_rotateImage180) {
        for (int i = 0; i < (color_frm.get_width() * color_frm.get_height()); i++) {
            for (size_t pixelIndex = 0; pixelIndex < bytesPerPixel(format); pixelIndex++) {
                ((char *)Frame.getRawImage())[i * bytesPerPixel(format) + pixelIndex] = ((char *)color_frm.get_data())[
                        ( Frame.getRawImageSize() - ((i+1) * bytesPerPixel(format) ) + pixelIndex];
            }
        }
    } else {
        memcpy((void*)Frame.getRawImage(), (void*)color_frm.get_data(), mem_to_wrt);
    }
    m_rgb_stamp.update();
    if (timeStamp != nullptr)
    {
        *timeStamp = m_rgb_stamp;
    }
    return true;
}

bool realsense2Driver::getImage(depthImage& Frame, Stamp *timeStamp, const rs2::frameset &sourceFrame)
{
    rs2::depth_frame depth_frm = sourceFrame.get_depth_frame();
    rs2_format format = depth_frm.get_profile().format();

    int pixCode = pixFormatToCode(format);

    int w = depth_frm.get_width();
    int h = depth_frm.get_height();

    if (pixCode == VOCAB_PIXEL_INVALID)
    {
        yCError(REALSENSE2) << "Pixel Format not recognized";
        return false;
    }

    Frame.resize(w, h);

    float* rawImage = &Frame.pixel(0,0);
    const auto * rawImageRs =(const uint16_t *) depth_frm.get_data();
    for(int i = 0; i < w * h; i++)
    {

        if (m_rotateImage180) {
            rawImage[i] = m_scale * rawImageRs[(w * h - 1) - i];
        }else {
            rawImage[i] = m_scale * rawImageRs[i];
        }
    }

    m_depth_stamp.update();
    if (timeStamp != nullptr)
    {
        *timeStamp = m_depth_stamp;
    }
    return true;
}

bool realsense2Driver::getImages(FlexImage& colorFrame, ImageOf<PixelFloat>& depthFrame, Stamp* colorStamp, Stamp* depthStamp)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    rs2::frameset data;
    try
    {
        data = m_pipeline.wait_for_frames();
    }
    catch (const rs2::error& e)
    {
        yCError(REALSENSE2) << "m_pipeline.wait_for_frames() failed with error:"<< "(" << e.what() << ")";
        m_lastError = e.what();
        return false;
    }
    if (m_alignment_stream != RS2_STREAM_ANY) // RS2_STREAM_ANY is used as no-alignment-needed value.
    {
        rs2::align align(m_alignment_stream);
        data = align.process(data);
    }
    return getImage(colorFrame, colorStamp, data) && getImage(depthFrame, depthStamp, data);
}

IRGBDSensor::RGBDSensor_status realsense2Driver::getSensorStatus()
{
    return RGBD_SENSOR_OK_IN_USE;
}

std::string realsense2Driver::getLastErrorMsg(Stamp* timeStamp)
{
    return m_lastError;
}

bool realsense2Driver::getCameraDescription(CameraDescriptor* camera)
{
    camera->deviceDescription = get_device_information(m_device);
    camera->busType = BUS_USB;
    return true;
}

bool realsense2Driver::hasFeature(int feature, bool* hasFeature)
{
    cameraFeature_id_t f;
    f = static_cast<cameraFeature_id_t>(feature);
    if (f < YARP_FEATURE_BRIGHTNESS || f > YARP_FEATURE_NUMBER_OF-1)
    {
        return false;
    }

    *hasFeature = std::find(m_supportedFeatures.begin(), m_supportedFeatures.end(), f) != m_supportedFeatures.end();

    return true;
}

bool realsense2Driver::setFeature(int feature, double value)
{
    bool b = false;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }

    float valToSet = 0.0;
    b = false;
    auto f = static_cast<cameraFeature_id_t>(feature);
    switch(f)
    {
    case YARP_FEATURE_EXPOSURE:
        if(optionPerc2Value(RS2_OPTION_EXPOSURE, m_color_sensor, value, valToSet))
        {
            b = setOption(RS2_OPTION_EXPOSURE, m_color_sensor, valToSet);
            if (m_stereoMode)
            {
                if(optionPerc2Value(RS2_OPTION_EXPOSURE, m_depth_sensor, value, valToSet))
                {
                    b &= setOption(RS2_OPTION_EXPOSURE, m_depth_sensor, valToSet);
                }
            }
        }
        break;
    case YARP_FEATURE_GAIN:
        if(optionPerc2Value(RS2_OPTION_GAIN, m_color_sensor,value, valToSet))
        {
            b = setOption(RS2_OPTION_GAIN, m_color_sensor, valToSet);
            if (m_stereoMode)
            {
                if(optionPerc2Value(RS2_OPTION_EXPOSURE, m_depth_sensor, value, valToSet))
                {
                    b &= setOption(RS2_OPTION_EXPOSURE, m_depth_sensor, valToSet);
                }
            }
        }
        break;
    case YARP_FEATURE_FRAME_RATE:
    {
        b = setFramerate((int) value);
        break;
    }
    case YARP_FEATURE_WHITE_BALANCE:
        if(optionPerc2Value(RS2_OPTION_WHITE_BALANCE, m_color_sensor, value, valToSet))
            b = setOption(RS2_OPTION_WHITE_BALANCE, m_color_sensor, valToSet);
        break;
    case YARP_FEATURE_SHARPNESS:
        if(optionPerc2Value(RS2_OPTION_SHARPNESS, m_color_sensor, value, valToSet))
            b = setOption(RS2_OPTION_SHARPNESS, m_color_sensor, valToSet);
        break;
    case YARP_FEATURE_HUE:
        if(optionPerc2Value(RS2_OPTION_HUE, m_color_sensor, value, valToSet))
            b = setOption(RS2_OPTION_HUE, m_color_sensor, valToSet);
        break;
    case YARP_FEATURE_SATURATION:
        if(optionPerc2Value(RS2_OPTION_SATURATION, m_color_sensor, value, valToSet))
            b = setOption(RS2_OPTION_SATURATION, m_color_sensor, valToSet);
        break;
    default:
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }
    if (!b)
    {
        yCError(REALSENSE2) << "Something went wrong setting the feature requested, run the device with --verbose for the supported options";
        if (m_verbose)
        {
            print_supported_options(*m_color_sensor);
        }
        return false;
    }
    return true;
}

bool realsense2Driver::getFeature(int feature, double *value)
{
    bool b = false;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }

    float valToGet = 0.0;
    b = false;

    auto f = static_cast<cameraFeature_id_t>(feature);
    switch(f)
    {
    case YARP_FEATURE_EXPOSURE:
        if (getOption(RS2_OPTION_EXPOSURE, m_color_sensor, valToGet))
            b = optionValue2Perc(RS2_OPTION_EXPOSURE, m_color_sensor, (float&) value, valToGet);
        break;
    case YARP_FEATURE_GAIN:
        if (getOption(RS2_OPTION_GAIN, m_color_sensor, valToGet))
            b = optionValue2Perc(RS2_OPTION_GAIN, m_color_sensor, (float&) value, valToGet);
        break;
    case YARP_FEATURE_FRAME_RATE:
    {
        b = true;
        *value = (double) m_fps;
        break;
    }
    case YARP_FEATURE_WHITE_BALANCE:
        if (getOption(RS2_OPTION_WHITE_BALANCE, m_color_sensor, valToGet))
            b = optionValue2Perc(RS2_OPTION_WHITE_BALANCE, m_color_sensor, (float&) value, valToGet);
        break;
    case YARP_FEATURE_SHARPNESS:
        if (getOption(RS2_OPTION_SHARPNESS, m_color_sensor, valToGet))
            b = optionValue2Perc(RS2_OPTION_SHARPNESS, m_color_sensor, (float&) value, valToGet);
        break;
    case YARP_FEATURE_HUE:
        if (getOption(RS2_OPTION_HUE, m_color_sensor, valToGet))
            b = optionValue2Perc(RS2_OPTION_HUE, m_color_sensor, (float&) value, valToGet);
        break;
    case YARP_FEATURE_SATURATION:
        if (getOption(RS2_OPTION_SATURATION, m_color_sensor, valToGet))
            b = optionValue2Perc(RS2_OPTION_SATURATION, m_color_sensor, (float&) value, valToGet);
        break;
    default:
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }
    if (!b)
    {
        yCError(REALSENSE2) << "Something went wrong setting the feature requested, run the device with --verbose for the supported options";
        if (m_verbose)
        {
            print_supported_options(*m_color_sensor);
        }
        return false;
    }
    return true;
}

bool realsense2Driver::setFeature(int feature, double value1, double value2)
{
    yCError(REALSENSE2) << "No 2-valued feature are supported";
    return false;
}

bool realsense2Driver::getFeature(int feature, double *value1, double *value2)
{
    yCError(REALSENSE2) << "No 2-valued feature are supported";
    return false;
}

bool realsense2Driver::hasOnOff(  int feature, bool *HasOnOff)
{
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }

    auto f = static_cast<cameraFeature_id_t>(feature);
    if (f == YARP_FEATURE_WHITE_BALANCE || f == YARP_FEATURE_MIRROR || f == YARP_FEATURE_EXPOSURE)
    {
        *HasOnOff = true;
        return true;
    }
    *HasOnOff = false;
    return true;
}

bool realsense2Driver::setActive( int feature, bool onoff)
{
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }

    if (!hasOnOff(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature does not have OnOff.. call hasOnOff() to know if a specific feature support OnOff mode";
        return false;
    }

    switch(feature)
    {
    case YARP_FEATURE_WHITE_BALANCE:
        b = setOption(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, m_color_sensor, (float) onoff);
        return b;
    case YARP_FEATURE_EXPOSURE:
        b = setOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE, m_color_sensor, (float) onoff);
        return b;
    default:
        return false;
    }

    return true;
}

bool realsense2Driver::getActive( int feature, bool *isActive)
{
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }

    if (!hasOnOff(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature does not have OnOff.. call hasOnOff() to know if a specific feature support OnOff mode";
        return false;
    }
    float response = 0.0;
    switch(feature)
    {
    case YARP_FEATURE_WHITE_BALANCE:
        b = getOption(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, m_color_sensor, response); //TODO check if this exotic conversion works
        *isActive = (bool) response;
        return b;
    case YARP_FEATURE_EXPOSURE:
        b = getOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE, m_color_sensor, response); //TODO check if this exotic conversion works
        *isActive = (bool) response;
        return b;
    default:
        return false;
    }

    return true;
}

bool realsense2Driver::hasAuto(int feature, bool *hasAuto)
{
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }

    auto f = static_cast<cameraFeature_id_t>(feature);
    if (f == YARP_FEATURE_EXPOSURE || f == YARP_FEATURE_WHITE_BALANCE)
    {
        *hasAuto = true;
        return true;
    }
    *hasAuto = false;
    return true;
}

bool realsense2Driver::hasManual( int feature, bool* hasManual)
{
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }

    auto f = static_cast<cameraFeature_id_t>(feature);
    if (f == YARP_FEATURE_EXPOSURE || f == YARP_FEATURE_FRAME_RATE || f == YARP_FEATURE_GAIN ||
        f == YARP_FEATURE_HUE || f == YARP_FEATURE_SATURATION || f == YARP_FEATURE_SHARPNESS)
    {
        *hasManual = true;
        return true;
    }
    *hasManual = false;
    return true;
}

bool realsense2Driver::hasOnePush(int feature, bool* hasOnePush)
{
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }

    return hasAuto(feature, hasOnePush);
}

bool realsense2Driver::setMode(int feature, FeatureMode mode)
{
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }
    float one = 1.0;
    float zero = 0.0;

    auto f = static_cast<cameraFeature_id_t>(feature);
    if (f == YARP_FEATURE_WHITE_BALANCE)
    {
        switch(mode)
        {
        case MODE_AUTO:
            return setOption(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, m_color_sensor, one);
        case MODE_MANUAL:
            return setOption(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, m_color_sensor, zero);
        case MODE_UNKNOWN:
            return false;
        default:
            return false;
        }
        return true;
    }

    if (f == YARP_FEATURE_EXPOSURE)
    {
        switch(mode)
        {
        case MODE_AUTO:
            return setOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE, m_color_sensor, one);
        case MODE_MANUAL:
            return setOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE, m_color_sensor, zero);
        case MODE_UNKNOWN:
            return false;
        default:
            return false;
        }
        return true;
    }


    yCError(REALSENSE2) << "Feature does not have both auto and manual mode";
    return false;
}

bool realsense2Driver::getMode(int feature, FeatureMode* mode)
{
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }
    float res = 0.0;
    bool ret = true;
    auto f = static_cast<cameraFeature_id_t>(feature);
    if (f == YARP_FEATURE_WHITE_BALANCE)
    {
        ret &= getOption(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, m_color_sensor, res);
    }

    if (f == YARP_FEATURE_EXPOSURE)
    {
        ret &= getOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE, m_color_sensor, res);
    }

    if (res == 0.0)
    {
        *mode = MODE_MANUAL;
    }
    else if (res == 1.0)
    {
        *mode = MODE_AUTO;
    }
    else
    {
        *mode = MODE_UNKNOWN;
    }
    return ret;
}

bool realsense2Driver::setOnePush(int feature)
{
    bool b;
    if (!hasFeature(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature not supported!";
        return false;
    }

    if (!hasOnePush(feature, &b) || !b)
    {
        yCError(REALSENSE2) << "Feature doesn't have OnePush";
        return false;
    }

    setMode(feature, MODE_AUTO);
    setMode(feature, MODE_MANUAL);

    return true;
}

bool realsense2Driver::getImage(yarp::sig::ImageOf<yarp::sig::PixelMono>& image)
{
    if (!m_stereoMode)
    {
        yCError(REALSENSE2)<<"Infrared stereo stream not enabled";
        return false;
    }

    image.resize(width(), height());
    std::lock_guard<std::mutex> guard(m_mutex);
    rs2::frameset data;
    try
    {
        data = m_pipeline.wait_for_frames();
    }
    catch (const rs2::error& e)
    {
        yCError(REALSENSE2) << "m_pipeline.wait_for_frames() failed with error:"<< "(" << e.what() << ")";
        m_lastError = e.what();
        return false;
    }

    rs2::video_frame frm1 = data.get_infrared_frame(1);
    rs2::video_frame frm2 = data.get_infrared_frame(2);

    int pixCode = pixFormatToCode(frm1.get_profile().format());

    if (pixCode != VOCAB_PIXEL_MONO)
    {
        yCError(REALSENSE2) << "Expecting Pixel Format MONO";
        return false;
    }

    // Wrap rs images with yarp ones.
    ImageOf<PixelMono> imgL, imgR;
    imgL.setExternal((unsigned char*) (frm1.get_data()), frm1.get_width(), frm1.get_height());
    imgR.setExternal((unsigned char*) (frm2.get_data()), frm2.get_width(), frm2.get_height());
    return utils::horzConcat(imgL, imgR, image);
}

int  realsense2Driver::height() const
{
    return m_infrared_intrin.height;
}

int  realsense2Driver::width() const
{
    return m_infrared_intrin.width*2;
}
