// Copyright (c) 2020, Sherif Nekkah 
// All rights reserved. 
// 
// DISCLAMER:
//
//
// This package was created and used within an academic project and should
// be considered as experimental code. There may be bugs and deficiencies in the
// software. Feel free for suggestions, pull requests or any possible issue. 
//
//
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met: 
// 
//  * Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer. 
//  * Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the distribution. 
//  * Neither the name of  nor the names of its contributors may be used to 
//    endorse or promote products derived from this software without specific 
//    prior written permission. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE. 



#pragma once

#include <iostream>
#include <atomic>

//ROS Headers
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse.h>

//OpenCV Headers
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

//IDS Camera Headers
#include <peak_ipl/peak_ipl.hpp>
#include <peak/converters/peak_buffer_converter_ipl.hpp>
#include <peak/peak.hpp>

//AutoFeaturesManager
#include "autofeaturesmanager.h"

//Parameters
#include "acquisition_parameters.hpp"

#include <peak_cam/PeakCamConfig.h>


namespace peak_cam
{

class Peak_Cam
{
    using Config = PeakCamConfig;
    using ReconfigureServer = dynamic_reconfigure::Server<Config>;

public:
    Peak_Cam(ros::NodeHandle nh);
    ~Peak_Cam();
    
    // acquisitionLoop function and bool are public to run on particular thread
    void acquisitionLoop();
    
    // Preventing two threads to access variable acquisitionLoop_running
    std::atomic<bool> acquisitionLoop_running{false};

private:
    ros::NodeHandle nh_private;

    void reconfigureRequest(const Config &, uint32_t);
    void openDevice();
    void setDeviceParameters();
    void closeDevice();
    bool setCamInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp);
    bool saveIntrinsicsFile();

    image_transport::CameraPublisher pub_image_transport;
    sensor_msgs::Image ros_image_;
    sensor_msgs::CameraInfo ros_cam_info_;
    unsigned int ros_frame_count_;
    std::string camera_topic_;
    std::string cam_name_;
    std::string frame_name_;
    std::string cam_intr_filename_;

    ros::ServiceServer set_cam_info_srv_;

    dynamic_reconfigure::Server<Config> server;
    dynamic_reconfigure::Server<Config>::CallbackType f;

    std::shared_ptr<peak::core::DataStream> m_dataStream;
    std::shared_ptr<peak::core::Device> m_device;
    std::shared_ptr<peak::core::NodeMap> m_nodeMapRemoteDevice;
    peak::ipl::PixelFormatName pixel_format_name;
    peak::ipl::ImageTransformer m_imageTransformerIPL;
    sensor_msgs::Image image_for_encoding;

    // Manager for executing Auto features (Gain/Exposure/WhiteBalance)
    peak::ipl::Gain m_gainControllerIPL;
    peak::ipl::GammaCorrector m_gammaControllerIPL;
    AutoFeaturesManager m_autoFeaturesManager;
    bool m_gainAutoSupported {false};
    bool m_balanceWhiteAutoSupported {false};
    bool m_hostColorGainsEnabled {false};
    bool m_exposureAutoSupported {false};

    // Camera Parameters
    Peak_Params peak_params;
};

} // namespace peak_cam
