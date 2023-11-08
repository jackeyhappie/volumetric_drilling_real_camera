//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2022, AMBF
    (https://github.com/WPI-AIM/ambf)
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
    \author    <amunawar@jhu.edu>
    \author    Adnan Munawar
*/
//=============================================================================

// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <chai3d.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <diagnostic_msgs/KeyValue.h>

using namespace std;
using namespace ambf;


class afCameraHMD: public afObjectPlugin{
public:
    afCameraHMD();
    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual void reset() override;
    virtual bool close() override;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
    void numberCallback(const std_msgs::Int32::ConstPtr& msg);
    void stringCallback(const std_msgs::String::ConstPtr& msg);
    void floatCallback1(const std_msgs::Float32::ConstPtr& msg);
    void floatCallback2(const std_msgs::Float32::ConstPtr& msg);
    void floatCallback3(const std_msgs::Float32::ConstPtr& msg);
    void floatCallback4(const std_msgs::Float32::ConstPtr& msg);
    void KVCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg);

    // void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    // void imageCallback2(const sensor_msgs::CompressedImage::ConstPtr& msg);
    

    void updateHMDParams();

    void makeFullScreen();

protected:
    afCameraPtr m_camera;
    cFrameBufferPtr m_frameBuffer;
    cWorld* m_vrWorld;
    cMesh* m_quadMesh;
    int m_width;
    int m_height;
    int m_alias_scaling;
    cShaderProgramPtr m_shaderPgm;

    ////////////////////////////////////
    ros::NodeHandle NodeHandle_;

protected:
    float m_viewport_scale[2];
    float m_distortion_coeffs[4];
    float m_aberr_scale[3];
    float m_sep;
    float m_left_lens_center[2];
    float m_right_lens_center[2];
    float m_warp_scale;
    float m_warp_adj;
    float m_vpos;

    float m_distortion_coeffs2[4];
    float m_aberr_scale2[3];
    float m_left_lens_center2[2];
    float m_right_lens_center2[2];
    float m_vpos2;

    ros::NodeHandle* m_rosNode;
    ros::NodeHandle* m_rosNode2;
    ros::NodeHandle* mode_rosNode;
    ros::NodeHandle* para_rosNode1;
    ros::NodeHandle* para_rosNode2;
    ros::NodeHandle* para_rosNode3;
    ros::NodeHandle* para_rosNode4;
    ros::Subscriber sub, sub2, sub3, sub4, sub5, sub6, sub7;
    cv_bridge::CvImagePtr cv_ptr, cv_ptr2;
    // cv::Mat frame, frame2;
    cTexture2dPtr m_rosImageTexture;
    int clipsize=0.3;
    int mode=2;
};


AF_REGISTER_OBJECT_PLUGIN(afCameraHMD)
