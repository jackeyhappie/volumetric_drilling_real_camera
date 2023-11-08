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
    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \author    <pkunjam1@jhu.edu>
    \author    Punit Kunjam
*/
//=============================================================================

#include "hmd.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ambf_server/RosComBase.h>
#include <ros/console.h>



using namespace std;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

string g_current_filepath;

afCameraHMD::afCameraHMD()
{
    // For HTC Vive Pro
    m_width = 2880;
    m_height = 1600;
    m_alias_scaling = 1.0;
}

int afCameraHMD::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
       
    m_rosNode = afROSNode::getNode();
    m_rosNode2 = afROSNode::getNode();
    mode_rosNode = afROSNode::getNode();
    para_rosNode1 = afROSNode::getNode();
    para_rosNode2 = afROSNode::getNode();
    para_rosNode3 = afROSNode::getNode();
    para_rosNode4 = afROSNode::getNode();

    // sub = m_rosNode->subscribe("/ambf/env/cameras/stereoL/ImageData", 2, &afCameraHMD::imageCallback, this);
    // sub = m_rosNode->subscribe("/decklink_left/camera/image_raw", 2, &afCameraHMD::imageCallback, this);
    sub = m_rosNode->subscribe("/zed2i/zed_node/left_raw/image_raw_color", 2, &afCameraHMD::imageCallback, this);

    // sub2 = m_rosNode2->subscribe("/ambf/env/cameras/stereoR/ImageData", 2, &afCameraHMD::imageCallback2, this);
    // sub2 = m_rosNode2->subscribe("/decklink_right/camera/image_raw", 2, &afCameraHMD::imageCallback2, this);
    sub2 = m_rosNode2->subscribe("/zed2i/zed_node/right_raw/image_raw_color", 2, &afCameraHMD::imageCallback2, this);
    
    // sub3= mode_rosNode->subscribe("/volumetric/camera_params/op_mode", 2, &afCameraHMD::KVCallback,this);
    // sub3= mode_rosNode->subscribe("/mode_topic", 2, &afCameraHMD::KVCallback,this);
    sub3= mode_rosNode->subscribe("/volumetric/camera_params/op_mode", 2, &afCameraHMD::numberCallback,this);

    sub4 = para_rosNode1->subscribe("/volumetric/camera_params/dist_params1", 2, &afCameraHMD::floatCallback1,this);
    sub5 = para_rosNode2->subscribe("/volumetric/camera_params/dist_params2", 2, &afCameraHMD::floatCallback2,this);
    sub6 = para_rosNode3->subscribe("/volumetric/camera_params/dist_params3", 2, &afCameraHMD::floatCallback3,this);
    sub7 = para_rosNode4->subscribe("/volumetric/camera_params/dist_params4", 2, &afCameraHMD::floatCallback4,this);

    m_camera = (afCameraPtr)a_afObjectPtr;
    m_camera->setOverrideRendering(true);

    m_camera->getInternalCamera()->m_stereoOffsetW = 0.1;

    m_frameBuffer = cFrameBuffer::create();
    m_frameBuffer->setup(m_camera->getInternalCamera(), m_width * m_alias_scaling, m_height * m_alias_scaling, true, true, GL_RGBA);

    string file_path = __FILE__;
    g_current_filepath = file_path.substr(0, file_path.rfind("/"));

    afShaderAttributes shaderAttribs;
    shaderAttribs.m_shaderDefined = true;
    shaderAttribs.m_vtxFilepath = g_current_filepath + "/shaders/hmd_distortion.vs";
    shaderAttribs.m_fragFilepath = g_current_filepath + "/shaders/hmd_distortion.fs";

    m_shaderPgm = afShaderUtils::createFromAttribs(&shaderAttribs, "TEST", "VR_CAM");
    if (!m_shaderPgm)
    {
        cerr << "ERROR! FAILED TO LOAD SHADER PGM \n";
        return -1;
    }

    m_viewport_scale[0] = 0.122822f;
    m_viewport_scale[0] /= 2.0;
    m_viewport_scale[1] = 0.068234f;

    // m_distortion_coeffs[0] = 0.098;
    // m_distortion_coeffs[1] = 0.324;
    // m_distortion_coeffs[2] = -0.241;
    // m_distortion_coeffs[3] = 0.89;

    // For ZED 2i
    // m_distortion_coeffs[0] = 0.127;
    // m_distortion_coeffs[1] = -0.39;
    // m_distortion_coeffs[2] = 0.318;
    m_distortion_coeffs[0] = 0;//-0.072;
    m_distortion_coeffs[1] = 0;//0.386;
    m_distortion_coeffs[2] = 0;//-0.702;
    m_distortion_coeffs[3] = 1-m_distortion_coeffs[0]-m_distortion_coeffs[1]-m_distortion_coeffs[2];

    // For microscope
    // m_distortion_coeffs[0] = -0.132;
    // m_distortion_coeffs[1] = 0.555;
    // m_distortion_coeffs[2] = -0.772;
    // m_distortion_coeffs[3] = 1-m_distortion_coeffs[0]-m_distortion_coeffs[1]-m_distortion_coeffs[2];

    // m_distortion_coeffs2[0] = 0.098;
    // m_distortion_coeffs2[1] = 0.324;
    // m_distortion_coeffs2[2] = -0.241;
    m_distortion_coeffs2[0] = 0.000;
    m_distortion_coeffs2[1] = 0.0;
    m_distortion_coeffs2[2] = -0.0;
    m_distortion_coeffs2[3] = 1-m_distortion_coeffs2[0]-m_distortion_coeffs2[1]-m_distortion_coeffs2[2];

    m_aberr_scale[0] = 1.0;
    m_aberr_scale[1] = 1.0;
    m_aberr_scale[2] = 1.0;

    m_aberr_scale2[0] = 1.0*8;
    m_aberr_scale2[1] = 1.0*8;
    m_aberr_scale2[2] = 1.0*8;

    m_sep = 0.057863;
    m_vpos = 0.033896;
    m_vpos2 = 0.033896+0.015;
    // m_vpos2 = 0.033896;

    m_left_lens_center[0] = m_viewport_scale[0] - m_sep / 2.0;
    m_left_lens_center[1] = m_vpos;

    m_right_lens_center[0] = m_sep / 2.0;
    m_right_lens_center[1] = m_vpos;

    m_left_lens_center2[0] = m_viewport_scale[0] - m_sep / 2.0-0.01;
    // m_left_lens_center2[0] = m_viewport_scale[0] - m_sep / 2.0;
    m_left_lens_center2[1] = m_vpos2;

    m_right_lens_center2[0] = m_sep / 2.0-0.018;
    // m_right_lens_center2[0] = m_sep / 2.0;
    m_right_lens_center2[1] = m_vpos2;

    m_warp_scale = (m_left_lens_center[0] > m_right_lens_center[0]) ? m_left_lens_center[0] : m_right_lens_center[0];
    m_warp_adj = 1.0;

    m_quadMesh = new cMesh();
    float quad[] = {
        // positions
        -1.0f,
        1.0f,
        0.0f,
        -1.0f,
        -1.0f,
        0.0f,
        1.0f,
        -1.0f,
        0.0f,
        -1.0f,
        1.0f,
        0.0f,
        1.0f,
        -1.0f,
        0.0f,
        1.0f,
        1.0f,
        0.0f,
    };

    for (int vI = 0; vI < 2; vI++)
    {
        int off = vI * 9;
        cVector3d v0(quad[off + 0], quad[off + 1], quad[off + 2]);
        cVector3d v1(quad[off + 3], quad[off + 4], quad[off + 5]);
        cVector3d v2(quad[off + 6], quad[off + 7], quad[off + 8]);
        m_quadMesh->newTriangle(v0, v1, v2);
    }
    m_quadMesh->m_vertices->setTexCoord(1, 0.0, 0.0, 1.0);
    m_quadMesh->m_vertices->setTexCoord(2, 1.0, 0.0, 1.0);
    m_quadMesh->m_vertices->setTexCoord(0, 0.0, 1.0, 1.0);
    m_quadMesh->m_vertices->setTexCoord(3, 0.0, 1.0, 1.0);
    m_quadMesh->m_vertices->setTexCoord(4, 1.0, 0.0, 1.0);
    m_quadMesh->m_vertices->setTexCoord(5, 1.0, 1.0, 1.0);

    m_rosImageTexture = cTexture2d::create();
    //    m_rosImageTexture->setTextureId(0);
    //    m_rosImageTexture->setTextureUnit(GL_TEXTURE0);

    m_quadMesh->computeAllNormals();
    // m_quadMesh->m_texture = m_frameBuffer->m_imageBuffer;
    m_quadMesh->m_texture = m_rosImageTexture;
    m_quadMesh->m_metallicTexture = m_frameBuffer->m_imageBuffer;
    m_quadMesh->setUseTexture(true);

    m_quadMesh->setShaderProgram(m_shaderPgm);
    m_quadMesh->setShowEnabled(true);

    m_vrWorld = new cWorld();
    m_vrWorld->addChild(m_quadMesh);

    cerr << "INFO! LOADING VR PLUGIN \n";

    return 1;
}

void afCameraHMD::numberCallback(const std_msgs::Int32::ConstPtr& msg)
{
    // ROS_INFO_STREAM("1");
    mode= msg->data;
}

void afCameraHMD::floatCallback1(const std_msgs::Float32::ConstPtr& msg)
{
    // ROS_INFO_STREAM("1");
    m_distortion_coeffs[0] = msg->data;
}

void afCameraHMD::floatCallback2(const std_msgs::Float32::ConstPtr& msg)
{
    // ROS_INFO_STREAM("1");
    m_distortion_coeffs[1] = msg->data;
}

void afCameraHMD::floatCallback3(const std_msgs::Float32::ConstPtr& msg)
{
    // ROS_INFO_STREAM("1");
    m_distortion_coeffs[2] = msg->data;
}

void afCameraHMD::floatCallback4(const std_msgs::Float32::ConstPtr& msg)
{
    // ROS_INFO_STREAM("1");
    clipsize = msg->data;
}

void afCameraHMD::KVCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg)
{
    // ROS_INFO_STREAM("1");
    if (msg->value!="mode 1"){
        mode=1;
    }
}


GLenum getImageFormat(const std::string &encoding)
{
    if (encoding.compare("rgb8") == 0 || encoding.compare("rgba8") || encoding.compare("rgb16") || encoding.compare("rgba16"))
    {
        return GL_RGB;
    }
    else if (encoding.compare("bgr8") == 0 || encoding.compare("bgra8") || encoding.compare("bgr16") || encoding.compare("bgra16"))
    {
        return GL_BGR;
    }
    else if (encoding.compare("mono8") == 0 || encoding.compare("mono16") == 0)
    {
        return GL_LUMINANCE;
    }
    else
    {
        cerr << "ERROR! IMAGE PIXEL FORMAT NOT IMPLEMENTED: " << encoding << endl;
        return GL_RGB;
    }
}

GLenum getImageType(const std::string &encoding)
{
    if (encoding.compare("8") > 0)
    {
        return GL_UNSIGNED_BYTE;
    }
    else if (encoding.compare("16") > 0)
    {
        return GL_UNSIGNED_INT;
    }
    else
    {
        cerr << "ERROR! IMAGE PIXEL TYPE NOT IMPLEMENTED: " << encoding << endl;
        return GL_UNSIGNED_BYTE;
    }
}

void afCameraHMD::imageCallback(const sensor_msgs::ImageConstPtr &msg)
// void afCameraHMD::imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        // frame = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_COLOR);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert");
    }

    // cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(cv_ptr->image.cols/2,cv_ptr->image.rows/2));
    
    cv::Rect sizeRect(0,0,cv_ptr->image.cols-cv_ptr->image.cols*clipsize,cv_ptr->image.rows);
    cv_ptr->image = cv_ptr->image(sizeRect);

    // cv::imshow("Image1", cv_ptr->image);
    // cv::waitKey(1);
}

void afCameraHMD::imageCallback2(const sensor_msgs::ImageConstPtr &msg)
// void afCameraHMD::imageCallback2(const sensor_msgs::CompressedImage::ConstPtr& msg)
{

    try
    {
        cv_ptr2 = cv_bridge::toCvCopy(msg, msg->encoding);
        // frame2 = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_COLOR);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert");
    }

    cv::Rect sizeRect2(clipsize,0,cv_ptr2->image.cols-cv_ptr2->image.cols*clipsize,cv_ptr2->image.rows);
    cv_ptr2->image = cv_ptr2->image(sizeRect2);


    cv::hconcat(cv_ptr->image, cv_ptr2->image, cv_ptr->image);
    cv::flip(cv_ptr->image, cv_ptr->image, 0);
    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGBA2BGRA);
    int ros_image_size = cv_ptr->image.cols * cv_ptr->image.rows * cv_ptr->image.elemSize();
    int texture_image_size = m_rosImageTexture->m_image->getWidth() * m_rosImageTexture->m_image->getHeight() * m_rosImageTexture->m_image->getBytesPerPixel();

    if (ros_image_size != texture_image_size)
    {
        m_rosImageTexture->m_image->erase();
        // m_rosImageTexture->m_image->allocate(cv_ptr->image.cols, cv_ptr->image.rows, getImageFormat(cv_ptr->encoding), getImageType(cv_ptr->encoding));
        m_rosImageTexture->m_image->allocate(cv_ptr->image.cols, cv_ptr->image.rows, GL_RGBA, GL_UNSIGNED_BYTE); // For ZED 2i
        // m_rosImageTexture->m_image->allocate(cv_ptr->image.cols, cv_ptr->image.rows, GL_RGB, GL_UNSIGNED_BYTE);
    }

    //  cerr << "INFO! Image Sizes" << msg->width << "x" << msg->height << " - " << msg->encoding << endl;
    m_rosImageTexture->m_image->setData(cv_ptr->image.data, ros_image_size);
    m_rosImageTexture->markForUpdate();
    // cv::imshow("Image1", cv_ptr->image);
    // cv::waitKey(1);
    // cv::resize(cv_ptr2->image,cv_ptr2->image,cv::Size(cv_ptr2->image.cols/2,cv_ptr2->image.rows/2));
    // cv::imshow("Image2", cv_ptr2->image);
    // cv::waitKey(1);
}

void afCameraHMD::graphicsUpdate()
{
    static bool first_time = true;
    if (first_time)
    {
        makeFullScreen();
        first_time = false;
    }

    glfwMakeContextCurrent(m_camera->m_window);
    m_frameBuffer->renderView();
    updateHMDParams();
    afRenderOptions ro;
    ro.m_updateLabels = true;

    cWorld *cachedWorld = m_camera->getInternalCamera()->getParentWorld();
    m_camera->getInternalCamera()->setStereoMode(C_STEREO_DISABLED);
    m_camera->getInternalCamera()->setParentWorld(m_vrWorld);
    static cWorld *ew = new cWorld();
    cWorld *fl = m_camera->getInternalCamera()->m_frontLayer;
    m_camera->getInternalCamera()->m_frontLayer = ew;
    m_camera->render(ro);
    m_camera->getInternalCamera()->m_frontLayer = fl;
    m_camera->getInternalCamera()->setStereoMode(C_STEREO_PASSIVE_LEFT_RIGHT);
    m_camera->getInternalCamera()->setParentWorld(cachedWorld);
}

void afCameraHMD::physicsUpdate(double dt)
{
}

void afCameraHMD::reset()
{
}

bool afCameraHMD::close()
{
    return true;
}

void afCameraHMD::updateHMDParams()
{
    GLint id = m_shaderPgm->getId();
    //    cerr << "INFO! Shader ID " << id << endl;
    glUseProgram(id);

        glUniform1i(glGetUniformLocation(id, "windowNumber"), mode);
        glUniform1i(glGetUniformLocation(id, "warpTexture2"), 2);
        glUniform2fv(glGetUniformLocation(id, "ViewportScale"), 1, m_viewport_scale);
        glUniform3fv(glGetUniformLocation(id, "aberr"), 1, m_aberr_scale);
        glUniform3fv(glGetUniformLocation(id, "aberr2"), 1, m_aberr_scale2);
        glUniform1f(glGetUniformLocation(id, "WarpScale"), m_warp_scale * m_warp_adj);
        glUniform4fv(glGetUniformLocation(id, "HmdWarpParam"), 1, m_distortion_coeffs);
        glUniform2fv(glGetUniformLocation(id, "LensCenterLeft"), 1, m_left_lens_center);
        glUniform2fv(glGetUniformLocation(id, "LensCenterRight"), 1, m_right_lens_center);
        glUniform4fv(glGetUniformLocation(id, "HmdWarpParam2"), 1, m_distortion_coeffs);
        glUniform2fv(glGetUniformLocation(id, "LensCenterLeft2"), 1, m_left_lens_center2);
        glUniform2fv(glGetUniformLocation(id, "LensCenterRight2"), 1, m_right_lens_center2);

}

void afCameraHMD::makeFullScreen()
{
    const GLFWvidmode *mode = glfwGetVideoMode(m_camera->m_monitor);
    int w = 2880;
    int h = 1600;
    int x = mode->width - w;
    int y = mode->height - h;
    int xpos, ypos;
    glfwGetMonitorPos(m_camera->m_monitor, &xpos, &ypos);
    x += xpos;
    y += ypos;
    glfwSetWindowPos(m_camera->m_window, x, y);
    glfwSetWindowSize(m_camera->m_window, w, h);
    m_camera->m_width = w;
    m_camera->m_height = h;
    glfwSwapInterval(0);
    cerr << "\t Making " << m_camera->getName() << " fullscreen \n";
}
