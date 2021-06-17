//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
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
*/
//==============================================================================

#include "volumetric_drilling.h"
#include <boost/program_options.hpp>
#include <mutex>

using namespace std;

cVoxelObject* g_volObject;

cToolCursor* g_toolCursor;

int g_renderingMode = 0;

double g_opticalDensity;

mutex g_mutexVoxel;

cCollisionAABBBox g_volumeUpdate;

cColorb g_zeroColor(0x00, 0x00, 0x00, 0x00);

bool g_flagStart = true;

int counter = 0;

cGenericObject* g_selectedObject = NULL;

cTransform g_tool_T_object;

// a haptic device handler
cHapticDeviceHandler* g_deviceHandler;

// a pointer to the current haptic device
cGenericHapticDevicePtr g_hapticDevice;

bool g_flagMarkVolumeForUpdate = false;

afComm* g_commPtr;

enum HapticStates
{
    HAPTIC_IDLE,
    HAPTIC_SELECTION
};

HapticStates state = HAPTIC_IDLE;

void afVolmetricDrillingPlugin::init(int argc, char **argv, const afWorldPtr a_afWorld){

    m_worldPtr = a_afWorld;

    // Get first camera
    afCameraPtr camera = m_worldPtr->getCameras()[0];
    afImageResolutionAttribs imageAttrbs;
    imageAttrbs.m_width = 640;
    imageAttrbs.m_height = 480;
    afNoiseModelAttribs noiseAttribs;
    afShaderAttributes depthShaderAttribs;
    depthShaderAttribs.m_shaderDefined = false;
//    camera->enableImagePublishing(&imageAttrbs);
//    camera->enableDepthPublishing(&imageAttrbs, &noiseAttribs, &depthShaderAttribs);

    string images_path;
    string shaders_path;
    string prefix;
    int count;

    namespace p_opt = boost::program_options;

    p_opt::options_description cmd_opts("Volumetric Drilling Plugin Command Line Options");
    cmd_opts.add_options()
            ("info", "Show help info")
            ("images_path", p_opt::value<string>(), "Path to Images. E.g. ~/volumetric_drilling/resources/volumes/ear3")
            ("shaders_path", p_opt::value<string>(), "Path to Shaders. E.g. ~/volumetric_drilling/resources/shaders")
            ("prefix", p_opt::value<string>(), "Prefix of image name. E.g. plane00")
            ("count", p_opt::value<int>()->default_value(500), "Image Count. Number of images to load. E.g. ");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        cerr << cmd_opts << endl;
        return;
    }

    if (var_map.count("images_path") == 0 || var_map.count("prefix") == 0){
        cerr << cmd_opts << std::endl;
        cerr << "INFO! FOR " << getFilename()  << " PLEASE SPECIFY IMAGES PATH AND PREFIX. SEE EXAMPLE USAGE ABOVE." << endl;
        return;
    }
    else{
        images_path = var_map["images_path"].as<string>();
        prefix = var_map["prefix"].as<string>();
    }

    if (var_map.count("shaders_path")){
        shaders_path = var_map["shaders_path"].as<string>();
    }

    count = var_map["count"].as<int>();

    //    g_mutexVoxel = new mutex();
    double g_opticalDensity = 1.2;
    double maxStiffness = 10.0;

    g_commPtr = new afComm();
    g_commPtr->afCreateCommInstance(afType::RIGID_BODY, "VolumetricDrill", "/ambf/env/", 50 , 1000, 0.5);

    // create a haptic device handler
    g_deviceHandler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    g_deviceHandler->getDevice(g_hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = g_hapticDevice->getSpecifications();

    g_toolCursor = new cToolCursor(a_afWorld->getChaiWorld());

    a_afWorld->addSceneObjectToWorld(g_toolCursor);

    g_toolCursor->setHapticDevice(g_hapticDevice);

    // if the haptic device has a gripper, enable it as a user switch
    g_hapticDevice->setEnableGripperUserSwitch(true);

    g_toolCursor->setRadius(0.04);

    // map the physical workspace of the haptic device to a larger virtual workspace.

    g_toolCursor->setWorkspaceRadius(0.7);

    g_toolCursor->setWaitForSmallForce(true);

    g_toolCursor->start();

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = g_toolCursor->getWorkspaceScaleFactor();

    // stiffness properties
    maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // create a volumetric model
    g_volObject = new cVoxelObject();

    // add object to world
    a_afWorld->addSceneObjectToWorld(g_volObject);

    // position object
    g_volObject->setLocalPos(0.0, 0.0, 0.0);

    // rotate object
    g_volObject->rotateExtrinsicEulerAnglesDeg(90, 30, -90, C_EULER_ORDER_YXZ);

    // set the dimensions by assigning the position of the min and max corners
    g_volObject->m_minCorner.set(-0.5,-0.5,-0.5);
    g_volObject->m_maxCorner.set( 0.5, 0.5, 0.5);

    // set the texture coordinate at each corner.
    g_volObject->m_minTextureCoord.set(0.0, 0.0, 0.0);
    g_volObject->m_maxTextureCoord.set(1.0, 1.0, 1.0);

    // set haptic properties
    g_volObject->m_material->setStiffness(0.2 * maxStiffness);
    g_volObject->m_material->setStaticFriction(0.0);
    g_volObject->m_material->setDynamicFriction(0.0);

    // enable materials
    g_volObject->setUseMaterial(true);

    // set material
    g_volObject->m_material->setWhite();

    // set quality of graphic rendering
    g_volObject->setQuality(0.5);

    g_volObject->setTransparencyLevel(1.0);

    // create multi image
    cMultiImagePtr image = cMultiImage::create();

    string images_path_and_prefix = images_path + "/" + prefix;

    cerr << "Loading prefix " << images_path_and_prefix << endl;

    int filesloaded = image->loadFromFiles(images_path_and_prefix, "png", count);
    if (filesloaded == 0) {
        cout << "Error - Failed to load volume data " << images_path_and_prefix << ".png." << endl;
        close();
        return;
    }

    // create texture
    cTexture3dPtr texture = cTexture3d::create();

    // assign volumetric image to texture
    texture->setImage(image);

    // assign texture to voxel object
    g_volObject->setTexture(texture);

    // create texture
    texture = cTexture3d::create();

    // assign volumetric image to texture
    texture->setImage(image);

    // assign texture to voxel object
    g_volObject->setTexture(texture);

    // initially select an isosurface corresponding to the bone/heart level
    g_volObject->setIsosurfaceValue(0.45);

    // set optical density factor
    g_volObject->setOpticalDensity(g_opticalDensity);

    // set graphic rendering mode
    if (shaders_path.empty() == false){
        cerr << "INFO! USING CUSTOM SHADERS FOR VOLUMETRIC RENDERING" << endl;
        cerr << "INFO! CUSTOM SHADERS PATH: " <<  shaders_path << endl;
        string vert_shader = afUtils::loadFileContents(shaders_path + "/shader.vs");
        string frag_shader = afUtils::loadFileContents(shaders_path + "/shader.fs");
        //    cerr << "VERTEX SHADER: " << "\n\t" << vert_shader << endl;
        //    cerr << "FRAGMT SHADER: " << "\n\t"<< frag_shader << endl;
        g_volObject->setCustomShaders(vert_shader, frag_shader);
        //    cerr << __LINE__ << endl;
    }
    else{
        cerr << "INFO! CUSTOM SHARED PATH NOT DEFINED. USING DEFAULT ISO SURFACE COLOR SHADERS FOR VOLUMETRIC RENDERING" << endl;
        g_volObject->setRenderingModeIsosurfaceColors();
    }
}

void afVolmetricDrillingPlugin::graphicsUpdate(){
    // update region of voxels to be updated
    if (g_flagMarkVolumeForUpdate)
    {
        g_mutexVoxel.lock();
        cVector3d min = g_volumeUpdate.m_min;
        cVector3d max = g_volumeUpdate.m_max;
        g_volumeUpdate.setEmpty();
        g_mutexVoxel.unlock();
        ((cTexture3d*)g_volObject->m_texture.get())->markForPartialUpdate(min, max);
        g_flagMarkVolumeForUpdate = false;
    }
}

void afVolmetricDrillingPlugin::physicsUpdate(double dt){

    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);

    // update position and orientation of tool
    g_toolCursor->updateFromDevice();

    // read user switch
    int userSwitches = g_toolCursor->getUserSwitches();

    if (g_toolCursor->isInContact(g_volObject) && (userSwitches == 2))
    {
        // retrieve contact event
        //        std::cerr << "Num of collision events " << tool->m_hapticPoint->getNumCollisionEvents() << std::endl;
        for (int e = 0 ; e < g_toolCursor->m_hapticPoint->getNumCollisionEvents() ; e++){
            cCollisionEvent* contact = g_toolCursor->m_hapticPoint->getCollisionEvent(e);

            cMatrix3d R_t_w;
            //            tool->getHapticDevice()->getRotation(R_t_w);
            cVector3d Px(1, 0, 0);
            cVector3d dir = R_t_w * Px;

            cVector3d orig(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);
            for (int rI = 0 ; rI < 1 ; rI++){
                cVector3d ray = orig + rI * dir;
                g_volObject->m_texture->m_image->setVoxelColor(uint(ray.x()), uint(ray.y()), uint(ray.z()), g_zeroColor);

                g_mutexVoxel.lock();
                g_volumeUpdate.enclose(cVector3d(uint(ray.x()), uint(ray.y()), uint(ray.z())));
                g_mutexVoxel.unlock();
            }
            //                object->m_texture->m_image->setVoxelColor(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ, color);
            // mark voxel for update

        }

        g_flagMarkVolumeForUpdate = true;

        //            int ix = contact->m_voxelIndexX;
        //            int iy = contact->m_voxelIndexY;
        //            int iz = contact->m_voxelIndexZ;
    }

    // compute interaction forces
    g_toolCursor->computeInteractionForces();

    // check if device remains stuck inside voxel object
    cVector3d force = g_toolCursor->getDeviceGlobalForce();
    if (g_flagStart)
    {
        if (force.length() != 0.0)
        {
            g_toolCursor->initialize();
            counter = 0;
        }
        else
        {
            counter++;
            if (counter > 10)
                g_flagStart = false;
        }
    }
    else
    {
        if (force.length() > 10.0)
        {
            g_flagStart = true;
        }
    }


    /////////////////////////////////////////////////////////////////////////
    // MANIPULATION
    /////////////////////////////////////////////////////////////////////////

    // compute transformation from world to tool (haptic device)
    cTransform world_T_tool = g_toolCursor->getDeviceGlobalTransform();

    // get status of user switch
    bool button = g_toolCursor->getUserSwitch(0);

    //
    // STATE 1:
    // Idle mode - user presses the user switch
    //
    if ((state == HAPTIC_IDLE) && (button == true))
    {
        // check if at least one contact has occurred
        if (g_toolCursor->m_hapticPoint->getNumCollisionEvents() > 0)
        {
            // get contact event
            cCollisionEvent* collisionEvent = g_toolCursor->m_hapticPoint->getCollisionEvent(0);

            // get object from contact event
            g_selectedObject = collisionEvent->m_object;
        }
        else
        {
            g_selectedObject = g_volObject;
        }

        // get transformation from object
        cTransform world_T_object = g_selectedObject->getGlobalTransform();

        // compute inverse transformation from contact point to object
        cTransform tool_T_world = world_T_tool;
        tool_T_world.invert();

        // store current transformation tool
        g_tool_T_object = tool_T_world * world_T_object;

        // update state
        state = HAPTIC_SELECTION;
    }


    //
    // STATE 2:
    // Selection mode - operator maintains user switch enabled and moves object
    //
    else if ((state == HAPTIC_SELECTION) && (button == true))
    {
        // compute new transformation of object in global coordinates
        cTransform world_T_object = world_T_tool * g_tool_T_object;

        // compute new transformation of object in local coordinates
        cTransform parent_T_world = g_selectedObject->getParent()->getLocalTransform();
        parent_T_world.invert();
        cTransform parent_T_object = parent_T_world * world_T_object;

        // assign new local transformation to object
        g_selectedObject->setLocalTransform(parent_T_object);

        // set zero forces when manipulating objects
        g_toolCursor->setDeviceGlobalForce(0.0, 0.0, 0.0);

        g_toolCursor->initialize();
    }

    //
    // STATE 3:
    // Finalize Selection mode - operator releases user switch.
    //
    else
    {
        state = HAPTIC_IDLE;
    }


    cVector3d localPos = g_toolCursor->getDeviceLocalPos();
    cQuaternion localRot;
    localRot.fromRotMat(g_toolCursor->getDeviceLocalRot());

    g_commPtr->m_afRigidBodyCommPtr->cur_position(localPos.x(), localPos.y(), localPos.z());
    g_commPtr->m_afRigidBodyCommPtr->cur_orientation(localRot.x, localRot.y, localRot.z, localRot.w);
    g_commPtr->m_afRigidBodyCommPtr->cur_force(g_toolCursor->getDeviceLocalForce().x(), g_toolCursor->getDeviceLocalForce().y(), g_toolCursor->getDeviceLocalForce().z());
    g_commPtr->m_afRigidBodyCommPtr->cur_torque(g_toolCursor->getDeviceLocalTorque().x(), g_toolCursor->getDeviceLocalTorque().y(), g_toolCursor->getDeviceLocalTorque().z());


    /////////////////////////////////////////////////////////////////////////
    // FINALIZE
    /////////////////////////////////////////////////////////////////////////

    // send forces to haptic device
    //    tool->applyToDevice();
}

void afVolmetricDrillingPlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods){
    // option - reduce size along X axis
    if (a_key == GLFW_KEY_4)
    {
        double value = cClamp((g_volObject->m_maxCorner.x() - 0.005), 0.01, 0.5);
        g_volObject->m_maxCorner.x(value);
        g_volObject->m_minCorner.x(-value);
        g_volObject->m_maxTextureCoord.x(0.5+value);
        g_volObject->m_minTextureCoord.x(0.5-value);
        cout << "> Reduce size along X axis.                            \r";
    }

    // option - increase size along X axis
    else if (a_key == GLFW_KEY_5)
    {
        double value = cClamp((g_volObject->m_maxCorner.x() + 0.005), 0.01, 0.5);
        g_volObject->m_maxCorner.x(value);
        g_volObject->m_minCorner.x(-value);
        g_volObject->m_maxTextureCoord.x(0.5+value);
        g_volObject->m_minTextureCoord.x(0.5-value);
        cout << "> Increase size along X axis.                            \r";
    }

    // option - reduce size along Y axis
    else if (a_key == GLFW_KEY_6)
    {
        double value = cClamp((g_volObject->m_maxCorner.y() - 0.005), 0.01, 0.5);
        g_volObject->m_maxCorner.y(value);
        g_volObject->m_minCorner.y(-value);
        g_volObject->m_maxTextureCoord.y(0.5+value);
        g_volObject->m_minTextureCoord.y(0.5-value);
        cout << "> Reduce size along Y axis.                            \r";
    }

    // option - increase size along Y axis
    else if (a_key == GLFW_KEY_7)
    {
        double value = cClamp((g_volObject->m_maxCorner.y() + 0.005), 0.01, 0.5);
        g_volObject->m_maxCorner.y(value);
        g_volObject->m_minCorner.y(-value);
        g_volObject->m_maxTextureCoord.y(0.5+value);
        g_volObject->m_minTextureCoord.y(0.5-value);
        cout << "> Increase size along Y axis.                            \r";
    }

    // option - reduce size along Z axis
    else if (a_key == GLFW_KEY_8)
    {
        double value = cClamp((g_volObject->m_maxCorner.z() - 0.005), 0.01, 0.5);
        g_volObject->m_maxCorner.z(value);
        g_volObject->m_minCorner.z(-value);
        g_volObject->m_maxTextureCoord.z(0.5+value);
        g_volObject->m_minTextureCoord.z(0.5-value);
        cout << "> Reduce size along Z axis.                            \r";
    }

    // option - increase size along Z axis
    else if (a_key == GLFW_KEY_9)
    {
        double value = cClamp((g_volObject->m_maxCorner.z() + 0.005), 0.01, 0.5);
        g_volObject->m_maxCorner.z(value);
        g_volObject->m_minCorner.z(-value);
        g_volObject->m_maxTextureCoord.z(0.5+value);
        g_volObject->m_minTextureCoord.z(0.5-value);
        cout << "> Increase size along Z axis.                            \r";
    }
    // option - decrease quality of graphic rendering
    else if (a_key == GLFW_KEY_L)
    {
        double value = g_volObject->getQuality();
        g_volObject->setQuality(value - 0.01);
        cout << "> Quality set to " << cStr(g_volObject->getQuality(), 1) << "                            \r";
    }

    // option - increase quality of graphic rendering
    else if (a_key == GLFW_KEY_U)
    {
        double value = g_volObject->getQuality();
        g_volObject->setQuality(value + 0.01);
        cout << "> Quality set to " << cStr(g_volObject->getQuality(), 1) << "                            \r";
    }

    // option - polygonize model and save to file
    else if (a_key == GLFW_KEY_P)
    {
        cMultiMesh* surface = new cMultiMesh;
        g_volObject->polygonize(surface, 0.01, 0.01, 0.01);
        double SCALE = 0.1;
        double METERS_TO_MILLIMETERS = 1000.0;
        surface->scale(SCALE * METERS_TO_MILLIMETERS);
        surface->setUseVertexColors(true);
        surface->saveToFile("volume.obj");
        cout << "> Volume has been polygonized and saved to disk                            \r";
        delete surface;
    }
    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_UP)
    {
        double value = g_volObject->getOpacityThreshold();
        g_volObject->setOpacityThreshold(value + 0.01);
        cout << "> Opacity Threshold set to " << cStr(g_volObject->getOpacityThreshold(), 1) << "                            \n";
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_DOWN)
    {
        double value = g_volObject->getOpacityThreshold();
        g_volObject->setOpacityThreshold(value - 0.01);
        cout << "> Opacity Threshold set to " << cStr(g_volObject->getOpacityThreshold(), 1) << "                            \n";
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_RIGHT)
    {
        double value = g_volObject->getIsosurfaceValue();
        g_volObject->setIsosurfaceValue(value + 0.01);
        cout << "> Isosurface Threshold set to " << cStr(g_volObject->getIsosurfaceValue(), 1) << "                            \n";
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_LEFT)
    {
        double value = g_volObject->getIsosurfaceValue();
        g_volObject->setIsosurfaceValue(value - 0.01);
        cout << "> Isosurface Threshold set to " << cStr(g_volObject->getIsosurfaceValue(), 1) << "                            \n";
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_ENTER)
    {
        g_renderingMode++;
        if (g_renderingMode >7){
            g_renderingMode = 0;
        }
        switch (g_renderingMode) {
        case 0:
            g_volObject->setRenderingModeBasic();
            std::cerr << "setRenderingModeBasic" << std::endl;
            break;
        case 1:
            g_volObject->setRenderingModeVoxelColors();
            std::cerr << "setRenderingModeVoxelColors" << std::endl;
            break;
        case 2:
            g_volObject->setRenderingModeVoxelColorMap();
            std::cerr << "setRenderingModeVoxelColorMap" << std::endl;
            break;
        case 3:
            g_volObject->setRenderingModeIsosurfaceColors();
            std::cerr << "setRenderingModeIsosurfaceColors" << std::endl;
            break;
        case 4:
            g_volObject->setRenderingModeIsosurfaceMaterial();
            std::cerr << "setRenderingModeIsosurfaceMaterial" << std::endl;
            break;
        case 5:
            g_volObject->setRenderingModeIsosurfaceColorMap();
            std::cerr << "setRenderingModeIsosurfaceColorMap" << std::endl;
            break;
        case 6:
            g_volObject->setRenderingModeDVRColorMap();
            std::cerr << "setRenderingModeDVRColorMap" << std::endl;
            break;
        case 7:
            g_volObject->setRenderingModeCustom();
            std::cerr << "setRenderingModeCustom" << std::endl;
            break;
        default:
            break;
        }
    }
    else if (a_key == GLFW_KEY_PAGE_UP){
        g_opticalDensity += 0.1;
        g_volObject->setOpticalDensity(g_opticalDensity);
        cout << "> Optical Density set to " << cStr(g_opticalDensity, 1) << "                            \n";
    }

    else if (a_key == GLFW_KEY_PAGE_DOWN){
        g_opticalDensity -= 0.1;
        g_volObject->setOpticalDensity(g_opticalDensity);
        cout << "> Optical Density set to " << cStr(g_opticalDensity, 1) << "                            \n";
    }

    else if (a_key == GLFW_KEY_HOME){
        float val = g_volObject->getOpacityThreshold();
        g_volObject->setOpacityThreshold(val + 0.1);
        cout << "> Optical Threshold set to " << cStr(g_volObject->getOpacityThreshold(), 1) << "                            \n";
    }

    else if (a_key == GLFW_KEY_END){
        float val = g_volObject->getOpacityThreshold();
        g_volObject->setOpacityThreshold(val - 0.1);
        cout << "> Optical Threshold set to " << cStr(g_volObject->getOpacityThreshold(), 1) << "                            \n";
    }
}

void afVolmetricDrillingPlugin::mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes){

}

void afVolmetricDrillingPlugin::mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos){

}

void afVolmetricDrillingPlugin::reset(){

}

bool afVolmetricDrillingPlugin::close()
{
    g_toolCursor->stop();
    delete g_deviceHandler;
}