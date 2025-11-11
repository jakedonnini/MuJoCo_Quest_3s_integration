// OpenVR_Bridge creates basic tools for VR for mujoco
#include "OpenVR_Bridge.h"
#include <openvr.h>
#include <cmath>
#include <iostream>
#include <array>

bool OpenVRBridge::init_vr() {
    eError = vr::VRInitError_None;
    vr_system = vr::VR_Init(&eError, vr::VRApplication_Scene);

    if (eError != vr::VRInitError_None) {
        std::cerr << "Unable to init VR runtime: "
                  << vr::VR_GetVRInitErrorAsEnglishDescription(eError) << std::endl;
        return false;
    }

    if (!vr::VRCompositor())
	{
		printf( "Compositor initialization failed. See log file for details\n" );
		return false;
	}

    vr::VRCompositor()->SetTrackingSpace(vr::TrackingUniverseStanding);

    // get HMD eye-to-head offsets (no rotation)
    for(int n=0; n<2; n++) {
        vr::EVREye eye = (n == 0) ? vr::Eye_Left : vr::Eye_Right;
        vr::HmdMatrix34_t tmp = vr_system->GetEyeToHeadTransform(eye);
        eyeoffset[n][0] = tmp.m[0][3];
        eyeoffset[n][1] = tmp.m[1][3];
        eyeoffset[n][2] = tmp.m[2][3];
    }

    std::cout << "Eye offsets (L/R): "
              << eyeoffset[0][0] << "," << eyeoffset[0][1] << "," << eyeoffset[0][2] << " / "
              << eyeoffset[1][0] << "," << eyeoffset[1][1] << "," << eyeoffset[1][2] << std::endl;

    return true;
}

std::array<std::array<float, 3>, 2> OpenVRBridge::getEyeOffset() {
    std::array<std::array<float, 3>, 2> out{};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
            out[i][j] = eyeoffset[i][j];
    return out;
}

std::array<float, 6> OpenVRBridge::getFrustum(int i) {
    if (!vr_system) {
        std::cerr << "OpenVRBridge::getFrustum called before init_vr() or after shutdown. Returning defaults." << std::endl;
        return {0.0f, 1.0f, -1.0f, 1.0f, 0.1f, 100.0f};
    }
    vr::EVREye eye = (i == 0) ? vr::Eye_Left : vr::Eye_Right;

    float nearClip = 0.05f; // Set near clip distance
    float farClip = 50.0f; // Set far clip distance

    float left, right, top, bottom;
    vr_system->GetProjectionRaw(eye, &left, &right, &top, &bottom);

    float frustum_center = 0.5f*(left + right)*nearClip;
    float frustum_width = right - left;

    std::array<float, 6> frustum = {frustum_center, frustum_width, -bottom*nearClip, -top*nearClip, nearClip, farClip};
    return frustum;
}

std::array<int, 2> OpenVRBridge::getRecommendedRenderTargetSize() {
    if (!vr_system) {
        std::cerr << "vr_system not initialized! Returning {0,0}." << std::endl;
        return {0, 0};
    }

    uint32_t width = 0, height = 0;
    vr_system->GetRecommendedRenderTargetSize(&width, &height);
    return {static_cast<int>(width), static_cast<int>(height)};
}

void OpenVRBridge::getProjectionRaw(vr::EVREye eye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom) {
    if (!vr_system) {
        std::cerr << "OpenVRBridge::getProjectionRaw called before init_vr() or after shutdown." << std::endl;
        return;
    }
    vr_system->GetProjectionRaw(eye, pfLeft, pfRight, pfTop, pfBottom);
}

Pose getPoseFromMatrix(const vr::HmdMatrix34_t& mat) {
    Pose pose;

    // Extract position
    pose.position[0] = mat.m[0][3];
    pose.position[1] = mat.m[1][3];
    pose.position[2] = mat.m[2][3];

    // Extract orientation (quaternion)
    float qw = sqrt(fmax(0, 1 + mat.m[0][0] + mat.m[1][1] + mat.m[2][2])) / 2;
    float qx = sqrt(fmax(0, 1 + mat.m[0][0] - mat.m[1][1] - mat.m[2][2])) / 2;
    float qy = sqrt(fmax(0, 1 - mat.m[0][0] + mat.m[1][1] - mat.m[2][2])) / 2;
    float qz = sqrt(fmax(0, 1 - mat.m[0][0] - mat.m[1][1] + mat.m[2][2])) / 2;
    qx = copysign(qx, mat.m[2][1] - mat.m[1][2]);
    qy = copysign(qy, mat.m[0][2] - mat.m[2][0]);
    qz = copysign(qz, mat.m[1][0] - mat.m[0][1]);

    pose.orientation[0] = qw;
    pose.orientation[1] = qx;
    pose.orientation[2] = qy;
    pose.orientation[3] = qz;

    pose.roomMatrix[0] = mat.m[0][0];
    pose.roomMatrix[1] = mat.m[0][1];
    pose.roomMatrix[2] = mat.m[0][2];
    pose.roomMatrix[3] = mat.m[1][0];
    pose.roomMatrix[4] = mat.m[1][1];
    pose.roomMatrix[5] = mat.m[1][2];
    pose.roomMatrix[6] = mat.m[2][0];
    pose.roomMatrix[7] = mat.m[2][1];
    pose.roomMatrix[8] = mat.m[2][2];

    pose.valid = true;
    return pose;
}

AllPoses OpenVRBridge::poll_vr() {
    if (!vr_system) return {};

    vr::TrackedDevicePose_t trackedDevicePose[k_unMaxTrackedDeviceCount];
    vr::VRCompositor()->WaitGetPoses(trackedDevicePose, k_unMaxTrackedDeviceCount, nullptr, 0);

    // get head pose
    if (trackedDevicePose[k_unTrackedDeviceIndex_Hmd].bPoseIsValid) {
        const vr::HmdMatrix34_t& mat = trackedDevicePose[k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking;
        // helper function to convert HmdMatrix34_t to Pose
        allPoses.hmdPose = getPoseFromMatrix(mat);
    } else {
        allPoses.hmdPose.valid = false;
    }

    // get left controller pose
    if (trackedDevicePose[k_unTrackedDeviceIndex_Controller[0]].bPoseIsValid) {
        const vr::HmdMatrix34_t& mat = trackedDevicePose[k_unTrackedDeviceIndex_Controller[0]].mDeviceToAbsoluteTracking;
        allPoses.leftControllerPose = getPoseFromMatrix(mat);
    } else {
        allPoses.leftControllerPose.valid = false;
    }

    // get right controller pose
    if (trackedDevicePose[k_unTrackedDeviceIndex_Controller[1]].bPoseIsValid) {
        const vr::HmdMatrix34_t& mat = trackedDevicePose[k_unTrackedDeviceIndex_Controller[1]].mDeviceToAbsoluteTracking;
        allPoses.rightControllerPose = getPoseFromMatrix(mat);
    } else {
        allPoses.rightControllerPose.valid = false;
    }

    return allPoses;
}

void OpenVRBridge::submit_vr_frame(GLuint EyeTex) {
    vr::Texture_t vrTexture = { (void*)(uintptr_t)EyeTex, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };

    // submit textures to OpenVR compositor
    vr::VRCompositor()->Submit(vr::Eye_Left, &vrTexture);
    vr::VRCompositor()->Submit(vr::Eye_Right, &vrTexture);
    // Notify the compositor that we've finished submitting for this frame
    vr::VRCompositor()->PostPresentHandoff();
}

void OpenVRBridge::shutdown_vr() {
    if (vr_system) {
        vr::VR_Shutdown();
        vr_system = nullptr;
    }
}