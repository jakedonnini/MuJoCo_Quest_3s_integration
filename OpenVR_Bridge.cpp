// OpenVR_Bridge creates basic tools for VR for mujoco
#include "OpenVR_Bridge.h"
#include <openvr.h>
#include <cmath>
#include <iostream>

bool OpenVRBridge::init_vr() {
    eError = vr::VRInitError_None;
    vr_system = vr::VR_Init(&eError, vr::VRApplication_Scene);

    if (eError != vr::VRInitError_None) {
        std::cerr << "Unable to init VR runtime: "
                  << vr::VR_GetVRInitErrorAsEnglishDescription(eError) << std::endl;
        return false;
    }

    if ( !vr::VRCompositor() )
	{
		printf( "Compositor initialization failed. See log file for details\n" );
		return false;
	}

    // Initialize camera textures and size here as needed
    // ...

    return true;
}

AllPoses OpenVRBridge::poll_vr() {
    if (!vr_system) return {};

    vr::TrackedDevicePose_t trackedDevicePose[k_unMaxTrackedDeviceCount];
    vr::VRCompositor()->WaitGetPoses(trackedDevicePose, k_unMaxTrackedDeviceCount, nullptr, 0);

    if (trackedDevicePose[k_unTrackedDeviceIndex_Hmd].bPoseIsValid) {
        const vr::HmdMatrix34_t& mat = trackedDevicePose[k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking;

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

        pose.orientation[0] = qx;
        pose.orientation[1] = qy;
        pose.orientation[2] = qz;
        pose.orientation[3] = qw;

        pose.valid = true;
    } else {
        pose.valid = false;
    }
}

void OpenVRBridge::shutdown_vr() {
    if (vr_system) {
        vr::VR_Shutdown();
        vr_system = nullptr;
    }
}