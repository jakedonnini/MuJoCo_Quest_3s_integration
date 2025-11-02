// OpenVR_Bridge.h
// Header matching OpenVR_Bridge.cpp: simple structs and a small wrapper class
// for basic OpenVR initialization / pose polling used by the MuJoCo VR integration.

#ifndef OPENVR_BRIDGE_H
#define OPENVR_BRIDGE_H

#include <openvr.h>
#include <cstdint>

// Simple pose container used in the implementation file.
struct Pose {
    float position[3];
    float orientation[4];
    bool valid;
};

// Camera textures used by the example implementation.
struct Camera {
    vr::Texture_t leftEyeTexture;
    vr::Texture_t rightEyeTexture;
    float size[2];
};

extern Camera camera;

// Aggregated poses (HMD + controllers)
struct AllPoses {
    Pose hmdPose;
    Pose leftControllerPose;
    Pose rightControllerPose;
};

// Small wrapper class declared/defined in OpenVR_Bridge.cpp
class OpenVRBridge {
public:
    OpenVRBridge() = default;
    ~OpenVRBridge() = default;

    // Initialize the OpenVR system. Returns true on success.
    bool init_vr();

    // Poll current VR poses and return aggregated poses.
    AllPoses poll_vr();

    // Shutdown / cleanup OpenVR.
    void shutdown_vr();

    // Get projection frustum for eye i (0 = left, 1 = right)
    std::array<float, 6> getFrustum(int i);

private:
    // Private container holding the HMD + controller poses. Default-initialized.
    AllPoses allPoses{};



    vr::IVRSystem* vr_system = nullptr;
    vr::EVRInitError eError = vr::VRInitError_None;

    typedef uint32_t TrackedDeviceIndex_t;
    static const uint32_t k_unTrackedDeviceIndexInvalid = 0xFFFFFFFF;
    static const uint32_t k_unMaxTrackedDeviceCount = 64;
    static const uint32_t k_unTrackedDeviceIndex_Hmd = 0;
    static constexpr uint32_t k_unTrackedDeviceIndex_Controller[2] = {1, 2};
};

#endif // OPENVR_BRIDGE_H
