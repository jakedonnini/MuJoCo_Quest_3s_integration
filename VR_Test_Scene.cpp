#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d11.h>
#include <dxgi.h>
#include <DirectXMath.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <iostream>
#include <array>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>

#pragma comment(lib, "d3d11.lib")

#define CHECK_XR_RESULT(res, msg) \
    if (XR_FAILED(res)) { \
        std::cerr << "OpenXR Error: " << msg << " (" << res << ")" << std::endl; \
        exit(1); \
    }

namespace Side {
    const int LEFT = 0;
    const int RIGHT = 1;
    const int COUNT = 2;
}  // namespace Side

struct InputState {
    XrActionSet actionSet{XR_NULL_HANDLE};
    XrAction grabAction{XR_NULL_HANDLE};
    XrAction poseAction{XR_NULL_HANDLE};
    XrAction vibrateAction{XR_NULL_HANDLE};
    XrAction quitAction{XR_NULL_HANDLE};
    std::array<XrPath, Side::COUNT> handSubactionPath;
    std::array<XrSpace, Side::COUNT> handSpace;
    std::array<float, Side::COUNT> handScale = {{1.0f, 1.0f}};
    std::array<XrBool32, Side::COUNT> handActive;
};

int main() {
    std::cout << "=== OpenXR Headset Tracker ===\n";

    // 1️⃣ Create DXGI factory and enumerate adapters
    IDXGIFactory* dxgiFactory = nullptr;
    if (FAILED(CreateDXGIFactory(__uuidof(IDXGIFactory), (void**)&dxgiFactory))) {
        std::cerr << "Failed to create DXGI Factory\n";
        return 1;
    }

    std::vector<IDXGIAdapter*> adapters;
    UINT i = 0;
    IDXGIAdapter* adapter = nullptr;
    std::cout << "Available GPUs:\n";
    while (dxgiFactory->EnumAdapters(i, &adapter) != DXGI_ERROR_NOT_FOUND) {
        DXGI_ADAPTER_DESC desc;
        adapter->GetDesc(&desc);
        std::wcout << L"  [" << i << "] " << desc.Description << L"\n";
        adapters.push_back(adapter);
        i++;
    }

    if (adapters.empty()) {
        std::cerr << "No adapters found\n";
        dxgiFactory->Release();
        return 1;
    }

    // 2️⃣ Create OpenXR instance with D3D11 extension
    const char* extensions[] = { XR_KHR_D3D11_ENABLE_EXTENSION_NAME };
    XrInstanceCreateInfo createInfo{ XR_TYPE_INSTANCE_CREATE_INFO };
    strcpy_s(createInfo.applicationInfo.applicationName, "Headset Tracker");
    createInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
    createInfo.enabledExtensionCount = 1;
    createInfo.enabledExtensionNames = extensions;

    XrInstance instance = XR_NULL_HANDLE;
    CHECK_XR_RESULT(xrCreateInstance(&createInfo, &instance), "Failed to create OpenXR instance");
    std::cout << "OpenXR Instance created.\n";

    // 3️⃣ Get system ID
    XrSystemId systemId;
    XrSystemGetInfo systemInfo{ XR_TYPE_SYSTEM_GET_INFO };
    systemInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    CHECK_XR_RESULT(xrGetSystem(instance, &systemInfo, &systemId), "Failed to get system");
    std::cout << "System ID: " << systemId << std::endl;

    // 4️⃣ Get required D3D11 adapter LUID from runtime
    PFN_xrGetD3D11GraphicsRequirementsKHR pfnGetD3D11Reqs = nullptr;
    CHECK_XR_RESULT(xrGetInstanceProcAddr(
        instance,
        "xrGetD3D11GraphicsRequirementsKHR",
        (PFN_xrVoidFunction*)&pfnGetD3D11Reqs),
        "Failed to get xrGetD3D11GraphicsRequirementsKHR");

    XrGraphicsRequirementsD3D11KHR graphicsRequirements{ XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR };
    pfnGetD3D11Reqs(instance, systemId, &graphicsRequirements);

    // 5️⃣ Match adapter by LUID
    IDXGIAdapter* chosenAdapter = nullptr;
    for (auto a : adapters) {
        DXGI_ADAPTER_DESC desc;
        a->GetDesc(&desc);
        if (memcmp(&desc.AdapterLuid, &graphicsRequirements.adapterLuid, sizeof(LUID)) == 0) {
            chosenAdapter = a;
            break;
        }
    }

    if (!chosenAdapter) {
        std::cerr << "Could not find GPU matching headset.\n";
        for (auto a : adapters) a->Release();
        dxgiFactory->Release();
        return 1;
    }

    // 6️⃣ Create D3D11 device on chosen adapter
    ID3D11Device* d3dDevice = nullptr;
    ID3D11DeviceContext* d3dContext = nullptr;
    D3D_FEATURE_LEVEL featureLevel;

    HRESULT hr = D3D11CreateDevice(
        chosenAdapter,
        D3D_DRIVER_TYPE_UNKNOWN,
        nullptr,
        0,
        nullptr,
        0,
        D3D11_SDK_VERSION,
        &d3dDevice,
        &featureLevel,
        &d3dContext
    );
    if (FAILED(hr)) {
        std::cerr << "Failed to create D3D11 device: " << std::hex << hr << std::endl;
        for (auto a : adapters) a->Release();
        dxgiFactory->Release();
        return 1;
    }
    std::cout << "D3D11 Device created successfully.\n";

    // 7️⃣ Create OpenXR session with graphics binding
    XrGraphicsBindingD3D11KHR graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_D3D11_KHR };
    graphicsBinding.device = d3dDevice;

    XrSessionCreateInfo sessionInfo{ XR_TYPE_SESSION_CREATE_INFO };
    sessionInfo.next = &graphicsBinding;
    sessionInfo.systemId = systemId;

    XrSession session = XR_NULL_HANDLE;
    CHECK_XR_RESULT(xrCreateSession(instance, &sessionInfo, &session), "Failed to create session");
    std::cout << "OpenXR session created successfully.\n";

    // 8️⃣ Create reference space

    // world space (prefer LOCAL for device-relative, room-agnostic tracking)
    XrReferenceSpaceCreateInfo referenceSpaceInfo{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    referenceSpaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    referenceSpaceInfo.poseInReferenceSpace = {{0,0,0,1}, {0,0,0}}; // Identity pose
    XrSpace referenceSpace = XR_NULL_HANDLE;
    XrResult refRes = xrCreateReferenceSpace(session, &referenceSpaceInfo, &referenceSpace);
    if (XR_FAILED(refRes)) {
        // Fallback to STAGE if LOCAL not available
        referenceSpaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
        CHECK_XR_RESULT(
            xrCreateReferenceSpace(session, &referenceSpaceInfo, &referenceSpace),
            "Failed to create reference space (LOCAL and STAGE)"
        );
        std::cout << "Using STAGE reference space (LOCAL unavailable).\n";
    } else {
        std::cout << "Using LOCAL reference space.\n";
    }

    // space of the headset itself
    XrReferenceSpaceCreateInfo viewSpaceInfo{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    viewSpaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW; // Headset itself
    viewSpaceInfo.poseInReferenceSpace = {{0,0,0,1}, {0,0,0}};
    XrSpace headSpace = XR_NULL_HANDLE;
    CHECK_XR_RESULT(
        xrCreateReferenceSpace(session, &viewSpaceInfo, &headSpace),
        "Failed to create head/view space"
    );

    // Controller tracking
    InputState m_input;

    // Create action set
    XrActionSet actionSet = XR_NULL_HANDLE;
    {
        XrActionSetCreateInfo actionSetInfo{ XR_TYPE_ACTION_SET_CREATE_INFO };
        strcpy_s(actionSetInfo.actionSetName, "gameplay");
        strcpy_s(actionSetInfo.localizedActionSetName, "Gameplay");
        actionSetInfo.priority = 0;
        CHECK_XR_RESULT(xrCreateActionSet(instance, &actionSetInfo, &m_input.actionSet), "Failed to create action set");
    }

    // Create hand pose actions
    CHECK_XR_RESULT(xrStringToPath(instance, "/user/hand/left", &m_input.handSubactionPath[Side::LEFT]), "Failed to get left hand path");
    CHECK_XR_RESULT(xrStringToPath(instance, "/user/hand/right", &m_input.handSubactionPath[Side::RIGHT]), "Failed to get right hand path");

    {
        XrActionCreateInfo actionInfo{ XR_TYPE_ACTION_CREATE_INFO };
        actionInfo.actionType = XR_ACTION_TYPE_POSE_INPUT;
        strcpy_s(actionInfo.actionName, "hand_pose");
        strcpy_s(actionInfo.localizedActionName, "Hand Pose");
        actionInfo.countSubactionPaths = uint32_t(m_input.handSubactionPath.size()); // for the two hands
        actionInfo.subactionPaths = m_input.handSubactionPath.data();
        CHECK_XR_RESULT(xrCreateAction(m_input.actionSet, &actionInfo, &m_input.poseAction), "Failed to create pose action");
    }

    // Suggest bindings: try Meta/Oculus profiles and fallback to simple controller
    XrPath leftGripPath, rightGripPath, leftAimPath, rightAimPath;
    xrStringToPath(instance, "/user/hand/left/input/grip/pose", &leftGripPath);
    xrStringToPath(instance, "/user/hand/right/input/grip/pose", &rightGripPath);
    xrStringToPath(instance, "/user/hand/left/input/aim/pose", &leftAimPath);
    xrStringToPath(instance, "/user/hand/right/input/aim/pose", &rightAimPath);

    const char* candidateProfiles[] = {
        "/interaction_profiles/meta/touch_plus_controller",
        "/interaction_profiles/oculus/touch_controller_v3",
        "/interaction_profiles/oculus/touch_controller_v2",
        "/interaction_profiles/oculus/touch_controller",
        "/interaction_profiles/khr/simple_controller"
    };

    bool anySuggested = false;
    for (const char* profileStr : candidateProfiles) {
        XrPath profilePath{};
        if (XR_FAILED(xrStringToPath(instance, profileStr, &profilePath))) continue;
        std::vector<XrActionSuggestedBinding> bindings;
        if (std::string(profileStr).find("simple_controller") != std::string::npos) {
            bindings = {
                { m_input.poseAction, leftAimPath },
                { m_input.poseAction, rightAimPath }
            };
        } else {
            // Bind both grip and aim poses for robustness on Oculus/Meta
            bindings = {
                { m_input.poseAction, leftGripPath },
                { m_input.poseAction, rightGripPath },
                { m_input.poseAction, leftAimPath },
                { m_input.poseAction, rightAimPath }
            };
        }
        XrInteractionProfileSuggestedBinding suggested{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
        suggested.interactionProfile = profilePath;
        suggested.suggestedBindings = bindings.data();
        suggested.countSuggestedBindings = (uint32_t)bindings.size();
        XrResult sb = xrSuggestInteractionProfileBindings(instance, &suggested);
        if (XR_SUCCEEDED(sb)) {
            std::cout << "Suggested bindings for profile: " << profileStr << "\n";
            anySuggested = true;
        } else {
            std::cerr << "Warning: Failed to suggest bindings for '" << profileStr << "' (" << sb << ")\n";
        }
    }
    if (!anySuggested) {
        std::cerr << "No interaction profile bindings suggested. Controller poses may be inactive.\n";
    }

    std::cout << "Tracking headset... Press Ctrl+C to exit.\n";

    // 9️⃣ Frame loop with proper checks and wait for valid pose

    XrEventDataBuffer eventBuffer{XR_TYPE_EVENT_DATA_BUFFER};
    bool sessionRunning = false;
    XrSessionState sessionState = XR_SESSION_STATE_UNKNOWN;

    while (!sessionRunning) {
        while (xrPollEvent(instance, &eventBuffer) == XR_SUCCESS) {
            switch (eventBuffer.type) {
                case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                    auto* stateChanged = reinterpret_cast<XrEventDataSessionStateChanged*>(&eventBuffer);
                    sessionState = stateChanged->state;
                    std::cout << "Session state: " << static_cast<int>(sessionState) << std::endl;
                    
                    if (sessionState == XR_SESSION_STATE_READY) {
                        XrSessionBeginInfo beginInfo{XR_TYPE_SESSION_BEGIN_INFO};
                        beginInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                        CHECK_XR_RESULT(xrBeginSession(session, &beginInfo), "xrBeginSession failed");

                        // maybe able to create before session begins
                        // Create action spaces
                        XrActionSpaceCreateInfo actionSpaceInfo{ XR_TYPE_ACTION_SPACE_CREATE_INFO };
                        actionSpaceInfo.action = m_input.poseAction;
                        actionSpaceInfo.poseInActionSpace.orientation.w = 1.f;
                        actionSpaceInfo.subactionPath = m_input.handSubactionPath[Side::LEFT];
                        CHECK_XR_RESULT(xrCreateActionSpace(session, &actionSpaceInfo, &m_input.handSpace[Side::LEFT]), "Failed to create left hand action space");
                        actionSpaceInfo.subactionPath = m_input.handSubactionPath[Side::RIGHT];
                        CHECK_XR_RESULT(xrCreateActionSpace(session, &actionSpaceInfo, &m_input.handSpace[Side::RIGHT]), "Failed to create right hand action space");

                        // Attach action set
                        XrSessionActionSetsAttachInfo attachInfo{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
                        attachInfo.countActionSets = 1;
                        attachInfo.actionSets = &m_input.actionSet;
                        CHECK_XR_RESULT(xrAttachSessionActionSets(session, &attachInfo), "Failed to attach session action sets");

                        // Log active interaction profiles for each hand
                        for (auto hand : {Side::LEFT, Side::RIGHT}) {
                            XrInteractionProfileState prof{XR_TYPE_INTERACTION_PROFILE_STATE};
                            XrResult gp = xrGetCurrentInteractionProfile(session, m_input.handSubactionPath[hand], &prof);
                            if (XR_SUCCEEDED(gp) && prof.interactionProfile != XR_NULL_PATH) {
                                // We can't easily convert path to string without extension; just note presence
                                std::cout << "Hand " << (hand == Side::LEFT ? "Left" : "Right") << ": interaction profile active.\n";
                            } else {
                                std::cout << "Hand " << (hand == Side::LEFT ? "Left" : "Right") << ": no interaction profile active yet.\n";
                            }
                        }

                        sessionRunning = true;
                    }
                    break;
                }
            }
            eventBuffer.type = XR_TYPE_EVENT_DATA_BUFFER; // reset
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Session is now focused. Polling poses...\n";

    if (sessionState == XR_SESSION_STATE_VISIBLE) {
        std::cout << "Headset tracking active.\n";
    }

    // 7️⃣ Frame loop
    while (true) {
        XrFrameWaitInfo waitInfo{ XR_TYPE_FRAME_WAIT_INFO };
        XrFrameState frameState{ XR_TYPE_FRAME_STATE };
        xrWaitFrame(session, &waitInfo, &frameState);

        XrFrameBeginInfo beginInfo{ XR_TYPE_FRAME_BEGIN_INFO };
        xrBeginFrame(session, &beginInfo);

        XrSpaceLocation location{ XR_TYPE_SPACE_LOCATION };
        // head space and reference space must be different
        xrLocateSpace(headSpace, referenceSpace, frameState.predictedDisplayTime, &location);

        if ((location.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
            (location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
            XrVector3f pos = location.pose.position;
            XrQuaternionf ori = location.pose.orientation;
            std::cout << "Head Pos: (" << pos.x << "," << pos.y << "," << pos.z << ") "
                      << "Ori: (" << ori.x << "," << ori.y << "," << ori.z << "," << ori.w << ")\n";
        } else {
            std::cout << "Pose not valid yet.\n";
        }

         // ------------------------
        // Sync action sets
        // ------------------------
        m_input.handActive = {{XR_FALSE, XR_FALSE}};
        // Explicitly sync left and right subaction paths to ensure both hands become active
        XrActiveActionSet activeSets[2] = {
            { m_input.actionSet, m_input.handSubactionPath[Side::LEFT] },
            { m_input.actionSet, m_input.handSubactionPath[Side::RIGHT] }
        };
        XrActionsSyncInfo syncInfo{ XR_TYPE_ACTIONS_SYNC_INFO };
        syncInfo.countActiveActionSets = 2;
        syncInfo.activeActionSets = activeSets;
        CHECK_XR_RESULT(xrSyncActions(session, &syncInfo), "Failed to sync actions");

        // ------------------------
        // Get controller action states
        // ------------------------
        for (auto hand : {Side::LEFT, Side::RIGHT}) {
            XrActionStateGetInfo getInfo{XR_TYPE_ACTION_STATE_GET_INFO};
            getInfo.action = m_input.poseAction;
            getInfo.subactionPath = m_input.handSubactionPath[hand];
            XrActionStatePose poseState{XR_TYPE_ACTION_STATE_POSE};
            CHECK_XR_RESULT(xrGetActionStatePose(session, &getInfo, &poseState), "Failed to get action state pose");
            m_input.handActive[hand] = poseState.isActive;

            if (!poseState.isActive) {
                std::cout << "Controller " << (hand == Side::LEFT ? "Left" : "Right") << " pose action not active (controller off or not bound).\n";
                continue;
            }

            // List bound sources (diagnostic): which inputs are bound to this action?
            XrBoundSourcesForActionEnumerateInfo bsInfo{XR_TYPE_BOUND_SOURCES_FOR_ACTION_ENUMERATE_INFO};
            bsInfo.action = m_input.poseAction;
            uint32_t sourceCount = 0;
            xrEnumerateBoundSourcesForAction(session, &bsInfo, 0, &sourceCount, nullptr);
            std::vector<XrPath> sources(sourceCount);
            if (sourceCount > 0) {
                xrEnumerateBoundSourcesForAction(session, &bsInfo, sourceCount, &sourceCount, sources.data());
                for (auto p : sources) {
                    char buf[256] = {};
                    uint32_t outLen = 0;
                    if (XR_SUCCEEDED(xrPathToString(instance, p, (uint32_t)sizeof(buf), &outLen, buf))) {
                        std::cout << "Bound source: " << buf << "\n";
                    }
                }
            }

            XrSpaceLocation spaceLocation{XR_TYPE_SPACE_LOCATION};
            XrResult res = xrLocateSpace(m_input.handSpace[hand], referenceSpace, frameState.predictedDisplayTime, &spaceLocation);
            if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
                (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                // print the controller pose
                XrVector3f pos = spaceLocation.pose.position;
                XrQuaternionf ori = spaceLocation.pose.orientation;
                std::cout << "Controller " << (hand == Side::LEFT ? "Left" : "Right") << " Pos: ("
                            << pos.x << "," << pos.y << "," << pos.z << ") "
                            << "Ori: (" << ori.x << "," << ori.y << "," << ori.z << "," << ori.w << ")\n";
            } else {
                // Pose not valid: print flags for diagnosis
                std::cout << "Controller " << (hand == Side::LEFT ? "Left" : "Right")
                          << " pose not valid. res=" << res
                          << " flags=0x" << std::hex << spaceLocation.locationFlags << std::dec
                          << " (need POSITION_VALID and ORIENTATION_VALID).\n";
            }

        }

        // Submit an empty frame (no layers) to keep the runtime happy
        XrFrameEndInfo endInfo{XR_TYPE_FRAME_END_INFO};
        endInfo.displayTime = frameState.predictedDisplayTime;
        endInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        endInfo.layerCount = 0;
        endInfo.layers = nullptr;
        xrEndFrame(session, &endInfo);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }


    // Cleanup
    xrDestroySpace(headSpace);
    xrDestroySpace(referenceSpace);
    xrDestroySession(session);
    xrDestroyInstance(instance);
    d3dContext->Release();
    d3dDevice->Release();
    for (auto a : adapters) a->Release();
    dxgiFactory->Release();
}
