#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d11.h>
#include <dxgi.h>
#include <DirectXMath.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#pragma comment(lib, "d3d11.lib")

#define CHECK_XR_RESULT(res, msg) \
    if (XR_FAILED(res)) { \
        std::cerr << "OpenXR Error: " << msg << " (" << res << ")" << std::endl; \
        exit(1); \
    }

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

    // world space 
    XrReferenceSpaceCreateInfo referenceSpaceInfo{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    referenceSpaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE; // or XR_REFERENCE_SPACE_TYPE_STAGE
    referenceSpaceInfo.poseInReferenceSpace = {{0,0,0,1}, {0,0,0}}; // Identity pose
    XrSpace referenceSpace = XR_NULL_HANDLE;
    CHECK_XR_RESULT(
        xrCreateReferenceSpace(session, &referenceSpaceInfo, &referenceSpace),
        "Failed to create reference space"
    );

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
    // Create action set
    XrActionSet actionSet = XR_NULL_HANDLE;
    {
        XrActionSetCreateInfo actionSetInfo{ XR_TYPE_ACTION_SET_CREATE_INFO };
        strcpy_s(actionSetInfo.actionSetName, "gameplay");
        strcpy_s(actionSetInfo.localizedActionSetName, "Gameplay");
        actionSetInfo.priority = 0;
        CHECK_XR_RESULT(xrCreateActionSet(instance, &actionSetInfo, &actionSet), "Failed to create action set");
    }

    // Create hand pose actions
    XrAction leftHandPoseAction = XR_NULL_HANDLE;
    XrAction rightHandPoseAction = XR_NULL_HANDLE;
    XrPath leftHandPath, rightHandPath;
    CHECK_XR_RESULT(xrStringToPath(instance, "/user/hand/left", &leftHandPath), "Failed to get left hand path");
    CHECK_XR_RESULT(xrStringToPath(instance, "/user/hand/right", &rightHandPath), "Failed to get right hand path");

    {
        XrActionCreateInfo actionInfo{ XR_TYPE_ACTION_CREATE_INFO };
        actionInfo.actionType = XR_ACTION_TYPE_POSE_INPUT;
        actionInfo.countSubactionPaths = 1;

        // Left hand
        actionInfo.subactionPaths = &leftHandPath;
        strcpy_s(actionInfo.actionName, "left_hand_pose");
        strcpy_s(actionInfo.localizedActionName, "Left Hand Pose");
        CHECK_XR_RESULT(xrCreateAction(actionSet, &actionInfo, &leftHandPoseAction), "Failed to create left hand pose action");

        // Right hand
        actionInfo.subactionPaths = &rightHandPath;
        strcpy_s(actionInfo.actionName, "right_hand_pose");
        strcpy_s(actionInfo.localizedActionName, "Right Hand Pose");
        CHECK_XR_RESULT(xrCreateAction(actionSet, &actionInfo, &rightHandPoseAction), "Failed to create right hand pose action");
    }

    // Suggest bindings for Oculus Touch
    XrPath profilePath;
    xrStringToPath(instance, "/interaction_profiles/oculus/touch_controller", &profilePath);

    XrPath leftGripPath, rightGripPath;
    xrStringToPath(instance, "/user/hand/left/input/aim/pose", &leftGripPath);
    xrStringToPath(instance, "/user/hand/right/input/aim/pose", &rightGripPath);


    std::vector<XrActionSuggestedBinding> bindings = {
        { leftHandPoseAction, leftGripPath },
        { rightHandPoseAction, rightGripPath }
    };

    XrInteractionProfileSuggestedBinding suggested{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggested.interactionProfile = profilePath;
    suggested.suggestedBindings = bindings.data();
    suggested.countSuggestedBindings = (uint32_t)bindings.size();
    xrSuggestInteractionProfileBindings(instance, &suggested);

    XrSpace leftHandSpace = XR_NULL_HANDLE;
    XrSpace rightHandSpace = XR_NULL_HANDLE;

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
                    std::cout << "Session state: " << sessionState << std::endl;
                    
                    if (sessionState == XR_SESSION_STATE_READY) {
                        XrSessionBeginInfo beginInfo{XR_TYPE_SESSION_BEGIN_INFO};
                        beginInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                        CHECK_XR_RESULT(xrBeginSession(session, &beginInfo), "xrBeginSession failed");

                        // Attach action set
                        XrSessionActionSetsAttachInfo attachInfo{ XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO };
                        attachInfo.countActionSets = 1;
                        attachInfo.actionSets = &actionSet;
                        CHECK_XR_RESULT(xrAttachSessionActionSets(session, &attachInfo), "Failed to attach action sets");

                        // Create action spaces
                        XrActionSpaceCreateInfo spaceInfo{ XR_TYPE_ACTION_SPACE_CREATE_INFO };
                        spaceInfo.poseInActionSpace = {{0,0,0,1},{0,0,0}};

                        spaceInfo.action = leftHandPoseAction;
                        spaceInfo.subactionPath = leftHandPath;
                        CHECK_XR_RESULT(xrCreateActionSpace(session, &spaceInfo, &leftHandSpace), "Failed to create left hand space");

                        spaceInfo.action = rightHandPoseAction;
                        spaceInfo.subactionPath = rightHandPath;
                        CHECK_XR_RESULT(xrCreateActionSpace(session, &spaceInfo, &rightHandSpace), "Failed to create right hand space");

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
        XrActiveActionSet activeSet{ actionSet };
        XrActionsSyncInfo syncInfo{ XR_TYPE_ACTIONS_SYNC_INFO };
        syncInfo.countActiveActionSets = 1;
        syncInfo.activeActionSets = &activeSet;
        xrSyncActions(session, &syncInfo);

        // ------------------------
        // Get controller action states
        // ------------------------
        auto getPoseState = [&](XrAction action, XrPath path) -> bool {
            XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };
            getInfo.action = action;
            getInfo.subactionPath = path;
            XrActionStatePose poseState{ XR_TYPE_ACTION_STATE_POSE };
            xrGetActionStatePose(session, &getInfo, &poseState);
            return poseState.isActive == XR_TRUE;
        };

        bool leftActive  = getPoseState(leftHandPoseAction, leftHandPath);
        bool rightActive = getPoseState(rightHandPoseAction, rightHandPath);

        // ------------------------
        // Locate controller spaces if active
        // ------------------------
        auto printController = [&](const char* name, XrSpace space, bool active) {
            XrSpaceLocation loc{ XR_TYPE_SPACE_LOCATION };
            if (active) {
                xrLocateSpace(space, referenceSpace, frameState.predictedDisplayTime, &loc);
            }

            if (!active || !(loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)) {
                std::cout << name << " Pos: INVALID";
            } else {
                auto& p = loc.pose.position;
                std::cout << name << " Pos: (" << p.x << "," << p.y << "," << p.z << ")";
            }

            if (!active || !(loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
                std::cout << " Ori: INVALID\n";
            } else {
                auto& o = loc.pose.orientation;
                std::cout << " Ori: (" << o.x << "," << o.y << "," << o.z << "," << o.w << ")\n";
            }
        };

        printController("Left Controller", leftHandSpace, leftActive);
        printController("Right Controller", rightHandSpace, rightActive);

        xrEndFrame(session, nullptr);
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
