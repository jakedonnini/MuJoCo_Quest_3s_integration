# MuJoCo_Quest_3s_integration
The goal is to create easy to use tools to allow a person to enter a MuJoCo scene with a Quest 3/s to run human robot experiments virtually.

# my setup (Delete for full release)
1. ssh jaked@DESKTOP-HSTA4V5
2. cd OneDrive\Documents\Mujoco_VR\MuJoCo_Quest_3s_integration\build
3. cmake .. -G "Visual Studio 17 2022"
4. cmake --build . --config Debug
5. .\Debug\VR_Test_Scene.exe

# How to intall
1. Install the Meta Horizon Link App on your PC
2. Install OpenXR Tools for windows and insure the quest is in the current runtime

# Notes
=== OpenXR Headset Tracker ===
Available GPUs:
  [0] NVIDIA GeForce GTX 1060 6GB
  [1] NVIDIA GeForce GTX 1060 6GB
  [2] Microsoft Basic Render Driver
OpenXR Instance created.
System ID: 24
D3D11 Device created successfully.
REFLECT [RUNTIMEIPC] WARN@ buck-out\v2\gen\fbsource\5bd4e3313a7009a2\arvr\projects\xrruntime\mobile\VrRuntime\Common\RuntimeServiceSDK\__rssdk_codegen_generated_cpp__\buck-headers\RuntimeServiceSDK/RuntimeServiceSDKTypesClient.h(52): InitClient: EnsureServiceStarted failed
REFLECT [RUNTIMEIPC] WARN@ arvr\projects\xrruntime\mobile\1stParty\RuntimeIPC\RuntimeIPCHelpers\Include\Private\RuntimeReflectIPC.h(2694): ipc_GetServerState FAILED: PerformanceManagerState
REFLECT [RUNTIMEIPC] WARN@ arvr\projects\xrruntime\mobile\1stParty\RuntimeIPC\RuntimeIPCHelpers\Include\Private\RuntimeReflectIPC.h(2694): ipc_GetServerState FAILED: KPIFeatureMeasurementsState
REFLECT [RUNTIMEIPC] WARN@ arvr\projects\xrruntime\mobile\1stParty\RuntimeIPC\RuntimeIPCHelpers\Include\Private\RuntimeReflectIPC.h(2694): ipc_GetServerState FAILED: SystemPerformanceState
REFLECT [RUNTIMEIPC] WARN@ arvr\projects\xrruntime\mobile\1stParty\RuntimeIPC\RuntimeIPCHelpers\Include\Private\RuntimeReflectIPC.h(2694): ipc_GetServerState FAILED: EyeTrackingGlobalState
REFLECT [RUNTIMEIPC] WARN@ arvr\projects\xrruntime\mobile\1stParty\RuntimeIPC\RuntimeIPCHelpers\Include\Private\RuntimeReflectIPC.h(2694): ipc_GetServerState FAILED: FaceTrackingGlobalState
REFLECT [RUNTIMEIPC] WARN@ arvr\projects\xrruntime\mobile\1stParty\RuntimeIPC\RuntimeIPCHelpers\Include\Private\RuntimeReflectIPC.h(2694): ipc_GetServerState FAILED: CodecAvatarEncoderGlobalState
REFLECT [RUNTIMEIPC] WARN@ arvr\projects\xrruntime\mobile\1stParty\RuntimeIPC\RuntimeIPCHelpers\Include\Private\RuntimeReflectIPC.h(2694): ipc_GetServerState FAILED: CompositedPassthroughLayerState
REFLECT [RUNTIMEIPC] WARN@ arvr\projects\xrruntime\mobile\1stParty\RuntimeIPC\RuntimeIPCHelpers\Include\Private\RuntimeReflectIPC.h(2694): ipc_GetServerState FAILED: LayerState
REFLECT [RUNTIMEIPC] WARN@ arvr\projects\xrruntime\mobile\1stParty\RuntimeIPC\RuntimeIPCHelpers\Include\Private\RuntimeReflectIPC.h(2694): ipc_GetServerState FAILED: ToClientKadavuState
REFLECT [RUNTIMEIPC] WARN@ arvr\projects\xrruntime\mobile\1stParty\RuntimeIPC\RuntimeIPCHelpers\Include\Private\RuntimeReflectIPC.h(3022): ipc_GetClientStateHandle FAILED: SurfaceInputsClientState
OpenXR session created successfully.
Tracking headset... Press Ctrl+C to exit.
Session state: 1
Session state: 2
Session is now focused. Polling poses...
Head Pos: (0.0458792,1.09527,0.113532) Ori: (-0.318669,-0.0468254,0.0871655,-0.942687)
Left Controller Pos: INVALID Ori: INVALID
Right Controller Pos: INVALID Ori: INVALID
Head Pos: (0.0450687,1.09498,0.113509) Ori: (-0.318675,-0.0481832,0.0868162,-0.942649)
Left Controller Pos: INVALID Ori: INVALID
Right Controller Pos: INVALID Ori: INVALID
Head Pos: (0.0444343,1.09484,0.113407) Ori: (-0.318269,-0.0487653,0.0857889,-0.94285)
Left Controller Pos: INVALID Ori: INVALID
Right Controller Pos: INVALID Ori: INVALID
Head Pos: (0.0438326,1.0947,0.113114) Ori: (-0.317737,-0.049586,0.0852173,-0.943039)
Left Controller Pos: INVALID Ori: INVALID