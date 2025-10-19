# MuJoCo_Quest_3s_integration
The goal is to create easy to use tools to allow a person to enter a MuJoCo scene with a Quest 3/s to run human robot experiments virtually.

# my setup (Delete for full release)
1. ssh jaked@DESKTOP-HSTA4V5 (woodstown5)
2. cd OneDrive\Documents\Mujoco_VR\MuJoCo_Quest_3s_integration\build
3. cmake .. -G "Visual Studio 17 2022"
4. cmake --build . --config Debug
5. .\Debug\VR_Test_Scene.exe

# How to intall
1. Install the Meta Horizon Link App on your PC
2. Install OpenXR Tools for windows and insure the quest is in the current runtime

# Pipeline
XR Runtime (OpenXR) 
   ↓  xrLocateViews / xrLocateSpace
   [head pose, hand poses, predictedDisplayTime]
   ↓
Sync/Transform Layer
   - Convert XR poses → MuJoCo coordinate system
   - Update mjvCamera and mjData.qpos
   ↓
MuJoCo Simulation
   - Step physics
   - Render left/right eye frames
   ↓
XR Compositor (xrEndFrame)
   - Submit two rendered images (one per eye)

# Notes
[16:56:53.871][Info   ] Available reference spaces: 3
[16:56:53.878][Info   ] System Properties: Name=Meta Quest 3S VendorId=-11714
[16:56:53.879][Info   ] System Graphics Properties: MaxWidth=4096 MaxHeight=4096 MaxLayers=16
[16:56:53.880][Info   ] System Tracking Properties: OrientationTracking=True PositionTracking=True
[16:56:53.881][Info   ] Creating swapchain for view 0 with dimensions Width=1568 Height=1600 SampleCount=1
[16:56:53.899][Info   ] Creating swapchain for view 1 with dimensions Width=1568 Height=1600 SampleCount=1
[16:56:53.905][Info   ] XrEventDataSessionStateChanged: state XR_SESSION_STATE_UNKNOWN->XR_SESSION_STATE_IDLE session=2
time=24635504732800
[16:56:53.906][Info   ] XrEventDataSessionStateChanged: state XR_SESSION_STATE_IDLE->XR_SESSION_STATE_READY session=2 ti
me=24635504749699
[16:56:53.926][Info   ] XrEventDataSessionStateChanged: state XR_SESSION_STATE_READY->XR_SESSION_STATE_SYNCHRONIZED sess
ion=2 time=24635603462299
[16:56:55.051][Info   ] XrEventDataSessionStateChanged: state XR_SESSION_STATE_SYNCHRONIZED->XR_SESSION_STATE_VISIBLE se
ssion=2 time=24636728942200
[16:56:55.052][Info   ] XrEventDataSessionStateChanged: state XR_SESSION_STATE_VISIBLE->XR_SESSION_STATE_FOCUSED session
=2 time=24636728997799
[16:56:57.647][Info   ] Grab action is bound to 'Left Hand Meta Quest Touch Plus Squeeze' and 'Right Hand Meta Quest Tou
ch Plus Squeeze'
[16:56:57.647][Info   ] Quit action is bound to 'Left Hand Meta Quest Touch Plus Menu Button'
[16:56:57.651][Info   ] Pose action is bound to 'Left Hand Meta Quest Touch Plus Grip Pose' and 'Right Hand Meta Quest T
ouch Plus Grip Pose'
[16:56:57.652][Info   ] Vibrate action is bound to 'Left Hand Meta Quest Touch Plus Vibration' and 'Right Hand Meta Ques
t Touch Plus Vibration'

PS C:\Users\jaked\OneDrive\Documents\Mujoco_VR\MuJoCo_Quest_3s_integration\build> 63