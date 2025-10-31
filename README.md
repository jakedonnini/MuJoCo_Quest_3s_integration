# MuJoCo_Quest_3s_integration
The goal is to create easy to use tools to allow a person to enter a MuJoCo scene with a Quest 3/s to run human robot experiments virtually.

# my setup (Delete for full release)
1. ssh jaked@DESKTOP-HSTA4V5 (woodstown5)
2. cd OneDrive\Documents\Mujoco_VR\MuJoCo_Quest_3s_integration\build
3. cmake .. -G "Visual Studio 17 2022" -D USE_OPENXR=OFF \
   -D OPENVR_INCLUDE_DIR="C:\\Users\\jaked\\OneDrive\\Documents\\Mujoco_VR\\MuJoCo_Quest_3s_integration\\steamvr\\openvr\\headers" \
   -D OPENVR_LIBRARY="C:\\Users\\jaked\\OneDrive\\Documents\\Mujoco_VR\\MuJoCo_Quest_3s_integration\\steamvr\\openvr\\lib\\win64\\openvr_api.lib"
4. cmake --build . --config Debug --target MuJoCo_VR_Main_OpenVR
5. .\Debug\MuJoCo_VR_Main_OpenVR.exe

# From your repo root or the samples folder:
cd C:\Users\jaked\OneDrive\Documents\Mujoco_VR\MuJoCo_Quest_3s_integration\steamvr\openvr\samples

cmake -S . -B build -G "Visual Studio 17 2022" -DBUILD_QT_SAMPLES=OFF
cmake --build build --config Debug --target hellovr_opengl

# How to install / run
1. Install the Meta Horizon Link App on your PC
2. Install Steam and SteamVR. Start SteamVR before launching the app.
3. Ensure the Quest is connected via Horizon Link and recognized by SteamVR.
4. If the app says openvr_api.dll is missing, copy it from: 
   - `C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR\\bin\\win64\\openvr_api.dll` into your build `Debug` folder
5. Ensure the MuJoCo model XML exists at the hardcoded path in `MuJoCo_VR_Main.cpp` (change `MODEL_XML` if needed).

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
PS C:\Users\jaked\OneDrive\Documents\Mujoco_VR\MuJoCo_Quest_3s_integration\build> .\Debug\OpenXR_GL.exe                 Press any key to shutdown...
Error [GENERAL | xrCreateInstance | OpenXR-Loader] : LoaderInstance::CreateInstance chained CreateInstance call failed
Error [GENERAL | xrCreateInstance | OpenXR-Loader] : xrCreateInstance failed
XrResult failure [XR_ERROR_API_VERSION_UNSUPPORTED]
    Origin: xrCreateInstance(&createInfo, &g_instance)
    Source: C:\Users\jaked\OneDrive\Documents\Mujoco_VR\MuJoCo_Quest_3s_integration\openxr_gl.cpp:532