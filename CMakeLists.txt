cmake_minimum_required(VERSION 3.15)
project(MuJoCo_Quest3_OpenXR LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Allow user to override OpenXR paths
set(OPENXR_INCLUDE_DIR "" CACHE PATH "Path to OpenXR include directory (containing openxr/openxr.h)")
set(OPENXR_LIBRARY "" CACHE FILEPATH "Path to OpenXR loader library (import lib) e.g. openxr_loader.lib")

# If user didn't set explicit vars, try to find common locations
if(NOT OPENXR_INCLUDE_DIR)
    find_path(OPENXR_INCLUDE_DIR
        NAMES openxr/openxr.h
        PATHS
            "$ENV{OPENXR_ROOT}" 
            "/DESKTOP-HSTA4V5/Mujoco_VR/MuJoCo_Quest_3s_integration/OpenXR-SDK/include"
            "C:/Program Files/OpenXR-SDK/include"
            "MuJoCo_Quest_3s_integration/OpenXR-SDK/include"
            "C:/OpenXR/include"
            
    )
endif()

if(NOT OPENXR_LIBRARY)
    find_library(OPENXR_LIBRARY
        NAMES openxr_loader openxr_loader_1 libopenxr_loader
        PATHS
            "$ENV{OPENXR_ROOT}" 
            "C:/Program Files/OpenXR-SDK/lib"
            "C:/Program Files (x86)/OpenXR-SDK/lib"
            "C:/OpenXR/lib"
    )
endif()

# Try local project SDK folder if the above didn't find anything. Common local path: <project>/OpenXR-SDK
if((NOT OPENXR_INCLUDE_DIR OR NOT OPENXR_LIBRARY) AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/OpenXR-SDK")
    set(LOCAL_OPENXR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/OpenXR-SDK")
    if(NOT OPENXR_INCLUDE_DIR AND EXISTS "${LOCAL_OPENXR_ROOT}/include/openxr/openxr.h")
        set(OPENXR_INCLUDE_DIR "${LOCAL_OPENXR_ROOT}/include")
        message(STATUS "Auto-detected OpenXR include at ${OPENXR_INCLUDE_DIR}")
    endif()

    if(NOT OPENXR_LIBRARY)
        # try common lib locations inside the local SDK
        if(EXISTS "${LOCAL_OPENXR_ROOT}/lib/openxr_loader.lib")
            set(OPENXR_LIBRARY "${LOCAL_OPENXR_ROOT}/lib/openxr_loader.lib")
        elseif(EXISTS "${LOCAL_OPENXR_ROOT}/lib/x64/openxr_loader.lib")
            set(OPENXR_LIBRARY "${LOCAL_OPENXR_ROOT}/lib/x64/openxr_loader.lib")
        elseif(EXISTS "${LOCAL_OPENXR_ROOT}/lib/x86/openxr_loader.lib")
            set(OPENXR_LIBRARY "${LOCAL_OPENXR_ROOT}/lib/x86/openxr_loader.lib")
        elseif(EXISTS "${LOCAL_OPENXR_ROOT}/lib64/openxr_loader.lib")
            set(OPENXR_LIBRARY "${LOCAL_OPENXR_ROOT}/lib64/openxr_loader.lib")
        elseif(EXISTS "${LOCAL_OPENXR_ROOT}/build/src/loader/Debug/openxr_loaderd.lib")
            set(OPENXR_LIBRARY "${LOCAL_OPENXR_ROOT}/build/src/loader/Debug/openxr_loaderd.lib")
        elseif(EXISTS "${LOCAL_OPENXR_ROOT}/build/src/loader/Release/openxr_loader.lib")
            set(OPENXR_LIBRARY "${LOCAL_OPENXR_ROOT}/build/src/loader/Release/openxr_loader.lib")
        endif()
        if(OPENXR_LIBRARY)
            message(STATUS "Auto-detected OpenXR library at ${OPENXR_LIBRARY}")
        endif()
    endif()
endif()

if(NOT OPENXR_INCLUDE_DIR OR NOT OPENXR_LIBRARY)
    message(FATAL_ERROR "OpenXR SDK not found. Please set OPENXR_INCLUDE_DIR and OPENXR_LIBRARY CMake variables or install the OpenXR SDK/loader.\nExample: -DOPENXR_INCLUDE_DIR=\"C:/Program Files/OpenXR-SDK/include\" -DOPENXR_LIBRARY=\"C:/Program Files/OpenXR-SDK/lib/openxr_loader.lib\"")
endif()

message(STATUS "Using OpenXR include: ${OPENXR_INCLUDE_DIR}")
message(STATUS "Using OpenXR library: ${OPENXR_LIBRARY}")

add_executable(VR_Test_Scene
    VR_Test_Scene.cpp
)

# Ensure the include directory actually points to the folder that contains the 'openxr' folder
if(EXISTS "${OPENXR_INCLUDE_DIR}/openxr/openxr.h")
    set(OPENXR_INCLUDE_PARENT "${OPENXR_INCLUDE_DIR}")
elseif(EXISTS "${OPENXR_INCLUDE_DIR}/openxr.h")
    # user may have passed the include dir pointing directly at the header file
    get_filename_component(OPENXR_INCLUDE_PARENT "${OPENXR_INCLUDE_DIR}" DIRECTORY)
else()
    # fallback to given value
    set(OPENXR_INCLUDE_PARENT "${OPENXR_INCLUDE_DIR}")
endif()

# Add include globally and also to the target (global helps some generators like Visual Studio when building on UNC paths)
include_directories(${OPENXR_INCLUDE_PARENT})
target_include_directories(VR_Test_Scene PRIVATE ${OPENXR_INCLUDE_PARENT})

# When using D3D11 platform bindings, define this so openxr_platform.h exposes D3D11 structs
target_compile_definitions(VR_Test_Scene PRIVATE XR_USE_GRAPHICS_API_D3D11)
# Also ensure platform macro is defined so Win32-specific types are available
target_compile_definitions(VR_Test_Scene PRIVATE XR_USE_PLATFORM_WIN32)

if(WIN32)
    # Link against the OpenXR loader and required Win32/D3D11 libs
    target_link_libraries(VR_Test_Scene PRIVATE ${OPENXR_LIBRARY} d3d11 dxgi gdi32 user32)
    # Ensure Windows lean headers
    target_compile_definitions(VR_Test_Scene PRIVATE WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
    target_include_directories(VR_Test_Scene PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

message(STATUS "CMake configuration complete. Configure and build with the commands shown in the project README or the instructions below.")
