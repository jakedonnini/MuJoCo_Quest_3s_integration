cmake_minimum_required(VERSION 3.15)
project(MuJoCo_Quest3_OpenXR LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependency acquisition options for MuJoCo_VR_Main integration
option(USE_VCPKG "Use vcpkg for dependency resolution (GLFW/Eigen)" OFF)
option(USE_FETCHCONTENT "Use FetchContent to fetch dependencies (GLFW/Eigen)" OFF)
option(DOWNLOAD_MUJOCO "Download MuJoCo automatically if not found" OFF)
option(ENABLE_MUJOCO_VIEWER "Create GLFW/OpenGL MuJoCo viewer window (may conflict with OpenXR D3D11)" OFF)

# Optional local dependency roots (set these if deps live next to this CMake)
set(GLFW_DIR "" CACHE PATH "Path to GLFW root (contains include and lib/lib-vc2022)")
set(EIGEN_DIR "" CACHE PATH "Path to Eigen root (contains Eigen/ headers or include/eigen3)")
set(MUJOCO_DIR "" CACHE PATH "Path to MuJoCo root (contains include and lib)")

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

# -------------------------------
# Windows HelloXR-style program
# Builds your main.cpp + openxr_program.cpp with D3D11 graphics plugin and Win32 platform plugin
# -------------------------------
set(HELLOXR_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/OpenXR-SDK-Source/src/tests/hello_xr")
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
if (WIN32)
    # ------------------------------------------------------------------------
    # Additional deps for MuJoCo_VR_Main (GLFW, Eigen, MuJoCo, OpenGL)
    # These are linked only to the OpenXR_MuJoCoWin target further below.
    # ------------------------------------------------------------------------
    # OpenGL (system library)
    find_package(OpenGL REQUIRED)

    # --- GLFW --- (choose one of vcpkg, FetchContent, or manual)
    if(USE_VCPKG)
        find_package(glfw3 REQUIRED)
        set(GLFW_TARGET glfw)
    elseif(USE_FETCHCONTENT)
        include(FetchContent)
        FetchContent_Declare(
            glfw
            GIT_REPOSITORY https://github.com/glfw/glfw.git
            GIT_TAG 3.4
        )
        set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(glfw)
        # Upstream defines target name 'glfw'
        set(GLFW_TARGET glfw)
    else()
        # Manual GLFW search
        find_path(GLFW_INCLUDE_DIR 
            NAMES GLFW/glfw3.h
            PATHS 
                $ENV{GLFW_DIR}/include
                ${GLFW_DIR}/include
                ${CMAKE_CURRENT_SOURCE_DIR}/glfw-3.4/include
                ${CMAKE_CURRENT_SOURCE_DIR}/glfw-3.4.bin.WIN64/include
                ${CMAKE_CURRENT_SOURCE_DIR}/glfw/include
                ${CMAKE_CURRENT_SOURCE_DIR}/deps/glfw/include
                ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/include
                ${CMAKE_CURRENT_SOURCE_DIR}/external/glfw/include
                ${CMAKE_CURRENT_SOURCE_DIR}/include
                "/usr/local/include"
                "/usr/include"
            DOC "GLFW include directory"
        )
        find_library(GLFW_LIBRARY
            NAMES glfw3 glfw
            PATHS 
                $ENV{GLFW_DIR}/lib-vc2022
                $ENV{GLFW_DIR}/lib
                ${GLFW_DIR}/lib-vc2022
                ${GLFW_DIR}/lib
                ${CMAKE_CURRENT_SOURCE_DIR}/glfw-3.4/lib-vc2022
                ${CMAKE_CURRENT_SOURCE_DIR}/glfw-3.4/bin/Release
                ${CMAKE_CURRENT_SOURCE_DIR}/glfw-3.4.bin.WIN64/lib-vc2022
                ${CMAKE_CURRENT_SOURCE_DIR}/glfw/lib-vc2022
                ${CMAKE_CURRENT_SOURCE_DIR}/glfw/lib
                ${CMAKE_CURRENT_SOURCE_DIR}/deps/glfw/lib-vc2022
                ${CMAKE_CURRENT_SOURCE_DIR}/deps/glfw/lib
                ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/lib-vc2022
                ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/lib
                ${CMAKE_CURRENT_SOURCE_DIR}/external/glfw/lib-vc2022
                ${CMAKE_CURRENT_SOURCE_DIR}/external/glfw/lib
                "/usr/local/lib"
                "/usr/lib"
            DOC "GLFW library"
        )
        if(GLFW_INCLUDE_DIR AND GLFW_LIBRARY)
            add_library(glfw_manual UNKNOWN IMPORTED)
            set_target_properties(glfw_manual PROPERTIES
                IMPORTED_LOCATION ${GLFW_LIBRARY}
                INTERFACE_INCLUDE_DIRECTORIES ${GLFW_INCLUDE_DIR}
            )
            set(GLFW_TARGET glfw_manual)
            message(STATUS "Found GLFW: ${GLFW_LIBRARY}")
        else()
            message(FATAL_ERROR "GLFW not found. Please install GLFW or set GLFW_DIR")
        endif()
    endif()

    # --- Eigen --- (header-only)
    if(USE_VCPKG)
        find_package(Eigen3 REQUIRED)
        set(EIGEN_TARGET Eigen3::Eigen)
    elseif(USE_FETCHCONTENT)
        include(FetchContent)
        FetchContent_Declare(
            eigen
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG 3.4.0
        )
        set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
        set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(eigen)
        # Provide an alias-like INTERFACE target for convenience
        add_library(eigen_manual INTERFACE)
        target_include_directories(eigen_manual INTERFACE ${eigen_SOURCE_DIR})
        set(EIGEN_TARGET eigen_manual)
    else()
        # Manual Eigen search
        find_path(EIGEN_INCLUDE_DIR 
            NAMES Eigen/Core
            PATHS 
                $ENV{EIGEN_DIR}
                ${EIGEN_DIR}
                ${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0
                ${CMAKE_CURRENT_SOURCE_DIR}/eigen
                ${CMAKE_CURRENT_SOURCE_DIR}/deps/eigen-3.4.0
                ${CMAKE_CURRENT_SOURCE_DIR}/deps/eigen
                ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.4.0
                ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen
                ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen-3.4.0
                ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen
                ${CMAKE_CURRENT_SOURCE_DIR}/include/eigen3
                "/usr/local/include/eigen3"
                "/usr/include/eigen3"
            DOC "Eigen include directory"
        )
        if(EIGEN_INCLUDE_DIR)
            add_library(eigen_manual INTERFACE)
            target_include_directories(eigen_manual INTERFACE ${EIGEN_INCLUDE_DIR})
            set(EIGEN_TARGET eigen_manual)
            message(STATUS "Found Eigen: ${EIGEN_INCLUDE_DIR}")
        else()
            message(FATAL_ERROR "Eigen not found. Please install Eigen or set EIGEN_DIR")
        endif()
    endif()

    # --- MuJoCo ---
    function(download_mujoco)
        if(WIN32)
            set(MUJOCO_URL "https://github.com/google-deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-windows-x86_64.zip")
            set(MUJOCO_ARCHIVE "mujoco-3.2.3-windows-x86_64.zip")
            set(MUJOCO_EXTRACT_DIR "mujoco-3.2.3-windows-x86_64")
        elseif(APPLE)
            set(MUJOCO_URL "https://github.com/google-deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-macos-universal2.dmg")
            set(MUJOCO_ARCHIVE "mujoco-3.2.3-macos-universal2.dmg")
            set(MUJOCO_EXTRACT_DIR "mujoco-3.2.3-macos-universal2")
        else()
            set(MUJOCO_URL "https://github.com/google-deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-linux-x86_64.tar.gz")
            set(MUJOCO_ARCHIVE "mujoco-3.2.3-linux-x86_64.tar.gz")
            set(MUJOCO_EXTRACT_DIR "mujoco-3.2.3")
        endif()

        set(MUJOCO_DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/deps")
        file(MAKE_DIRECTORY "${MUJOCO_DOWNLOAD_DIR}")

        if(NOT EXISTS "${MUJOCO_DOWNLOAD_DIR}/${MUJOCO_EXTRACT_DIR}")
            message(STATUS "Downloading MuJoCo...")
            file(DOWNLOAD ${MUJOCO_URL} "${MUJOCO_DOWNLOAD_DIR}/${MUJOCO_ARCHIVE}"
                SHOW_PROGRESS
                STATUS DOWNLOAD_STATUS
            )
            list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
            if(NOT STATUS_CODE EQUAL 0)
                message(FATAL_ERROR "Failed to download MuJoCo: ${DOWNLOAD_STATUS}")
            endif()

            message(STATUS "Extracting MuJoCo...")
            if(MUJOCO_ARCHIVE MATCHES "\\.zip$")
                execute_process(
                    COMMAND ${CMAKE_COMMAND} -E tar xvf "${MUJOCO_DOWNLOAD_DIR}/${MUJOCO_ARCHIVE}"
                    WORKING_DIRECTORY ${MUJOCO_DOWNLOAD_DIR}
                )
            elseif(MUJOCO_ARCHIVE MATCHES "\\.tar\\.gz$")
                execute_process(
                    COMMAND ${CMAKE_COMMAND} -E tar xzf "${MUJOCO_DOWNLOAD_DIR}/${MUJOCO_ARCHIVE}"
                    WORKING_DIRECTORY ${MUJOCO_DOWNLOAD_DIR}
                )
            else()
                message(FATAL_ERROR "Unsupported MuJoCo archive format: ${MUJOCO_ARCHIVE}")
            endif()
        endif()

        set(MUJOCO_DIR "${MUJOCO_DOWNLOAD_DIR}/${MUJOCO_EXTRACT_DIR}" PARENT_SCOPE)
    endfunction()

    # Try to find MuJoCo manually first
    find_path(MUJOCO_INCLUDE_DIR 
        NAMES mujoco/mujoco.h
        PATHS 
            $ENV{MUJOCO_DIR}/include
            ${MUJOCO_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/mujoco-3.3.6-windows-x86_64/include
            ${CMAKE_CURRENT_SOURCE_DIR}/mujoco-3.2.3-windows-x86_64/include
            ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/include
            ${CMAKE_CURRENT_SOURCE_DIR}/deps/mujoco/include
            ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco/include
            ${CMAKE_CURRENT_SOURCE_DIR}/external/mujoco/include
            "/usr/local/include"
            "/usr/include"
        DOC "MuJoCo include directory"
    )
    find_library(MUJOCO_LIBRARY
        NAMES mujoco
        PATHS 
            $ENV{MUJOCO_DIR}/lib
            ${MUJOCO_DIR}/lib
            ${CMAKE_CURRENT_SOURCE_DIR}/mujoco-3.3.6-windows-x86_64/lib
            ${CMAKE_CURRENT_SOURCE_DIR}/mujoco-3.2.3-windows-x86_64/lib
            ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/lib
            ${CMAKE_CURRENT_SOURCE_DIR}/deps/mujoco/lib
            ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco/lib
            ${CMAKE_CURRENT_SOURCE_DIR}/external/mujoco/lib
            "/usr/local/lib"
            "/usr/lib"
        DOC "MuJoCo library"
    )
    if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
        if(DOWNLOAD_MUJOCO)
            message(STATUS "MuJoCo not found, attempting to download...")
            download_mujoco()
            # Re-search in downloaded location
            find_path(MUJOCO_INCLUDE_DIR 
                NAMES mujoco/mujoco.h
                PATHS ${MUJOCO_DIR}/include
                NO_DEFAULT_PATH
            )
            find_library(MUJOCO_LIBRARY
                NAMES mujoco
                PATHS ${MUJOCO_DIR}/lib
                NO_DEFAULT_PATH
            )
        endif()
        if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
            message(FATAL_ERROR "MuJoCo not found. Please install MuJoCo or set MUJOCO_DIR")
        endif()
    endif()

    # Determine MuJoCo directory from library path for DLL copying
    get_filename_component(MUJOCO_LIB_DIR ${MUJOCO_LIBRARY} DIRECTORY)
    get_filename_component(MUJOCO_ROOT_DIR ${MUJOCO_LIB_DIR} DIRECTORY)

    # Create MuJoCo imported target
    add_library(mujoco_imported UNKNOWN IMPORTED)
    set_target_properties(mujoco_imported PROPERTIES
        IMPORTED_LOCATION ${MUJOCO_LIBRARY}
        INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_INCLUDE_DIR}
    )

    add_executable(OpenXR_HelloWin
        # Your app sources
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
        ${INCLUDE_DIR}/openxr_program.cpp
        ${INCLUDE_DIR}/pch.cpp

        # HelloXR support (platform + graphics + common)
        ${INCLUDE_DIR}/platformplugin_factory.cpp
        ${INCLUDE_DIR}/platformplugin_win32.cpp
        ${INCLUDE_DIR}/graphicsplugin_factory.cpp
        ${INCLUDE_DIR}/graphicsplugin_d3d11.cpp
        ${INCLUDE_DIR}/d3d_common.cpp
        ${INCLUDE_DIR}/logger.cpp
    )

    # Includes: OpenXR headers, your root, HelloXR sample src (for graphicsplugin.h etc), and 'src' for common/xr_linear.h
    target_include_directories(OpenXR_HelloWin PRIVATE
        ${OPENXR_INCLUDE_PARENT}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${INCLUDE_DIR}
        # ${HELLOXR_SRC_DIR}
        # ${CMAKE_CURRENT_SOURCE_DIR}/OpenXR-SDK-Source/src
    )

    # Required platform/graphics defines
    target_compile_definitions(OpenXR_HelloWin PRIVATE
        XR_USE_PLATFORM_WIN32
        XR_USE_GRAPHICS_API_D3D11
        WIN32_LEAN_AND_MEAN
        _CRT_SECURE_NO_WARNINGS
    )

    # Link OpenXR loader + D3D11 stack
    target_link_libraries(OpenXR_HelloWin PRIVATE
        ${OPENXR_LIBRARY}
        d3d11
        dxgi
        d3dcompiler
        gdi32
        user32
    )

    set_target_properties(OpenXR_HelloWin PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    message(STATUS "Configured target 'OpenXR_HelloWin' with D3D11 + Win32 using HelloXR support files.")

    # Alternate app target that uses MuJoCo_VR_Main.cpp as entry point
    add_executable(OpenXR_MuJoCoWin
        ${CMAKE_CURRENT_SOURCE_DIR}/MuJoCo_VR_Main.cpp
        ${INCLUDE_DIR}/openxr_program.cpp
        ${INCLUDE_DIR}/pch.cpp

        ${INCLUDE_DIR}/platformplugin_factory.cpp
        ${INCLUDE_DIR}/platformplugin_win32.cpp
        ${INCLUDE_DIR}/graphicsplugin_factory.cpp
        ${INCLUDE_DIR}/graphicsplugin_d3d11.cpp
        ${INCLUDE_DIR}/d3d_common.cpp
        ${INCLUDE_DIR}/logger.cpp
    )

    target_include_directories(OpenXR_MuJoCoWin PRIVATE
        ${OPENXR_INCLUDE_PARENT}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${INCLUDE_DIR}
    )

    target_compile_definitions(OpenXR_MuJoCoWin PRIVATE
        XR_USE_PLATFORM_WIN32
        XR_USE_GRAPHICS_API_D3D11
        WIN32_LEAN_AND_MEAN
        _CRT_SECURE_NO_WARNINGS
        PROJECT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
        ENABLE_MUJOCO_VIEWER=$<IF:$<BOOL:${ENABLE_MUJOCO_VIEWER}>,1,0>
    )

    target_link_libraries(OpenXR_MuJoCoWin PRIVATE
        ${OPENXR_LIBRARY}
        d3d11
        dxgi
        d3dcompiler
        gdi32
        user32
        # MuJoCo + simulation/visualization deps
        ${GLFW_TARGET}
        ${EIGEN_TARGET}
        mujoco_imported
        OpenGL::GL
    )

    set_target_properties(OpenXR_MuJoCoWin PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    message(STATUS "Configured target 'OpenXR_MuJoCoWin' with D3D11 + Win32 using HelloXR support files.")

    # Copy MuJoCo DLL to output on Windows (best effort)
    if(MSVC)
        find_file(MUJOCO_DLL
            NAMES mujoco.dll
            PATHS 
                "${MUJOCO_ROOT_DIR}/bin"
                "${MUJOCO_ROOT_DIR}/lib"
                "${MUJOCO_LIB_DIR}"
            DOC "MuJoCo DLL"
        )
        if(MUJOCO_DLL)
            add_custom_command(TARGET OpenXR_MuJoCoWin POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${MUJOCO_DLL}"
                    $<TARGET_FILE_DIR:OpenXR_MuJoCoWin>
                COMMENT "Copying MuJoCo DLL to output directory"
            )
            message(STATUS "Will copy MuJoCo DLL: ${MUJOCO_DLL}")
        else()
            message(WARNING "MuJoCo DLL not found - you may need to copy mujoco.dll manually")
        endif()
    endif()
else()
    message(STATUS "Skipping 'OpenXR_HelloWin' (Windows-only or HelloXR source dir missing: ${HELLOXR_SRC_DIR})")
endif()
