#include "pch.h"
#include "common.h"
#include "options.h"
#include "platformdata.h"
#include "platformplugin.h"
#include "graphicsplugin.h"
#include "openxr_program.h"
#include "mujoco/mujoco.h"
#include "kinematics.h"
#include <GLFW/glfw3.h>

#if defined(_WIN32)
// Favor the high performance NVIDIA or AMD GPUs
extern "C" {
// http://developer.download.nvidia.com/devzone/devcenter/gamegraphics/files/OptimusRenderingPolicies.pdf
__declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
// https://gpuopen.com/learn/amdpowerxpressrequesthighperformance/
__declspec(dllexport) DWORD AmdPowerXpressRequestHighPerformance = 0x00000001;
}
#endif  // defined(_WIN32)

namespace {
void ShowHelp() {
    // TODO: Improve/update when things are more settled.
    Log::Write(Log::Level::Info,
               "HelloXr --graphics|-g <Graphics API> [--formfactor|-ff <Form factor>] [--viewconfig|-vc <View config>] "
               "[--blendmode|-bm <Blend mode>] [--space|-s <Space>] [--verbose|-v]");
    Log::Write(Log::Level::Info, "Graphics APIs:            D3D11, D3D12, OpenGLES, OpenGL, Vulkan2, Vulkan, Metal");
    Log::Write(Log::Level::Info, "Form factors:             Hmd, Handheld");
    Log::Write(Log::Level::Info, "View configurations:      Mono, Stereo");
    Log::Write(Log::Level::Info, "Environment blend modes:  Opaque, Additive, AlphaBlend");
    Log::Write(Log::Level::Info, "Spaces:                   View, Local, Stage");
}

bool UpdateOptionsFromCommandLine(Options& options, int argc, char* argv[]) {
    // Provide a sensible default on Windows so users can run without args
#if defined(_WIN32)
    if (options.GraphicsPlugin.empty()) {
        options.GraphicsPlugin = "D3D11";
    }
#endif

    int i = 1;  // Index 0 is the program name and is skipped.

    auto getNextArg = [&] {
        if (i >= argc) {
            throw std::invalid_argument("Argument parameter missing");
        }

        return std::string(argv[i++]);
    };

    while (i < argc) {
        const std::string arg = getNextArg();
        if (EqualsIgnoreCase(arg, "--graphics") || EqualsIgnoreCase(arg, "-g")) {
            options.GraphicsPlugin = getNextArg();
        } else if (EqualsIgnoreCase(arg, "--formfactor") || EqualsIgnoreCase(arg, "-ff")) {
            options.FormFactor = getNextArg();
        } else if (EqualsIgnoreCase(arg, "--viewconfig") || EqualsIgnoreCase(arg, "-vc")) {
            options.ViewConfiguration = getNextArg();
        } else if (EqualsIgnoreCase(arg, "--blendmode") || EqualsIgnoreCase(arg, "-bm")) {
            options.EnvironmentBlendMode = getNextArg();
        } else if (EqualsIgnoreCase(arg, "--space") || EqualsIgnoreCase(arg, "-s")) {
            options.AppSpace = getNextArg();
        } else if (EqualsIgnoreCase(arg, "--verbose") || EqualsIgnoreCase(arg, "-v")) {
            Log::SetLevel(Log::Level::Verbose);
        } else if (EqualsIgnoreCase(arg, "--help") || EqualsIgnoreCase(arg, "-h")) {
            ShowHelp();
            return false;
        } else {
            throw std::invalid_argument(Fmt("Unknown argument: %s", arg.c_str()));
        }
    }

    // If still empty here, require GraphicsPlugin
    if (options.GraphicsPlugin.empty()) {
        Log::Write(Log::Level::Error, "GraphicsPlugin parameter is required");
        ShowHelp();
        return false;
    }

    try {
        options.ParseStrings();
    } catch (std::invalid_argument& ia) {
        Log::Write(Log::Level::Error, ia.what());
        ShowHelp();
        return false;
    }
    Log::Write(Log::Level::Info, Fmt("Using GraphicsPlugin: %s", options.GraphicsPlugin.c_str()));
    return true;
}
}  // namespace



// Suppress warnings from external libraries
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4127) // conditional expression is constant (Eigen)
#endif

#define M_PI 3.14159265358979323846

mjvCamera cam1; 
mjvOption opt; 
mjvScene scn; 
mjrContext con; 
GLFWwindow* window;

// const char* MODEL_XML = "C:/Users/jaked/Documents/Physics_Sim/mujoco_menagerie-main/mujoco_menagerie-main/franka_emika_panda/mjx_panda.xml";
const char* MODEL_XML = "C:/Users/jaked/OneDrive/Documents/Mujoco_VR/MuJoCo_Quest_3s_integration/xml/mjx_panda_MPC.xml";

// update perdiction horizon markers
struct MarkerIds {
    int target_body;
    std::vector<int> pred_bodies;
};

MarkerIds initMarkers(mjModel* m, int num_predictions) {
    MarkerIds ids;
    ids.target_body = mj_name2id(m, mjOBJ_BODY, "target_marker_body");
    for (int i = 0; i < num_predictions; ++i) {
        std::string name = "ee_pred_" + std::to_string(i) + "_body";
        int bid = mj_name2id(m, mjOBJ_BODY, name.c_str());
        ids.pred_bodies.push_back(bid);
    }
    return ids;
}

void setMocapBodyPos(mjData* d, int body_id, const Eigen::Vector3d& p) {
    if (body_id < 0) return;
    d->mocap_pos[3*body_id + 0] = p.x();
    d->mocap_pos[3*body_id + 1] = p.y();
    d->mocap_pos[3*body_id + 2] = p.z();
    // Keep orientation identity
    d->mocap_quat[4*body_id + 0] = 1.0;
    d->mocap_quat[4*body_id + 1] = 0.0;
    d->mocap_quat[4*body_id + 2] = 0.0;
    d->mocap_quat[4*body_id + 3] = 0.0;
}

// basic implementation just to build for now
int main() {
    char error_msg[1000] = "Could not load XML";
    mjModel* m = mj_loadXML(MODEL_XML, NULL, error_msg, sizeof(error_msg));
    if (!m) { std::cerr << "Failed to load model: " << error_msg << std::endl; return 1; }
    mjData* d = mj_makeData(m);
    if (!d) { mj_deleteModel(m); std::cerr << "Failed to allocate data\n"; return 1; }

    // keep display for now for testing 
    if (!glfwInit()) { std::cerr << "GLFW init failed\n"; return 1; }
    window = glfwCreateWindow(1200, 900, "MuJoCo Panda", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    const int predict_horizon = 10; // temp for now
    MarkerIds markers = initMarkers(m, predict_horizon);

    // camera
    mjv_defaultCamera(&cam1);
    cam1.distance = 4.0;   // distance from target
    cam1.azimuth = 45.0;   // azimuth angle
    cam1.elevation = -30.0; // elevation angle

    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    mj_resetData(m, d);

    std::vector<int> joint_ids, qpos_addr, dof_addr;
    for (int i = 1; i <= 7; ++i) {
        int jid = mj_name2id(m, mjOBJ_JOINT, ("joint" + std::to_string(i)).c_str());
        if (jid >= 0) {
            joint_ids.push_back(jid);
            qpos_addr.push_back(m->jnt_qposadr[jid]);
            dof_addr.push_back(m->jnt_dofadr[jid]);
        }
    }

    // Check we found all 7 joints
    const int controlled_dofs = 7;
    if (controlled_dofs != (int)dof_addr.size()) {
        std::cerr << "Warning: Found " << dof_addr.size() << " DOFs, but expected " << controlled_dofs << ".\n";
    }

    // Set initial joint positions to home seed: [0, 0, 0, -pi/2, 0, pi/2, pi/4]
    Eigen::Vector<double, controlled_dofs> q_target;
    q_target << 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, M_PI/4;
    Eigen::Vector<double, controlled_dofs> q_current = q_target;

    const double Kp = 0.01;

    Eigen::Matrix<double, 8, 3> jointPositions;
    Eigen::Matrix4d T0e;
    std::vector<Eigen::Matrix4d> T_list;
    // jacobian
    Eigen::Matrix<double, 6, controlled_dofs> J;
    Eigen::Matrix<double, 6, controlled_dofs> J_mujoco;
    // target end-effector position (4x4 homogeneous transform)
    Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
    Eigen::Vector<double, controlled_dofs> u;
    Eigen::Vector<double, controlled_dofs> dq;

    // varibles for VR
    std::shared_ptr<Options> options = std::make_shared<Options>();
    std::shared_ptr<PlatformData> data = std::make_shared<PlatformData>();

    // Spawn a thread to wait for a keypress
    static bool quitKeyPressed = false;
    auto exitPollingThread = std::thread{[] {
        Log::Write(Log::Level::Info, "Press any key to shutdown...");
        (void)getchar();
        quitKeyPressed = true;
    }};
    exitPollingThread.detach();

    bool requestRestart = false;

    do {
        // Create platform-specific implementation.
        std::shared_ptr<IPlatformPlugin> platformPlugin = CreatePlatformPlugin(options, data);

        // Create graphics API implementation.
        std::shared_ptr<IGraphicsPlugin> graphicsPlugin = CreateGraphicsPlugin(options, platformPlugin);

        // Initialize the OpenXR program.
        std::shared_ptr<IOpenXrProgram> program = CreateOpenXrProgram(options, platformPlugin, graphicsPlugin);

        program->CreateInstance();
        program->InitializeSystem();

        options->SetEnvironmentBlendMode(program->GetPreferredBlendMode());
        // UpdateOptionsFromCommandLine(*options, argc, argv);
        platformPlugin->UpdateOptions(options);
        graphicsPlugin->UpdateOptions(options);

        program->InitializeDevice();
        program->InitializeSession();
        program->CreateSwapchains();

        while (!quitKeyPressed) {
            bool exitRenderLoop = false;
            program->PollEvents(&exitRenderLoop, &requestRestart);
            if (exitRenderLoop) {
                break;
            }

            if (program->IsSessionRunning()) {

                // MuJoCo control loop
                for (int i=0;i<m->nv;i++) d->qfrc_applied[i] = 0.0;
                // check the current joint angles
                for (int i = 0; i < controlled_dofs; ++i) {
                    q_current[i] = d->qpos[qpos_addr[i]];
                }

                // ---- update target marker position ----
                setMocapBodyPos(d, markers.target_body, T_target.block<3,1>(0,3));

                // static KinematicsCache cache;
                // // IK cache (declared once, reused each loop)
                // cache.setConfiguration(q_current);
                // Eigen::Vector<double, 7> dq_step = inverse_kinematics_step_optimized(cache, T_target, 0.1, 0.1);

                // q_target += dq_step; // scale step size

                program->PollActions();
                program->RenderFrame();
                // print head and hands pos to terminal
                VRTrackingState trackingState = program->GetTrackingState();
                std::cout << "Head Position: " << trackingState.head.position.x << ", " << trackingState.head.position.y << ", " << trackingState.head.position.z << std::endl;
                std::cout << "Head Orientation: " << trackingState.head.yawDeg << ", " << trackingState.head.pitchDeg << ", " << trackingState.head.rollDeg << std::endl;
                for (int i = 0; i < Side::COUNT; ++i) {
                    std::cout << "Hand " << (i == Side::LEFT ? "Left" : "Right") << " Position: " << trackingState.hand[i].position.x << ", " << trackingState.hand[i].position.y << ", " << trackingState.hand[i].position.z << std::endl;
                    std::cout << "Hand " << (i == Side::LEFT ? "Left" : "Right") << " Orientation: " << trackingState.hand[i].yawDeg << ", " << trackingState.hand[i].pitchDeg << ", " << trackingState.hand[i].rollDeg << std::endl;
                }

                std::cout << "q_current: " << q_current.transpose() << std::endl;

                // finish MuJoCo control loop
                for (int i = 0; i < controlled_dofs; ++i) {
                    int idx = i;
                    // check to make sure we don't exceed joint limits
                    if (q_target[i] < m->jnt_range[joint_ids[idx]*2]) {
                        q_target[i] = m->jnt_range[joint_ids[idx]*2];
                    } else if (q_target[i] > m->jnt_range[joint_ids[idx]*2 + 1]) {
                        q_target[i] = m->jnt_range[joint_ids[idx]*2 + 1];
                    }
                    d->qpos[qpos_addr[idx]] = q_target[i];
                }

                mj_step(m, d);

                mjv_updateScene(m, d, &opt, NULL, &cam1, mjCAT_ALL, &scn);
                mjr_render(mjrRect{0,0,1200,900}, &scn, &con);

                glfwSwapBuffers(window);
                glfwPollEvents();
            } else {
                // Throttle loop since xrWaitFrame won't be called.
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            } 
        }
    } while (!quitKeyPressed && requestRestart);

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(d);
    mj_deleteModel(m);

    std::cout << "Simulation closed.\n";
    return 0;
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif