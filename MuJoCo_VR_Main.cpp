#include <iostream>
#include <vector>
#include <cmath>
#include "mujoco/mujoco.h"
#include "kinematics.h"
#include <GLFW/glfw3.h>
#include <thread>
#include "OpenVR_Bridge.h"

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
    std::cout << "MuJoCo VR Integration\n";

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

    // Spawn a thread to wait for a keypress
    static bool quitKeyPressed = false;
    auto exitPollingThread = std::thread{[] {
        std::cout << "Press Enter to quit...\n";
        (void)getchar();
        quitKeyPressed = true;
    }};
    exitPollingThread.detach();

    bool requestRestart = false;


    // declare OpenVR bridge
    OpenVRBridge vrBridge;
    AllPoses vrPoses;

    do {

        if (!vrBridge.init_vr()) {
            std::cerr << "Failed to initialize OpenVR.\n";
            break;
        }


        while (!quitKeyPressed) {
            bool exitRenderLoop = false;
            if (exitRenderLoop) {
                break;
            }

            if (true) {

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

                vrPoses = vrBridge.poll_vr();

                if (vrPoses.hmdPose.valid) {
                    // Map HMD position to target position in simulation
                    Eigen::Vector3d hmd_pos(vrPoses.hmdPose.position[0],
                                            vrPoses.hmdPose.position[1],
                                            vrPoses.hmdPose.position[2]);
                    // Simple scaling and offset for demo purposes
                    // T_target(0,3) = hmd_pos.x() * 1.0;
                    // T_target(1,3) = hmd_pos.y() * 1.0;
                    // T_target(2,3) = hmd_pos.z() * 1.0 + 0.5; // offset above ground
                    std::cout << "HMD Position: " << hmd_pos.transpose() << std::endl;
                }

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

    vrBridge.shutdown_vr();

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(d);
    mj_deleteModel(m);

    std::cout << "Simulation closed.\n";
    return 0;
}
