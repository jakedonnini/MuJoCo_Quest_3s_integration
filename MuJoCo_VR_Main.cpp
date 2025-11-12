#include <iostream>
#include <vector>
#include <cmath>
#include "mujoco/mujoco.h"
#include "kinematics.h"

#define GLFW_INCLUDE_NONE     // prevents GLFW from including old gl.h
#include <GLFW/glfw3.h>
#include <glad/gl.h>

#include <thread>
#include <array>
#include "OpenVR_Bridge.h"

#define M_PI 3.14159265358979323846

// mjvCamera objCamera;       // Objective camera (monitor view) 
mjvOption opt; 
mjvScene scn; 
mjrContext con; 
GLFWwindow* window;

mjModel* m = 0;
mjData* d = 0;

GLuint leftEyeFBO, rightEyeFBO;
// GLuint leftEyeTex, rightEyeTex;
GLuint EyeTex;
GLuint leftEyeDepth, rightEyeDepth;

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

// Helper lambda for creating an FBO
auto createEyeBuffer = [&](GLuint &fbo, GLuint &tex, GLuint &depth, int renderWidth, int renderHeight) {
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // --- Color texture ---
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, renderWidth, renderHeight, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, tex, 0);

    // --- Depth buffer ---
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8,
                          renderWidth, renderHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                              GL_RENDERBUFFER, depth);

    // --- Check completeness ---
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cerr << "Error: FBO not complete!" << std::endl;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
};

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

void setMocapHandPos(mjData* d, int body_id, mjtNum modelpos[3], mjtNum modelquat[4]) {
    if (body_id < 0) return;
    d->mocap_pos[3*body_id + 0] = modelpos[0];
    d->mocap_pos[3*body_id + 1] = modelpos[1];
    d->mocap_pos[3*body_id + 2] = modelpos[2];
    // Keep orientation identity
    d->mocap_quat[4*body_id + 0] = modelquat[0];
    d->mocap_quat[4*body_id + 1] = modelquat[1];
    d->mocap_quat[4*body_id + 2] = modelquat[2];
    d->mocap_quat[4*body_id + 3] = modelquat[3];
}

bool checkOrthonormal(const mjtNum forward[3], const mjtNum up[3]) {
    auto dot = forward[0]*up[0] + forward[1]*up[1] + forward[2]*up[2];
    auto f_len = std::sqrt(forward[0]*forward[0] + forward[1]*forward[1] + forward[2]*forward[2]);
    auto u_len = std::sqrt(up[0]*up[0] + up[1]*up[1] + up[2]*up[2]);
    std::cout << "dot(forward, up): " << dot << std::endl;
    std::cout << "‖forward‖=" << f_len << "  ‖up‖=" << u_len << std::endl;
    return (std::abs(dot) < 1e-3 && std::abs(f_len - 1.0) < 1e-3 && std::abs(u_len - 1.0) < 1e-3);
}

// Convert OpenVR pose to MuJoCo coordinates (Y-up -> Z-up)
void convertVRtoMuJoCo(Pose& p) {
    // Rotate position
    float x = p.position[0];
    float y = p.position[1];
    float z = p.position[2];
    p.position[0] = x;
    p.position[1] = z;
    p.position[2] = -y;

    // Rotate orientation (quaternion) by R_vr2mj
    // Equivalent to pre-multiplying by 90° rotation around X axis: qfix = [√0.5, √0.5, 0, 0]
    const float s = std::sqrt(0.5f);
    float qfix[4] = {s, s, 0, 0};  // w, x, y, z (rotates +Y -> +Z)

    // Quaternion multiply: q' = qfix * q
    float qw = qfix[0]*p.orientation[0] - qfix[1]*p.orientation[1]
             - qfix[2]*p.orientation[2] - qfix[3]*p.orientation[3];
    float qx = qfix[0]*p.orientation[1] + qfix[1]*p.orientation[0]
             + qfix[2]*p.orientation[3] - qfix[3]*p.orientation[2];
    float qy = qfix[0]*p.orientation[2] - qfix[1]*p.orientation[3]
             + qfix[2]*p.orientation[0] + qfix[3]*p.orientation[1];
    float qz = qfix[0]*p.orientation[3] + qfix[1]*p.orientation[2]
             - qfix[2]*p.orientation[1] + qfix[3]*p.orientation[0];

    p.orientation[0] = qw;
    p.orientation[1] = qx;
    p.orientation[2] = qy;
    p.orientation[3] = qz;
}


void setVRCamPose(Pose hmdPose, float ipd, OpenVRBridge vrBridge) {
    // convert to room coords
    mjtNum roompos[3]  = { hmdPose.position[0], hmdPose.position[1], hmdPose.position[2] };
    mjtNum roomquat[4] = { hmdPose.orientation[0], hmdPose.orientation[1], hmdPose.orientation[2], hmdPose.orientation[3] };

    // mjtNum roomMat[9] = {
    //     hmdPose.roomMatrix[0], hmdPose.roomMatrix[3], hmdPose.roomMatrix[6],
    //     hmdPose.roomMatrix[1], hmdPose.roomMatrix[4], hmdPose.roomMatrix[7],
    //     hmdPose.roomMatrix[2], hmdPose.roomMatrix[5], hmdPose.roomMatrix[8]
    // };

    mjtNum roomMat[9] = {
        hmdPose.roomMatrix[0], hmdPose.roomMatrix[1], hmdPose.roomMatrix[2],
        hmdPose.roomMatrix[3], hmdPose.roomMatrix[4], hmdPose.roomMatrix[5],
        hmdPose.roomMatrix[6], hmdPose.roomMatrix[7], hmdPose.roomMatrix[8]
    };

    // convert room to quat
    // mju_mat2Quat(roomquat, roomMat);

    std::array<std::array<float, 3>, 2> eyeoffset = vrBridge.getEyeOffset();
    
    mjtNum modelPos[3];
    mjtNum modelQuat[4];
    mjv_room2model(modelPos, modelQuat, roompos, roomquat, &scn);

    mjtNum modelMat[9];
    mju_quat2Mat(modelMat, modelQuat);

    std::cout << "modelMat (column-major view) as rows:\n";
    std::cout << modelMat[0] << ", " << modelMat[3] << ", " << modelMat[6] << ";\n"
              << modelMat[1] << ", " << modelMat[4] << ", " << modelMat[7] << ";\n"
              << modelMat[2] << ", " << modelMat[5] << ", " << modelMat[8] << ";\n";

    for(int n=0; n<2; n++) {
        // assign position, apply eye-to-head offset
        // for(int i=0; i<3; i++)
        //     scn.camera[n].pos[i] = modelpos[i] +
        //         eyeoffset[n][0]*modelMat[3*i+0] +
        //         eyeoffset[n][1]*modelMat[3*i+1] +
        //         eyeoffset[n][2]*modelMat[3*i+2];

        mjtNum offsetModel[3];
        // Convert offset to mjtNum and multiply: offsetModel = R * offset_local
        // Note: R is column-major; mju_mulMatVec signature: out = mat * vec for mat as (nr x nc)
        // Provide mat (R), vec (offset), nr=3, nc=3.
        {
            mjtNum offLocal[3] = { (mjtNum)eyeoffset[n][0], (mjtNum)eyeoffset[n][1], (mjtNum)eyeoffset[n][2] };
            mju_mulMatVec(offsetModel, modelMat, offLocal, 3, 3);
        }

        // camera position = modelPos + offsetModel
        for (int i = 0; i < 3; ++i) {
            scn.camera[n].pos[i] = modelPos[i] + offsetModel[i];
        }

        // assign forward and up
        scn.camera[n].forward[0] = -modelMat[6]; // row 2 col 0
        scn.camera[n].forward[1] = -modelMat[7]; // row 2 col 1
        scn.camera[n].forward[2] = -modelMat[8]; // row 2 col 2
        scn.camera[n].up[0] = modelMat[3]; // row 0 col 1
        scn.camera[n].up[1] = modelMat[4]; // row 1 col 1
        scn.camera[n].up[2] = modelMat[5]; // row 2 col 1

        const mjtNum forward[3] = {scn.camera[n].forward[0], scn.camera[n].forward[1], scn.camera[n].forward[2]};
        const mjtNum up[3] = {scn.camera[n].up[0], scn.camera[n].up[1], scn.camera[n].up[2]};
        // print vecs 
        std::cout << "forward: " << forward[0] << ", " << forward[1] << ", " << forward[2] << std::endl;
        std::cout << "up: " << up[0] << ", " << up[1] << ", " << up[2] << std::endl;
        // std::cout << "Checking orthonormality for eye " << n << ": " << checkOrthonormal(forward, up) << std::endl;
    }
}

void cameraInit(OpenVRBridge vrBridge, int renderWidth, int renderHeight) {
    // Initialize both eyes frusta now that VR is initialized
    std::cout << "Initializing VR eye frusta...\n";
    for (int i = 0; i < 2; ++i) {
        std::array<float, 6> frustum = vrBridge.getFrustum(i);

        scn.camera[i].orthographic = 0; // perspective
        scn.camera[i].frustum_center = frustum[0]; 
        scn.camera[i].frustum_width = frustum[1];
        scn.camera[i].frustum_bottom = frustum[2];
        scn.camera[i].frustum_top = frustum[3];
        scn.camera[i].frustum_near = frustum[4];
        scn.camera[i].frustum_far = frustum[5];
    }

    // create vr texture
    glActiveTexture(GL_TEXTURE2);
    glGenTextures(1, &EyeTex);
    glBindTexture(GL_TEXTURE_2D, EyeTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 2*renderWidth, renderHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
}

void vr_render(int renderWidth, int renderHeight) {
    // resolve multi-sample offscreen buffer
    glBindFramebuffer(GL_READ_FRAMEBUFFER, con.offFBO);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con.offFBO_r);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glBlitFramebuffer(0, 0, 2*renderWidth, renderHeight,
                      0, 0, 2*renderWidth, renderHeight,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);

    // blit to window, left only, window is half-size
    glBindFramebuffer(GL_READ_FRAMEBUFFER, con.offFBO_r);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glDrawBuffer(con.windowDoublebuffer ? GL_BACK : GL_FRONT);
    glBlitFramebuffer(0, 0, renderWidth, renderHeight,
                      0, 0, renderWidth/2, renderHeight/2,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);

    // blit to vr texture
    glActiveTexture(GL_TEXTURE2);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con.offFBO_r);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, EyeTex, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT1);
    glBlitFramebuffer(0, 0, 2*renderWidth, renderHeight,
                      0, 0, 2*renderWidth, renderHeight,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, 0, 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
}

int initMuJoCo(const char* MODEL, int width2, int height) {

    // initialise GLFW
    if (!glfwInit()) { std::cerr << "GLFW init failed\n"; return 0; }

    glfwWindowHint(GLFW_SAMPLES, 0);
    glfwWindowHint(GLFW_DOUBLEBUFFER, 1);
    glfwWindowHint(GLFW_RESIZABLE, 0);
    window = glfwCreateWindow(width2/4, height/2, "MuJoCo VR", NULL, NULL);
    if( !window )
    {
        printf("Could not create GLFW window\n");
        return 0;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);


    // Initialize GLAD after making the GLFW context current
    if (!gladLoadGL(glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return 0;
    }

    // load the model
    char error_msg[1000] = "Could not load XML";
    m = mj_loadXML(MODEL, NULL, error_msg, sizeof(error_msg));
    if (!m) { std::cerr << "Failed to load model: " << error_msg << std::endl; return 0; }
    d = mj_makeData(m);
    if (!d) { mj_deleteModel(m); std::cerr << "Failed to allocate data\n"; return 0; }

    // forward the model once to initialize
    mj_forward(m, d);

    // set offscreen buffer size to match HMD
    m->vis.global.offwidth = width2;
    m->vis.global.offheight = height;
    m->vis.quality.offsamples = 8;

    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 1000);
    mjr_makeContext(m, &con, mjFONTSCALE_100);

    // mj_resetData(m, d);

    // initialize model transform
    scn.enabletransform = 1;
    scn.translate[1] = -0.5;
    scn.translate[2] = -0.5;
    scn.rotate[0] = (float)cos(-0.25*mjPI);
    scn.rotate[1] = (float)sin(-0.25*mjPI);
    scn.scale = 1;

    // stereo mode
    scn.stereo = mjSTEREO_SIDEBYSIDE;

    return 1;
}

// basic implementation just to build for now
int main() {
    // declare OpenVR bridge
    OpenVRBridge vrBridge;
    AllPoses vrPoses;

    std::cout << "MuJoCo VR Integration\n";

    if (!vrBridge.init_vr()) {
        std::cerr << "Failed to initialize OpenVR.\n";
        vrBridge.shutdown_vr();
        return 1;
    }

    std::cout << "VR First Init Complete\n";

    // get the target render size
    std::array<int, 2> renderTargetSize = vrBridge.getRecommendedRenderTargetSize();
    int renderWidth = renderTargetSize[0];
    int renderHeight = renderTargetSize[1];

    std::cout << "Size: " << renderWidth << "x" << renderHeight << "\n";

    if (!initMuJoCo(MODEL_XML, (int)2*renderWidth, (int)renderHeight)) {
        std::cerr << "Failed to initialize MuJoCo.\n";
        vrBridge.shutdown_vr();
        return 1;
    }

    std::cout << "MuJoCo initialized:\n";

    cameraInit(vrBridge, renderWidth, renderHeight);

    std::cout << "Camera initialized:\n";

    // Create both eyes’ buffers
    // createEyeBuffer(leftEyeFBO, leftEyeTex, leftEyeDepth, renderWidth, renderHeight);
    // createEyeBuffer(rightEyeFBO, rightEyeTex, rightEyeDepth, renderWidth, renderHeight);

    const int predict_horizon = 10; // temp for now
    MarkerIds markers = initMarkers(m, predict_horizon);

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

    std::cout << "Starting loop:\n";

    do {


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

                if (vrPoses.hmdPose.valid) { // check if others are vaildS
                    // Map HMD position to target position in simulation
                    convertVRtoMuJoCo(vrPoses.hmdPose);
                    setVRCamPose(vrPoses.hmdPose, 0.061f, vrBridge); // typical IPD ~61mm
                    
                    Eigen::Vector3d hmd_pos(vrPoses.hmdPose.position[0],
                                             vrPoses.hmdPose.position[1],
                                             vrPoses.hmdPose.position[2]);
                    std::cout << "HMD Position: " << hmd_pos.transpose() << std::endl;
                }

                // set the pos of left hand
                if (vrPoses.leftControllerPose.valid) {
                    mjtNum roompos[3]  = { vrPoses.leftControllerPose.position[0], vrPoses.leftControllerPose.position[1], vrPoses.leftControllerPose.position[2] };
                    mjtNum roomquat[4] = { vrPoses.leftControllerPose.orientation[0], vrPoses.leftControllerPose.orientation[1], vrPoses.leftControllerPose.orientation[2], vrPoses.leftControllerPose.orientation[3] };
                    
                    mjtNum modelpos[3];
                    mjtNum modelquat[4];
                    
                    // room to model
                    mjv_room2model(modelpos, modelquat, roompos, roomquat, &scn);

                    // std::cout << "Left Controller Position: " << modelpos[0] << ", " << modelpos[1] << ", " << modelpos[2] << std::endl;
                    setMocapHandPos(d, mj_name2id(m, mjOBJ_BODY, "vr_hand_left"), modelpos, modelquat);
                }

                // set the pos of right hand
                if (vrPoses.rightControllerPose.valid) {
                    mjtNum roompos[3]  = { vrPoses.rightControllerPose.position[0], vrPoses.rightControllerPose.position[1], vrPoses.rightControllerPose.position[2] };
                    mjtNum roomquat[4] = { vrPoses.rightControllerPose.orientation[0], vrPoses.rightControllerPose.orientation[1], vrPoses.rightControllerPose.orientation[2], vrPoses.rightControllerPose.orientation[3] };

                    mjtNum modelpos[3];
                    mjtNum modelquat[4];
                    
                    // room to model
                    mjv_room2model(modelpos, modelquat, roompos, roomquat, &scn);

                    // std::cout << "Right Controller Position: " << modelpos[0] << ", " << modelpos[1] << ", " << modelpos[2] << std::endl;
                    setMocapHandPos(d, mj_name2id(m, mjOBJ_BODY, "vr_hand_right"), modelpos, modelquat);
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

                // 1) update the MuJoCo scene (populate geometry, lights, etc.)
                mjv_updateScene(m, d, &opt, nullptr, nullptr, mjCAT_ALL, &scn);

                // 2) render offscreen into MuJoCo's offscreen buffer (which con.offFBO points to)
                // viewFull should match your offscreen size (2*renderWidth x renderHeight for side-by-side)
                mjrRect viewFull = {0, 0, 2*renderWidth, renderHeight};
                mjr_setBuffer(mjFB_OFFSCREEN, &con);
                glViewport(0, 0, 2*renderWidth, renderHeight);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                mjr_render(viewFull, &scn, &con);

                // render all the cameras
                vr_render(renderWidth, renderHeight);

                // submit frames to VR compositor
                vrBridge.submit_vr_frame(EyeTex);

                // swap if window is double-buffered, flush just in case
                if(con.windowDoublebuffer)
                    glfwSwapBuffers(window);
                glFlush();

                mj_step(m, d);

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
    // glDeleteFramebuffers(1, &leftEyeFBO);
    // glDeleteFramebuffers(1, &rightEyeFBO);
    glDeleteTextures(1, &EyeTex);

    std::cout << "Simulation closed.\n";
    return 0;
}
