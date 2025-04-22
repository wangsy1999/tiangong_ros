// interfaceBitbotNet.hpp
// 20250417 dcc <3120195094@bit.edu.cn>
// This file is part of the BitBot project to cook data from/to inference net.


#pragma once
#ifndef INTERFACE_BITBOT_HPP
#define INTERFACE_BITBOT_HPP
#include "interfaceHeaders.h"
#include <cmath>  
#include "eigen3/Eigen/Dense"
#include "parallel_ankle.hpp"

_D_INTERFACE_BEGIN

struct st_jointsI {
    double q[_JNT_NUM];
    double qdot_a[_JNT_NUM];
    double tor[_JNT_NUM];
};
struct st_jointsO {
    double q[_JNT_NUM];
    double qdot_d[_JNT_NUM];
    double tor[_JNT_NUM];
};
struct st_interfaceInput {
    double imu[9];
    double commands[3];
    double kp[_JNT_NUM];
    double time;
    xbox_flag flag;
    st_jointsI jntI;
};

struct st_interfaceOutput {
    st_jointsO jntO;
};

struct st_interface {
    st_interfaceInput input;
    st_interfaceOutput output;
};

// init the net: 把所有user_function.cpp里和网络部署相关的构建都放在这里
float joint_target_position[2][6] = {0};
float joint_target_torque[2][6] = {0};
float joint_current_position[2][6] = {0};
float joint_current_velocity[2][6] = {0};
float motor_target_position[2][6] = {0};
float motor_target_torque[2][6] = {0};
float motor_current_position[2][6] = {0};
float motor_current_velocity[2][6] = {0};
float joint[2][6] = {0}; 

float p_gains[2][6] = {0};
float d_gains[2][6] = {0};
constexpr float deg2rad = M_PI / 180.0;

std::vector<std::vector<float>> action_interpolated;
std::array<std::array<float, 6>, 2> default_position = {
    {{ 0.0, 0, -0.165647, 0.529741, -0.301101, 0.},    // left
     { 0.0, 0, -0.165647, 0.529741, -0.301101, 0.}}};  // right

const float stance_T = 0.375;
const float dt = 0.0025;
Eigen::Matrix<float, 3, 1> projected_gravity_mat;
const int policy_frequency = 10;
float gravity_vec[3] = {0.0, 0.0, -1};

std::vector<float> predicted_lin_velo(3);  // lin velo predicted by SE
Eigen::Vector3f inekf_predict_lin_velo;
Eigen::Matrix3f inekf_predict_pos;
Eigen::Vector3f inekf_predict_Proj_grav;
std::array<bool, 2> inekf_predict_contact_status;

Eigen::Matrix<float, 3, 1> CoMAngleVeloMat;
Eigen::Matrix<float, 3, 1> CoMVeloMat;
Eigen::Matrix<float, 3, 1> gravity_vec_mat;

std::vector<float> commands = {0.0, 0.0, 0.0};
float CoM_angle[3] = {0};        // CoM rpy orientation
float CoM_linear_velo[3] = {0};  // CoM linear velocity
float CoM_angle_velo[3] = {0};   // CoM angular velocity
float CoM_acc[3] = {0};       
float clock_input[2] = {0};

std::vector<float> dof_pos_obs(12);
std::vector<float> dof_vel_obs(12);
std::vector<float> last_action(12);
const float torque_compensate[2][6] = {{0.0, 0.0, 0.0, 0., 0., 0.},
                                       {0.0, 0.0, 0.0, 0., 0., 0.}};
const float compensate_threshold[2][6] = {
    {1.0, 1.0, 1.0, 1.0, 1., 1.},
    {1.0, 1.0, 1.0, 1.0, 1., 1.}};  // In rads

int control_periods = 0;
size_t run_ctrl_cnt = 0;
size_t start_delay_cnt = 500; 

ParallelAnkle<float> left_ankle({.l_bar1 = 0.04,
                                 .l_rod1 = 0.2405,  // 0.235
                                 .r_a1 = {-0.007828, 0.06565, 0.23257},
                                 .r_b1_0 = {-0.047244, 0.06565, 0.23257},
                                 .r_c1_0 = {-0.04432, 0.06565, -0.00824},
                                 .l_bar2 = 0.04,
                                 .l_rod2 = 0.1578,
                                 .r_a2 = {-0.007828, -0.06565, 0.15002},
                                 .r_b2_0 = {-0.047244, -0.06565, 0.15002},
                                 .r_c2_0 = {-0.04432, -0.06565, -0.00824}},
                                1e-6);

ParallelAnkle<float> right_ankle({.l_bar1 = 0.04,
                                  .l_rod1 = 0.1578,
                                  .r_a1 = {-0.007828, 0.06565, 0.15002},
                                  .r_b1_0 = {-0.047244, 0.06565, 0.15002},
                                  .r_c1_0 = {-0.04432, 0.06565, -0.00824},
                                  .l_bar2 = 0.04,
                                  .l_rod2 = 0.2405,
                                  .r_a2 = {-0.007828, -0.06565, 0.23257},
                                  .r_b2_0 = {-0.047244, -0.06565, 0.23257},
                                  .r_c2_0 = {-0.04432, -0.06565, -0.00824}},
                                 1e-6);

class filter {
    public:
     filter(int buffer_sz = 3) {
       this->buffer_sz = buffer_sz;
       this->buffer.resize(buffer_sz);
       for (auto &i : this->buffer) i = 0;
     }
     void clear() {
       for (auto &i : this->buffer) {
         i = 0;
       }
     }
   
     float operator()(float input) {
       buffer[this->pointer] = input;
       this->pointer++;
       this->pointer %= this->buffer_sz;
       float sum = 0;
       for (auto &i : this->buffer) {
         sum += i;
       }
       return sum / buffer_sz;
     }
   
    private:
     int buffer_sz;
     std::vector<float> buffer;
     int pointer = 0;
   };
// Velocity Average Filter
filter velo_filters[2][6];
filter velo_filter_net[2][6];

float_inference_net::Ptr inference_net;
float_inference_net::NetConfigT net_config =
    {
        .input_config = {.obs_scales_ang_vel = 1.0,
                         .obs_scales_lin_vel = 2.0,
                         .scales_commands = 1.0,
                         .obs_scales_dof_pos = 1.0,
                         .obs_scales_dof_vel = 0.05,
                         .obs_scales_euler = 1.0,
                         .clip_observations = 18.0,
                         .ctrl_model_input_size = 15 * 47,
                         .stack_length = 15,
                         .ctrl_model_unit_input_size = 47},
        .output_config =
            {.ctrl_clip_action = 18.0,
             .action_scale = 0.5,
             .ctrl_action_lower_limit =
                 {
                     -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30,
                 },
             .ctrl_action_upper_limit =
                 {
                     30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30
                 },
             .ctrl_model_output_size = 12},
        .action_default_pos = {
            default_position[0][0],
            default_position[0][1],
            default_position[0][2],
            default_position[0][3],
            default_position[0][4],  // Left ank pit
            default_position[0][5],  // Left ank roll
            default_position[1][0],
            default_position[1][1],
            default_position[1][2],
            default_position[1][3],
            default_position[1][4],
            default_position[1][5],
        }};


class c_interface {

public:
    inline c_interface(st_interface * interface) {
        this->m_interface = interface;
    }
    // init the net
    void init() {
        this->InitPolicy();
    }
    // run the control loop
    void run() {
        // get observation: 算kf，得到关节端observation，imu等其他量在PolicyController里对齐
        this->GetJointObservation();

        // run the policy controller loop: 算出了关节端action输出
        this->PolicyController(this->m_interface->input.time);

        // get torque: 算ik，得到电机端目标
        this->SetJointAction();

        // set command: 电机端目标与天工的格式对齐
        this->GiveJointAction2Interface();
    }

private:
    st_interface * m_interface;
    double q_d_last[_JNT_NUM] = {0.0};
    // init the net
    void InitPolicy() {
        // Action interpolation buffer
        action_interpolated = std::vector<std::vector<float>>(policy_frequency + 1,
                                                              std::vector<float>(12));
        // Create the policy network instance
        // last 4000
        inference_net = std::make_unique<float_inference_net>(
            "/home/wsy/robot/tiangong/src/rl_control_new/src/plugins/x_humanoid_rl_sdk/python_scripts"
            "policy_1.pt",     // control model  policy_202412271
            net_config,        // net config
            true,              // use async mode
            policy_frequency);  // policy frequency

    }
    // Policy controller loop: 还没替换
    void PolicyController(uint64_t cur_time) {
        CoM_angle[0] = this->m_interface->input.imu[2];  // 正确写法
//imu->GetRoll();  // imu angle
        CoM_angle[1] = this->m_interface->input.imu[1];  // 正确写法
//imu->GetPitch();
        CoM_angle[2] = this->m_interface->input.imu[0];
      
        // This is sort of filter.
        // Acceleration from IMU
        CoM_acc[0] = (std::abs(this->m_interface->input.imu[6]) > 50) 
             ? CoM_acc[0] 
             : this->m_interface->input.imu[6];
//imu->GetAccX();
        CoM_acc[1] = (std::abs(this->m_interface->input.imu[7]) > 50) 
        ? CoM_acc[1] 
        : this->m_interface->input.imu[8];
        //imu->GetAccY();
        CoM_acc[2] = (std::abs(this->m_interface->input.imu[8]) > 50) 
        ? CoM_acc[2] 
        : this->m_interface->input.imu[8];
        //imu->GetAccZ();
        
        
        
        float angle_velo_x = this->m_interface->input.imu[3];//imu->GetGyroX();  

        // Angular velocity from IMU
        float angle_velo_y = this->m_interface->input.imu[4];//imu->GetGyroY();
        float angle_velo_z = this->m_interface->input.imu[5];//imu->GetGyroZ();
      
        if (std::isnan(angle_velo_x)) angle_velo_x = 0;
        if (std::isnan(angle_velo_y)) angle_velo_y = 0;
        if (std::isnan(angle_velo_z)) angle_velo_z = 0;
    
        for (size_t i = 0; i < 3; i++) {
        if (std::isnan(CoM_angle[i])) CoM_angle[i] = 0;
        }
    
        CoM_angle_velo[0] = 
            std::abs(angle_velo_x) > 10 ? CoM_angle_velo[0] : angle_velo_x;  // filter the noise
        CoM_angle_velo[1] =
            std::abs(angle_velo_y) > 10 ? CoM_angle_velo[1] : angle_velo_y;
        CoM_angle_velo[2] =
            std::abs(angle_velo_z) > 10 ? CoM_angle_velo[2] : angle_velo_z;
        Eigen::Vector3f angle_velo_eigen(CoM_angle_velo[0], CoM_angle_velo[1],
                                        CoM_angle_velo[2]);
    
        // inference
        clock_t start_time, end_time;
        start_time = this->m_interface->input.time;

        static uint64_t init_policy_time = cur_time;
        if ((cur_time - init_policy_time) % policy_frequency == 0) {
            /*const float stance_T = 0.32;*/
            /*const float stance_T = 0.40;*/

        
            for (int i = 0; i < 2; i++) {
                clock_input[i] +=
                    2 * M_PI / (2 * stance_T) * dt * static_cast<float>(policy_frequency);
                if (clock_input[i] > 2 * M_PI) {
                clock_input[i] -= 2 * M_PI;
                }
            }
        
            Eigen::Matrix3f RotationMatrix;
            const Eigen::AngleAxisf roll(CoM_angle[0], Eigen::Vector3f::UnitX());
            const Eigen::AngleAxisf pitch(CoM_angle[1], Eigen::Vector3f::UnitY());
            const Eigen::AngleAxisf yaw(CoM_angle[2], Eigen::Vector3f::UnitZ());
            RotationMatrix = yaw * pitch * roll;
        
            std::vector<float> eu_ang{CoM_angle[0], CoM_angle[1], CoM_angle[2]};
        
            CoMAngleVeloMat << CoM_angle_velo[0], CoM_angle_velo[1], CoM_angle_velo[2];
            gravity_vec_mat << gravity_vec[0], gravity_vec[1], gravity_vec[2];
        
            projected_gravity_mat = RotationMatrix.transpose() * gravity_vec_mat;
        
            std::vector<float> base_ang_vel = {
                CoMAngleVeloMat(0, 0), CoMAngleVeloMat(1, 0), CoMAngleVeloMat(2, 0)};
            std::vector<float> project_gravity = {projected_gravity_mat(0, 0),
                                                    projected_gravity_mat(1, 0),
                                                    projected_gravity_mat(2, 0)};
        
            std::vector<float> clock_input_vec = {float(sin(clock_input[0])),
                                                    float(cos(clock_input[0]))};
            auto& input = this->m_interface->input;




            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 6; ++j) {
                    int idx = i * 6 + j;
                    joint_current_position[i][j] = static_cast<float>(input.jntI.q[idx]);
                    joint_current_velocity[i][j] = static_cast<float>(input.jntI.qdot_a[idx]);
                }
            }
            
            dof_pos_obs[0] = joint_current_position[0][0];  // Left hip pitch
            dof_pos_obs[1] = joint_current_position[0][1];  // Left hip roll
            dof_pos_obs[2] = joint_current_position[0][2];  // Left hip yaw
            dof_pos_obs[3] = joint_current_position[0][3];  // Left knee
            dof_pos_obs[4] = joint_current_position[0][4];  // Left ankle pitch
            dof_pos_obs[5] = joint_current_position[0][5];  // Left ankle roll
            //
            dof_pos_obs[6] = joint_current_position[1][0];   // Right hip pitch
            dof_pos_obs[7] = joint_current_position[1][1];   // Right hip roll
            dof_pos_obs[8] = joint_current_position[1][2];   // Right hip yaw
            dof_pos_obs[9] = joint_current_position[1][3];   // Right knee
            dof_pos_obs[10] = joint_current_position[1][4];  // Right ankle pitch
            dof_pos_obs[11] = joint_current_position[1][5];  // Right ankle roll
        
            // Add filter
            dof_vel_obs[0] =
                velo_filter_net[0][0](joint_current_velocity[0][0]);  // Left hip pitch
            dof_vel_obs[1] =
                velo_filter_net[0][1](joint_current_velocity[0][1]);  // Left hip roll
            dof_vel_obs[2] =
                velo_filter_net[0][2](joint_current_velocity[0][2]);  // Left hip yaw
            dof_vel_obs[3] =
                velo_filter_net[0][3](joint_current_velocity[0][3]);  // Left knee
            dof_vel_obs[4] = velo_filter_net[0][4](
                joint_current_velocity[0][4]);  // Left ankle pitch
            dof_vel_obs[5] =
                velo_filter_net[0][5](joint_current_velocity[0][5]);  // Left ankle roll
            //
            dof_vel_obs[6] =
                velo_filter_net[1][0](joint_current_velocity[1][0]);  // Right hip pitch
            dof_vel_obs[7] =
                velo_filter_net[1][1](joint_current_velocity[1][1]);  // Right hip roll
            dof_vel_obs[8] =
                velo_filter_net[1][2](joint_current_velocity[1][2]);  // Right hip yaw
            dof_vel_obs[9] =
                velo_filter_net[1][3](joint_current_velocity[1][3]);  // Right knee
            dof_vel_obs[10] = velo_filter_net[1][4](
                joint_current_velocity[1][4]);  // Right ankle pitch
            dof_vel_obs[11] = velo_filter_net[1][5](
                joint_current_velocity[1][5]);  // Right ankle roll
        
            // Euler angle
            /*bool ok = inference_net->InferenceOnceErax(*/
            /*    clock_input_vec, commands, dof_pos_obs, dof_vel_obs, last_action,*/
            /*    base_ang_vel, eu_ang);*/
            // Projected gravity
            
            commands = std::vector<float>(input.commands, input.commands + 3);

            bool ok = inference_net->InferenceOnceErax(
                clock_input_vec, commands, dof_pos_obs, dof_vel_obs, last_action,
                base_ang_vel, project_gravity);
        
            if (!ok) {
                std::cout << "inference failed" << std::endl;
            }
        }
        
        // get inference result when inference finished
        if (auto inference_status = inference_net->GetStatus();
            inference_status == float_inference_net::StatusT::FINISHED) {
            auto ok =
                inference_net->GetInfereceResult(last_action, action_interpolated);

            if (!ok) {
                std::cout << "get inference result failed" << std::endl;
            }
            control_periods = 0;  // reset control periods when new target is generated
        
            // Log data into policy logger
            inference_net->log_result();
        }
    
        if (run_ctrl_cnt > start_delay_cnt) {
            
            joint_target_position[0][0] =
                action_interpolated[control_periods][0];  // Left hip yaw
            joint_target_position[0][1] =
                action_interpolated[control_periods][1];  // Left hip roll
            joint_target_position[0][2] =
                action_interpolated[control_periods][2];  // Left hip pitch
            joint_target_position[0][3] =
                action_interpolated[control_periods][3];  // Left knee
            joint_target_position[0][4] =
                action_interpolated[control_periods][4];  // Left ankle pitch
            joint_target_position[0][5] =
                action_interpolated[control_periods][5];  // Left ankle roll
        
            
            joint_target_position[1][0] =
                action_interpolated[control_periods][6];  // Right hip yaw
            joint_target_position[1][1] =
                action_interpolated[control_periods][7];  // Right hip roll
            joint_target_position[1][2] =
                action_interpolated[control_periods][8];  // Right hip pitch
            joint_target_position[1][3] =
                action_interpolated[control_periods][9];  // Right knee
            joint_target_position[1][4] =
                action_interpolated[control_periods][10];  // Right ankle pitch
            joint_target_position[1][5] =
                action_interpolated[control_periods][11];  // Right ankle roll
        
        } else {
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 6; j++) {
                joint_target_position[i][j] = joint_target_position[i][j];
                }
        }
    
        control_periods += 1;
        control_periods =
            (control_periods > policy_frequency) ? policy_frequency : control_periods;
    
        end_time = this->m_interface->input.time;

        if (((float)(end_time - start_time) / CLOCKS_PER_SEC) > 0.0025) {
        std::cout << "Calling time: "
                    << (float)(end_time - start_time) / CLOCKS_PER_SEC << "s"
                    << std::endl;
        }
    
        TorqueController();
        run_ctrl_cnt++;
    }

    


// 调用解算
void GetJointObservation() {
    auto& input = this->m_interface->input;



    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 6; ++j) {
            int idx = i * 6 + j;
            joint_current_position[i][j] = static_cast<float>(input.jntI.q[idx]);
            joint_current_velocity[i][j] = static_cast<float>(input.jntI.qdot_a[idx]);
        }
    }
    

    auto left_res = left_ankle.ForwardKinematics(motor_current_position[0][4],
                                                 motor_current_position[0][5]);
    auto right_res = right_ankle.ForwardKinematics(motor_current_position[1][5],
                                                   motor_current_position[1][4]);

    auto left_vel_res = left_ankle.VelocityMapping(motor_current_velocity[0][4],
                                                   motor_current_velocity[0][5]);
    auto right_vel_res = left_ankle.VelocityMapping(motor_current_velocity[1][5],
                                                    motor_current_velocity[1][4]);

    joint_current_position[0][4] = left_res(0, 0);
    joint_current_position[0][5] = left_res(1, 0);
    joint_current_position[1][4] = right_res(0, 0);
    joint_current_position[1][5] = right_res(1, 0);

    joint_current_velocity[0][4] = left_vel_res(0, 0);
    joint_current_velocity[0][5] = left_vel_res(1, 0);
    joint_current_velocity[1][4] = right_vel_res(0, 0);
    joint_current_velocity[1][5] = right_vel_res(1, 0);
    }

    // joint pk control
    void TorqueController() {
        // Logger_More.startLog();
      
        // Compute torque for non-wheel joints.
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 6; j++) {
                float position_error = joint_target_position[i][j] -
                                    joint_current_position[i][j];  // position error
                /*joint_current_velocity[i][j] = velo_filters[i][j](*/
                /*    joint_current_velocity[i][j]);  // filter the velocity*/
        
                joint_target_torque[i][j] = p_gains[i][j] * position_error;  // P
                joint_target_torque[i][j] +=
                    -d_gains[i][j] * joint_current_velocity[i][j];  // D
        
                // Torque compensation is zero...
                if (abs(joint_target_position[i][j] - joint_current_position[i][j]) >
                    compensate_threshold[i][j] * deg2rad) {
                /*std::cout << "Compensate!!!" << std::endl;*/
                joint_target_torque[i][j] +=
                    torque_compensate[i][j] *
                    (joint_target_position[i][j] - joint_current_position[i][j]) /
                    abs((joint_target_position[i][j] - joint_current_position[i][j]));
                }
            }
      
        //log_result(0);
      
        // SetJointAction(); // 在外面搞
        // SetJointTorque_User(); // bitbot框架的关节力矩下发，这里不用
    }
    // ik and fk: 还没替换
    void SetJointAction() {
        // For normal joints
        for (size_t i = 0; i < 2; i++) {
            for (size_t j = 0; j < 6; j++) {
                if (j == 4 || j == 5) continue;
                motor_target_position[i][j] = joint_target_position[i][j];
                motor_target_torque[i][j] = joint_target_torque[i][j];
            }
        }
      
        // For ankle joints
        auto left_mot_pos_res = left_ankle.InverseKinematics(
            joint_target_position[0][4], joint_target_position[0][5]);
        auto right_mot_pos_res = right_ankle.InverseKinematics(
            joint_target_position[1][4], joint_target_position[1][5]);
        motor_target_position[0][4] = -left_mot_pos_res[0];
        motor_target_position[0][5] = -left_mot_pos_res[1];
        motor_target_position[1][4] = -right_mot_pos_res[1];
        motor_target_position[1][5] = -right_mot_pos_res[0];
      
        auto left_tor_res = left_ankle.TorqueRemapping(joint_target_torque[0][4],
                                                       joint_target_torque[0][5]);
        auto right_tor_res = right_ankle.TorqueRemapping(joint_target_torque[1][4],
                                                         joint_target_torque[1][5]);
        motor_target_torque[0][4] = -left_tor_res[0];
        motor_target_torque[0][5] = -left_tor_res[1];
        motor_target_torque[1][4] = -right_tor_res[1];
        motor_target_torque[1][5] = -right_tor_res[0];
    }

// ======================================================================
//  把电机侧目标 (motor_target_position / torque) 打包到接口输出格式
// ======================================================================
void GiveJointAction2Interface()
{
    // 把两条腿 2×6 = 12 个目标打到连续数组中
    for (int leg = 0; leg < 2; ++leg)
    {
        for (int j = 0; j < 6; ++j)
        {
            const int idx = leg * 6 + j;              // 线性索引
            const double q_des   = motor_target_position[leg][j];
            const double tau_des = motor_target_torque  [leg][j];

            // 1) 期望位置
            m_interface->output.jntO.q[idx] = q_des;

            // 2) 期望速度 —— 用上一拍差分即可满足外层 robot_data.q_dot_d_
            const double qdot_des = (q_des - q_d_last[idx]) / dt;
            m_interface->output.jntO.qdot_d[idx] = qdot_des;

            // 3) 期望力矩
            m_interface->output.jntO.tor[idx] = tau_des;

            // // 缓存本拍角度，供下一拍差分
            // q_d_last[idx] = q_des;
        }
    }
}

    
};

_D_INTERFACE_END

#endif