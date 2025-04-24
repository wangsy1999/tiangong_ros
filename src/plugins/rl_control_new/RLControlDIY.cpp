/**
 * @file RLControlDIY.cpp
 * @author Siyuan Wang
 * @brief  RLControlDIY 用于在天工lite上实现开放的强化学习控制器
 * @date 2025-04-18
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <bodyctrl_msgs/MotorStatusMsg.h>
#include <bodyctrl_msgs/Imu.h>
#include <bodyctrl_msgs/CmdSetMotorSpeed.h>
#include <bodyctrl_msgs/CmdMotorCtrl.h>
#include <bodyctrl_msgs/MotorName.h>
#include <thread>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <time.h>
#include <fstream>
// #include <fast_ros/fast_ros.h>
#include "broccoli/core/Time.hpp"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "util/LockFreeQueue.h"
#include "../x_humanoid_rl_sdk/include/robot_interface/RobotInterface.h"
// #include "../x_humanoid_rl_sdk/include/robot_FSM/RobotFSM.h"
#include "Joystick.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "interfaceBitbotNet.hpp"
#include "interfaceHeaders.h"


#define DATALOG_MAIN

using namespace broccoli::core;

namespace rl_control_new  // The usage of the namespace is a good practice but not mandatory
{

  enum class ControlState {
    IDLE = 0,
    STAND_INIT = 1,
    WALK = 2,
  };
static Eigen::VectorXd q_d_last = Eigen::VectorXd::Zero(12);

class RLControlDIY : public nodelet::Nodelet {
 public:
 RLControlDIY() {}

 private:
  ControlState control_state = ControlState::IDLE;
  bool LoadConfig(const std::string& config_file) {
    std::ifstream configFile(config_file.c_str());
    if (!configFile.is_open()) {
      std::cout << "Unable to open config file: " << config_file << std::endl;
      return false;
    }
    std::string line;
    while (std::getline(configFile, line)) {
      size_t pos = line.find('=');
      if (pos != std::string::npos) {
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        if (!key.empty()) {
          _config_map[key] = value;
        }
      }
    }
    configFile.close();
    return true;
  }

  double GetConfig(const std::string& key, double default_value) {
    auto it = _config_map.find(key);
    std::string val;
    if (it != _config_map.end()) {
      val = it->second;
    }
    return val.empty() ? default_value : ::atof(val.c_str());
  }

  virtual void onInit() {
    if (!LoadConfig(_config_file)) {
      std::cout << "load config file error: " << _config_file << std::endl;
    }
    auto &nh = getPrivateNodeHandle();
    pi = 3.14159265358979;
    rpm2rps = pi / 30.0;
    motor_num = 20;
    Q_a = Eigen::VectorXd::Zero(motor_num);
    Qdot_a = Eigen::VectorXd::Zero(motor_num);
    Tor_a = Eigen::VectorXd::Zero(motor_num);
    Q_d = Eigen::VectorXd::Zero(motor_num);
    Qdot_d = Eigen::VectorXd::Zero(motor_num);
    Tor_d = Eigen::VectorXd::Zero(motor_num);

    q_a = Eigen::VectorXd::Zero(motor_num);
    qdot_a = Eigen::VectorXd::Zero(motor_num);
    tor_a = Eigen::VectorXd::Zero(motor_num);
    q_d = Eigen::VectorXd::Zero(motor_num);
    qdot_d = Eigen::VectorXd::Zero(motor_num);
    tor_d = Eigen::VectorXd::Zero(motor_num);

    Q_a_last = Eigen::VectorXd::Zero(motor_num);
    Qdot_a_last = Eigen::VectorXd::Zero(motor_num);
    Tor_a_last = Eigen::VectorXd::Zero(motor_num);
    ct_scale = Eigen::VectorXd::Ones(motor_num);
    ct_scale << 2.5, 2.1, 2.5, 2.5, 1.4, 1.4, 
                2.5, 2.1, 2.5, 2.5, 1.4, 1.4, 
                1.4, 1.4, 1.4, 1.4, 
                1.4, 1.4, 1.4, 1.4;
    zero_pos = Eigen::VectorXd::Zero(motor_num);
    std::stringstream ss;
    for (int32_t i = 0; i < motor_num; ++i) {
      zero_pos[i] = GetConfig("zero_pos_" + std::to_string(i), 0.0);
      ss << zero_pos[i] << "  ";
    }
    std::cout << "zero pos: " << ss.str() << std::endl;
    std::cout << "xsense_data_roll: " << GetConfig("xsense_data_roll", 0.0) << std::endl;
    // zero_pos << -2.46139,  -2.54387, -0.649462,  -1.58331,  -2.15858,  0.610552, 
    //             -0.708972,  -2.18147,  0.752079,   2.65526,  2.31079, 3.07412,  
    //             3.2198,   -1.367,   1.5909,  0.8356,
    //             1.70386,  2.5492,   -0.9168, -2.3630; 

    init_pos = Eigen::VectorXd::Zero(motor_num);
    motor_dir = Eigen::VectorXd::Ones(motor_num);
    motor_dir << 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 
                1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 
                1.0, -1.0, -1.0, -1.0, 
                -1.0, -1.0, -1.0, 1.0;
    zero_cnt = Eigen::VectorXd::Zero(motor_num);
    zero_offset = Eigen::VectorXd::Ones(motor_num);
    zero_offset << 0.0, 0.0, -pi/3.0, 2.0*pi/3.0, -1.047, -1.047, 
                  0.0, 0.0, -pi/3.0, 2.0*pi/3.0, -1.047, -1.047, 
                  0.0, 0.2618, 0.0, 0.0, 
                  0.0, -0.2618, 0.0, 0.0;
    kp = Eigen::VectorXd::Ones(motor_num) * 50.0;
    kd = Eigen::VectorXd::Ones(motor_num) * 1.0;

    // 左腿
    kp(0) = 40.0; kd(0) = 4.0;  // Left hip pitch
    kp(1) = 40.0; kd(1) = 4.0;  // Left hip roll
    kp(2) = 30.0; kd(2) = 3.0;  // Left hip yaw
    kp(3) = 50.0; kd(3) = 5.0;  // Left knee
    kp(4) = 35.0; kd(4) = 3.5;  // Left ankle pitch
    kp(5) = 35.0; kd(5) = 3.5;  // Left ankle roll

    // 右腿
    kp(6) = 40.0; kd(6) = 4.0;
    kp(7) = 40.0; kd(7) = 4.0;
    kp(8) = 30.0; kd(8) = 3.0;
    kp(9) = 50.0; kd(9) = 5.0;
    kp(10) = 35.0; kd(10) = 3.5;
    kp(11) = 35.0; kd(11) = 3.5;

  
    imu_data = Eigen::VectorXd::Zero(9);
    imu_raw_data = Eigen::VectorXd::Zero(9);
    xsense_data = Eigen::VectorXd::Zero(13);
    data = Eigen::VectorXd::Zero(350);

    motor_name.insert({0, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_1});
    motor_name.insert({1, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_2});
    motor_name.insert({2, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_3});
    motor_name.insert({3, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_4});
    motor_name.insert({4, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_5});
    motor_name.insert({5, bodyctrl_msgs::MotorName::MOTOR_LEG_LEFT_6});
    motor_name.insert({6, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_1});
    motor_name.insert({7, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_2});
    motor_name.insert({8, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_3});
    motor_name.insert({9, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_4});
    motor_name.insert({10, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_5});
    motor_name.insert({11, bodyctrl_msgs::MotorName::MOTOR_LEG_RIGHT_6});
    motor_name.insert({12, bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_1});
    motor_name.insert({13, bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_2});
    motor_name.insert({14, bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_3});
    motor_name.insert({15, bodyctrl_msgs::MotorName::MOTOR_ARM_LEFT_4});
    motor_name.insert({16, bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_1});
    motor_name.insert({17, bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_2});
    motor_name.insert({18, bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_3});
    motor_name.insert({19, bodyctrl_msgs::MotorName::MOTOR_ARM_RIGHT_4});

    for (int i = 0; i < motor_num; i++) {
      motor_id.insert({motor_name[i], i});
    }

    // auto fnh = fast_ros::NodeHandle(nh);

    pubSetMotorCmd = nh.advertise<bodyctrl_msgs::CmdMotorCtrl>("/BodyControl/motor_ctrl", 1000);
    subState = nh.subscribe("/BodyControl/motor_state", 1000, &RLControlDIY::OnMotorStatusMsg, this);
    // subImu = fnh.subscribe("/BodyControl/imu", 1000, &RLControlNewPlugin::OnImuStatusMsg, this, fast_ros::ConnectionType::NATIVE_ROS);
    // subImuXsens = fnh.subscribe("/XSensImu/imu", 1000, &RLControlNewPlugin::OnXsensImuStatusMsg, this, fast_ros::ConnectionType::NATIVE_ROS);
    subImuXsens = nh.subscribe("/BodyControl/imu", 1000, &RLControlDIY::OnXsensImuStatusMsg, this);
    subJoyCmd = nh.subscribe<sensor_msgs::Joy>("/sbus_data", 1000, &RLControlDIY::xbox_map_read, this);
    subCmdVel = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, &RLControlDIY::OnCmdVelMsg, this);

    sleep(1);
    std::thread([this]() {
      rlControl_rlOnly();
    }).detach();
  }

  // 混合模式测试
  void rlControl_rlOnly() {
    static Dcc::INTERFACE_BITBOT::st_interface g_rl_interface;
    static Dcc::INTERFACE_BITBOT::c_interface rl_controller(&g_rl_interface);
    rl_controller.init();

    Joystick_humanoid joystick_humanoid;
    joystick_humanoid.init();

    RobotInterface* robot_interface = get_robot_interface();
    robot_interface->Init();

    Eigen::VectorXd init_pos_leg(12);
    init_pos_leg << 0.0, 0.0, -0.165, 0.53, -0.30, 0.0,
                    0.0, 0.0, -0.165, 0.53, -0.30, 0.0;

    long count = 0;
    double t_now = 0;
    double dt = 0.001;
    const double torque_limit = 100.0;  // 例如最大允许力矩为 60 Nm，根据实际电机能力调整
    while (true) {
      while (!queueMotorState.empty()) {
        auto msg = queueMotorState.pop();
        for (auto& one : msg->status) {
          int id = motor_id[one.name];
          Q_a(id) = one.pos;
          Qdot_a(id) = one.speed;
          Tor_a(id) = one.current * ct_scale(id);
        }
      }

      while (!queueImuXsens.empty()) {
        auto msg = queueImuXsens.pop();
        xsense_data(0) = msg->euler.yaw;
        xsense_data(1) = msg->euler.pitch;
        xsense_data(2) = msg->euler.roll;
        xsense_data(3) = msg->angular_velocity.x;
        xsense_data(4) = msg->angular_velocity.y;
        xsense_data(5) = msg->angular_velocity.z;
        xsense_data(6) = msg->linear_acceleration.x;
        xsense_data(7) = msg->linear_acceleration.y;
        xsense_data(8) = msg->linear_acceleration.z;
      }

      #ifdef USE_ROS_JOY
      while (!queueJoyCmd.empty()) {
        auto msg = queueJoyCmd.pop();
        xbox_map.a = msg->axes[8];
        xbox_map.b = msg->axes[9];
        xbox_map.c = msg->axes[10];
        xbox_map.d = msg->axes[11];
        xbox_map.e = msg->axes[4];
        xbox_map.f = msg->axes[7];
        xbox_map.g = msg->axes[5];
        xbox_map.h = msg->axes[6];
        xbox_map.x1 = msg->axes[3];
        xbox_map.x2 = msg->axes[0];
        xbox_map.y1 = msg->axes[2];
        xbox_map.y2 = msg->axes[1];
      }
      #endif

      t_now = count * dt;

      for (int i = 0; i < motor_num; ++i) {
        q_a(i) = (Q_a(i) - zero_pos(i)) * motor_dir(i) + zero_offset(i);
        q_a(i) += zero_cnt(i) * 2.0 * pi;
        qdot_a(i) = Qdot_a(i) * motor_dir(i);
        tor_a(i) = Tor_a(i) * motor_dir(i);
      }

      xbox_flag flag_ = joystick_humanoid.get_xbox_flag();
      g_rl_interface.input.time = t_now;
      for (int i = 0; i < 9; ++i) g_rl_interface.input.imu[i] = xsense_data(i);
      g_rl_interface.input.flag = flag_;
      for (int i = 0; i < 12; ++i) {
        g_rl_interface.input.jntI.q[i] = q_a(i);
        g_rl_interface.input.jntI.qdot_a[i] = qdot_a(i);
        g_rl_interface.input.kp[i] = kp(i);
        g_rl_interface.input.kd[i] = kd(i);
      }
      g_rl_interface.input.commands[0] = xbox_map.x2;
      g_rl_interface.input.commands[1] = xbox_map.y2;
      g_rl_interface.input.commands[2] = xbox_map.x1;

      // === 状态切换逻辑 ===
      if (xbox_map.a) {
        control_state = ControlState::STAND_INIT;
        std::cout << "[FSM] Switch to STAND_INIT" << std::endl;
      }
      if (xbox_map.b) {
        control_state = ControlState::IDLE;
        std::cout << "[FSM] Switch to IDLE" << std::endl;
      }
      if (xbox_map.c) {
        control_state = ControlState::WALK;
        std::cout << "[FSM] Switch to WALK" << std::endl;
      }

      switch (control_state) {
        case ControlState::IDLE:
          for (int i = 0; i < 12; ++i) {
            g_rl_interface.output.jntO.q[i] = q_a(i);
            g_rl_interface.output.jntO.qdot_d[i] = 0.0;
            g_rl_interface.output.jntO.tor[i] = 0.0;
          }
          break;
        case ControlState::STAND_INIT:
          for (int i = 0; i < 12; ++i) {
            g_rl_interface.output.jntO.q[i] = init_pos_leg(i);
            g_rl_interface.output.jntO.qdot_d[i] = 0.0;
            g_rl_interface.output.jntO.tor[i] = 0.0;
          }
          break;
        case ControlState::WALK:
          rl_controller.run();
          break;
      }

      for (int i = 0; i < 11; i++) {
        robot_data.q_d_(i) = g_rl_interface.output.jntO.q[i];
        robot_data.q_dot_d_(i) = g_rl_interface.output.jntO.qdot_d[i];
        robot_data.tau_d_(i) = g_rl_interface.output.jntO.tor[i];
        // === 力矩保护 ===
        if (robot_data.tau_d_(i) > torque_limit)
            robot_data.tau_d_(i) = torque_limit;
        else if (robot_data.tau_d_(i) < -torque_limit)
            robot_data.tau_d_(i) = -torque_limit;
      }


      bodyctrl_msgs::CmdMotorCtrl msg;
      for (int i = 0; i < 11; i++) {
        bodyctrl_msgs::MotorCtrl cmd;
        cmd.name = motor_name[i];
        cmd.kp = kp(i);
        cmd.kd = kd(i);
        cmd.pos = (robot_data.q_d_(i) - zero_offset(i) - zero_cnt(i) * 2.0 * pi) * motor_dir(i) + zero_pos(i);
        cmd.spd = robot_data.q_dot_d_(i) * motor_dir(i);
        cmd.tor = robot_data.tau_d_(i) * motor_dir(i);
        msg.header.stamp = ros::Time::now();
        msg.cmds.push_back(cmd);
      }
      for (int i = 12; i < 19; i++) {
        bodyctrl_msgs::MotorCtrl cmd;
        cmd.name = motor_name[i];
        cmd.pos = (0 - zero_offset(i) - zero_cnt(i) * 2.0 * pi) * motor_dir(i) + zero_pos(i);
        cmd.spd = robot_data.q_dot_d_(i) * motor_dir(i);
        msg.header.stamp = ros::Time::now();
        msg.cmds.push_back(cmd);
      }
      pubSetMotorCmd.publish(msg);

      for (int i = 0; i < 12; ++i) {
        q_d_last(i) = g_rl_interface.output.jntO.q[i];
      }
      count++;
    }
  }



  void OnMotorStatusMsg(const bodyctrl_msgs::MotorStatusMsg::ConstPtr &msg) {
    auto wrapper = msg;
    queueMotorState.push(wrapper);
  }

  void OnImuStatusMsg(const bodyctrl_msgs::Imu::ConstPtr &msg) {
    auto wrapper = msg;
    queueImuRm.push(wrapper);
  }

  void OnXsensImuStatusMsg(const bodyctrl_msgs::Imu::ConstPtr& msg)
  {
    auto wrapper = msg;
    queueImuXsens.push(wrapper);
  }

  void xbox_map_read(const sensor_msgs::Joy::ConstPtr &msg)
  {
    auto wrapper = msg;
    queueJoyCmd.push(wrapper);
  }

  void OnCmdVelMsg(const geometry_msgs::Twist::ConstPtr& msg){
    auto wrapper = msg;
    queueCmdVel.push(wrapper);
  }

  ros::Subscriber subState;
  // fast_ros::Subscriber subImu, subImuXsens;
  ros::Subscriber subImu, subImuXsens;
  ros::Publisher pubSetMotorCmd;
  ros::Subscriber subJoyCmd;
  ros::Subscriber subCmdVel;
  Eigen::VectorXd q_a;
  Eigen::VectorXd qdot_a;
  Eigen::VectorXd tor_a;
  Eigen::VectorXd q_d;
  Eigen::VectorXd qdot_d;
  Eigen::VectorXd tor_d;
  Eigen::VectorXd Q_a;
  Eigen::VectorXd Qdot_a;
  Eigen::VectorXd Tor_a;
  Eigen::VectorXd Q_a_last;
  Eigen::VectorXd Qdot_a_last;
  Eigen::VectorXd Tor_a_last;
  Eigen::VectorXd Q_d;
  Eigen::VectorXd Qdot_d;
  Eigen::VectorXd Tor_d;
  Eigen::VectorXd ct_scale;
  Eigen::VectorXd data;
  Eigen::VectorXd zero_pos;
  Eigen::VectorXd zero_offset;
  Eigen::VectorXd init_pos;
  Eigen::VectorXd motor_dir;
  Eigen::VectorXd zero_cnt;
  Eigen::VectorXd imu_raw_data;
  Eigen::VectorXd imu_data;
  Eigen::VectorXd xsense_data;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
  double x_speed_command = 0.;  
  double y_speed_command = 0.;  
  double yaw_speed_command = 0.;
  std::string _config_file = "/home/ubuntu/data/param/rl_control_new.txt";
  std::unordered_map<std::string, std::string> _config_map;

  #ifdef USE_ROS_JOY
    xbox_map_t xbox_map;
  #endif

  int motor_num;
  std::map<int, int> motor_id, motor_name;
  float rpm2rps;
  float pi;

  LockFreeQueue<bodyctrl_msgs::MotorStatusMsg::ConstPtr> queueMotorState;
  LockFreeQueue<bodyctrl_msgs::Imu::ConstPtr> queueImuRm;
  LockFreeQueue<bodyctrl_msgs::Imu::ConstPtr> queueImuXsens;
  LockFreeQueue<sensor_msgs::Joy::ConstPtr> queueJoyCmd;
  LockFreeQueue<geometry_msgs::Twist::ConstPtr> queueCmdVel;

  // robot_data
  RobotData robot_data;
};
}

PLUGINLIB_EXPORT_CLASS(rl_control_new::RLControlDIY, nodelet::Nodelet
);
