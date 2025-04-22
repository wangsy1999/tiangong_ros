/**
 * @file ImuProcessWorker.hpp
 * @author Zishun Zhou
 * @brief
 *
 * @date 2025-03-10
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include "AbstractWorker.hpp"
#include "../Schedulers/AbstractScheduler.hpp"
#include "../Utils/StaticStringUtils.hpp"
#include "../Utils/ZenBuffer.hpp"
#include "../Utils/MathTypes.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Utils/MathTypes.hpp"

#include <iostream>
#include <memory>

namespace z
{
    /**
     * @brief ImuProcessWorker 类型是一个IMU数据处理工人类型，这个类型用于处理IMU传感器的数据，包括加速度，角速度和角度。
     * 通常来说，这个类型可以被用于主任务队列中。
     * 这个类会在TaskCycleBegin方法中获取IMU数据并对齐进行滤波和去除异常值。用户可以通过配置文件来配置滤波器的权重。
     * @details
     * 该类会要求数据总线中包含"AccelerationRaw","AngleVelocityRaw","AngleRaw"这三个数据用于存储IMU传感器的原始数据。
     * 该类会在数据总线中存储"AccelerationValue","AngleVelocityValue","AngleValue"这三个数据用于存储滤波处理后的IMU数据。
     *
     * @details config.json配置文件示例：
     * { \n
     *   "Workers": { \n
     *      "ImuProcess": { \n
     *          "AccFilterWeight": [ \n
     *            1,  //加速度滤波权重,表示一个长度为2周期，每个周期的权重都是1的滤波器(有限长冲激响应滤波器FIR)  \n
     *            1 \n
     *          ], \n
     *          "GyroFilterWeight": [ \n
     *            1, \n
     *            1 \n
     *          ], \n
     *          "MagFilterWeight": [ \n
     *            1, \n
     *            1 \n
     *          ] \n
     *      } \n
     * } \n
     *
     *
     * @tparam SchedulerType 调度器类型
     * @tparam ImuType IMU传感器类型，用户可以通过这个参数来指定IMU传感器的具体类型，
     * 但是这个类型必须要实现GetAccX, GetAccY, GetAccZ, GetGyroX, GetGyroY, GetGyroZ, GetRoll, GetPitch, GetYaw这些方法。
     * @tparam ImuPrecision IMU数据的精度，用户可以通过这个参数来指定IMU数据的精度，比如可以指定为float或者double
     */
    template <typename SchedulerType, typename ImuPrecision, size_t JointNumber>
    class RosProcessWorker : public AbstractWorker<SchedulerType>
    {
        ///@brief 传感器数据必须是数值类型
        static_assert(std::is_arithmetic<ImuPrecision>::value, "ImuPrecision must be a arithmetic type");

        ///@brief 传感器数据类型
        // using ImuValVec = math::Vector<ImuPrecision, 3>;
        using ValVec3 = math::Vector<ImuPrecision, 3>;

    public:
        /**
         * @brief 构造一个IMU数据处理工人类型
         *
         * @param scheduler 调度器的指针
         * @param ImuInstance IMU传感器实例指针
         * @param root_cfg 配置文件
         */
        RosProcessWorker(SchedulerType *scheduler, const nlohmann::json &root_cfg)
            : AbstractWorker<SchedulerType>(scheduler)
        {
            nlohmann::json cfg = root_cfg["Workers"]["RosNav"];
            this->PrintSplitLine();
            std::cout << "RosProcessWorker" << std::endl;

            try
            {
                this->max_ang_vel  =  cfg["MaxAngularSpeed"][0].get<ImuPrecision>();
                this->max_lin_vel  =  cfg["MaxLinearSpeed"][0].get<ImuPrecision>();
                this->target_x_    =  cfg["TargetPosX"][0].get<ImuPrecision>();
                this->target_y_    =  cfg["TargetPosY"][0].get<ImuPrecision>();
                        
                // this->max_ang_vel = max_ang_vel_cfg ;
                // this->max_lin_vel = max_lin_vel_cfg ;
                // this->target_x_   = target_x__cfg   ;
                // this->target_y_   = target_y__cfg   ;
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                std::cerr << "Failed to get nav config, use default value." << std::endl;
                this->max_ang_vel = 0.2;
                this->max_lin_vel = 0.2;
                this->target_x_ = 0;
                this->target_y_ = 0;

            }
            std::cout << "[RosNavWorker] Set max speed: " << this->max_lin_vel << ", max angular speed: " << this->max_ang_vel << std::endl;
            std::cout << "[RosNavWorker] Target position: " << this->target_x_ << ", " << this->target_y_ << std::endl;
            this->PrintSplitLine();
            ros::NodeHandle this_nh;
            this->nh = &this_nh;

            this->pubImu = nh->advertise<sensor_msgs::Imu>("/bitbot/imu", 10);

            // 创建关节状态消息发布器
            this->pubJoint_State = nh->advertise<sensor_msgs::JointState>("/bitbot/joint_states", 10);

            // 创建足底力量消息发布器
            this->publFoot_Force = nh->advertise<geometry_msgs::WrenchStamped>("/bitbot/left_foot_force", 10);
            this->pubrFoot_Force = nh->advertise<geometry_msgs::WrenchStamped>("/bitbot/right_foot_force", 10);

            // 创建订阅
            this->get_estimated_state = nh->subscribe<nav_msgs::Odometry>("/Odometry", 1000, &RosProcessWorker::estimatedStateCallback, this);
            // 创建新线程并执行ros::spin()
            this->spin_thread = std::thread(&RosProcessWorker::spinThread, this);
            this->start_nav_ = false;

            this->angular_kp_ = 1.0;
            this->linear_kp_ = 2;
            this->angle_tolerance_ = 0.2;
            this->pos_tolerance_ = 0.5;
            this->count = 0;
            this->angular_z = 0;
            this->linear_x = 0;

            this->pushed_command.data()[0] = 0;
            this->pushed_command.data()[1] = 0;
            this->pushed_command.data()[2] = 0;
        }

        /**
         * @brief 析构函数，虚函数，用于释放资源
         *
         */
        ~RosProcessWorker()
        {
            this->run = false;
        }

        /**
         * @brief TaskCycleBegin方法，在每次任务队列循环的开始会被调度器调用，用于获取IMU数据并进行滤波和去除异常值。
         *
         */
        void TaskCycleBegin() override
        {
        }

        /**
         * @brief TaskRun方法默认没有实现工作逻辑，因为对IMU数据的处理通常在流水线的开始阶段。
         *
         */
        void TaskRun() override
        {
            auto time_now = ros::Time::now();
            z::math::Vector<ImuPrecision, 3> acc;
            z::math::Vector<ImuPrecision, 3> gyr;
            this->Scheduler->template GetData<"AccelerationValue">(acc);
            this->Scheduler->template GetData<"AngleVelocityValue">(gyr);
            this->imu_msg.header.stamp = time_now;
            imu_msg.header.frame_id = "imu_link";
            imu_msg.angular_velocity.x = gyr.data()[0];
            imu_msg.angular_velocity.y = gyr.data()[1];
            imu_msg.angular_velocity.z = gyr.data()[2];
            imu_msg.linear_acceleration.x = acc.data()[0];
            imu_msg.linear_acceleration.y = acc.data()[1];
            imu_msg.linear_acceleration.z = acc.data()[2];
            this->pubImu.publish(this->imu_msg);

            z::math::Vector<ImuPrecision, JointNumber> JointVel;
            this->Scheduler->template GetData<"CurrentMotorVelocity">(JointVel);
            z::math::Vector<ImuPrecision, JointNumber> JointPos;
            this->Scheduler->template GetData<"CurrentMotorPosition">(JointPos);
            joint_state_msg.header.stamp = time_now;
            joint_state_msg.name.resize(12);
            joint_state_msg.position.resize(12);
            joint_state_msg.velocity.resize(12);
            joint_state_msg.name = {
                "lleg1",
                "lleg2",
                "lleg3",
                "lleg4",
                "lleg5",
                "lleg6",
                "rleg1",
                "rleg2",
                "rleg3",
                "rleg4",
                "rleg5",
                "rleg6"};
            joint_state_msg.position[0] = JointPos.data()[0];
            joint_state_msg.position[1] = JointPos.data()[1];            
            joint_state_msg.position[2] = JointPos.data()[2];
            joint_state_msg.position[3] = JointPos.data()[3];
            joint_state_msg.position[4] = JointPos.data()[4];
            joint_state_msg.position[5] = JointPos.data()[5];
            joint_state_msg.position[6] = JointPos.data()[6];
            joint_state_msg.position[7] = JointPos.data()[7];
            joint_state_msg.position[8] = JointPos.data()[8];
            joint_state_msg.position[9] = JointPos.data()[9];
            joint_state_msg.position[10] = JointPos.data()[10];
            joint_state_msg.position[11] = JointPos.data()[11];

            joint_state_msg.velocity[0] = JointVel.data()[0];
            joint_state_msg.velocity[1] = JointVel.data()[1];            
            joint_state_msg.velocity[2] = JointVel.data()[2];
            joint_state_msg.velocity[3] = JointVel.data()[3];
            joint_state_msg.velocity[4] = JointVel.data()[4];
            joint_state_msg.velocity[5] = JointVel.data()[5];
            joint_state_msg.velocity[6] = JointVel.data()[6];
            joint_state_msg.velocity[7] = JointVel.data()[7];
            joint_state_msg.velocity[8] = JointVel.data()[8];
            joint_state_msg.velocity[9] = JointVel.data()[9];
            joint_state_msg.velocity[10] = JointVel.data()[10];
            joint_state_msg.velocity[11] = JointVel.data()[11];
            
            pubJoint_State.publish(this->joint_state_msg);

            z::math::Vector<ImuPrecision, 6> forceSensorL;
            this->Scheduler->template GetData<"ForceSensorLeft">(forceSensorL);
            l_foot_force_msg.header.stamp = time_now;
            l_foot_force_msg.header.frame_id = "l_foot_force_link";
            l_foot_force_msg.wrench.force.x = forceSensorL.data()[0];
            l_foot_force_msg.wrench.force.y = forceSensorL.data()[1];
            l_foot_force_msg.wrench.force.z = forceSensorL.data()[2];
            l_foot_force_msg.wrench.torque.x = forceSensorL.data()[3];
            l_foot_force_msg.wrench.torque.y = forceSensorL.data()[4];
            l_foot_force_msg.wrench.torque.z = forceSensorL.data()[5];
            publFoot_Force.publish(this->l_foot_force_msg);

            z::math::Vector<ImuPrecision, 6> forceSensorR;
            this->Scheduler->template GetData<"ForceSensorRight">(forceSensorR);
            r_foot_force_msg.header.stamp = time_now;
            r_foot_force_msg.header.frame_id = "r_foot_force_link";
            r_foot_force_msg.wrench.force.x = forceSensorR.data()[0];
            r_foot_force_msg.wrench.force.y = forceSensorR.data()[1];
            r_foot_force_msg.wrench.force.z = forceSensorR.data()[2];
            r_foot_force_msg.wrench.torque.x = forceSensorR.data()[3];
            r_foot_force_msg.wrench.torque.y = forceSensorR.data()[4];
            r_foot_force_msg.wrench.torque.z = forceSensorR.data()[5];
            pubrFoot_Force.publish(this->r_foot_force_msg);

            this->count++;
            if (this->start_nav_ && (this->count % 1000 == 0))
            {
                this->count = 0;
                double dx = target_x_ - current_x_;
                double dy = target_y_ - current_y_;
                double target_angle = atan2(dy, dx);

                double distance = sqrt(dx * dx + dy * dy);

                double angle_error = this->normalizeAngle(target_angle - this->current_yaw_);
                switch (this->state_)
                {
                case this->ROTATING:

                    if (fabs(angle_error) < this->angle_tolerance_)
                    {
                        state_ = MOVING_FORWARD;
                        std::cout << "ROTATING: Rotation completed, moving forward." << std::endl;
                        return;
                    }

                    this->angular_z = std::min(this->angular_kp_ * angle_error, max_ang_vel);
                    this->angular_z = std::max(this->angular_z, -max_ang_vel);
                    this->linear_x = 0;
                    this->pushed_command.data()[0] = this->linear_x;
                    this->pushed_command.data()[1] = 0;
                    this->pushed_command.data()[2] = this->angular_z;    

                    std::cout << "ROTATING: " << this->pushed_command << std::endl;

                    this->Scheduler->template SetData<"NetUserCommand3">(pushed_command);

                    break;

                case this->MOVING_FORWARD:
                    this->angular_z = 0;

                    if (distance < this->pos_tolerance_)
                    {
                        state_ = GOAL_REACHED;
                        this->linear_x = 0;
                        std::cout << "MOVING_FORWARD: Moved to target position!" << std::endl;
                        return;
                    }

                    this->linear_x = std::min(this->linear_kp_ * distance, max_lin_vel);
                    this->linear_x = std::max(this->linear_x, -max_lin_vel);
                    this->angular_z = 0;
                    this->pushed_command.data()[0] = this->linear_x;
                    this->pushed_command.data()[1] = 0;
                    this->pushed_command.data()[2] = this->angular_z;    
                    std::cout << "MOVING_FORWARD: " << this->pushed_command << std::endl;
                    this->Scheduler->template SetData<"NetUserCommand3">(pushed_command);

                    break;

                case this->GOAL_REACHED:
                    this->linear_x = 0;
                    this->angular_z = 0;
                    this->pushed_command.data()[0] = this->linear_x;
                    this->pushed_command.data()[1] = 0;
                    this->pushed_command.data()[2] = this->angular_z;   
                    std::cout << "GOAL_REACHED: " << this->pushed_command << std::endl;
                    this->Scheduler->template SetData<"NetUserCommand3">(pushed_command);

                    break;
                }
            }
            ValVec3 command_logger;
            command_logger.data()[0] = this->linear_x;
            command_logger.data()[1] = this->angular_z;
            command_logger.data()[2] = 0.0;

            this->Scheduler->template SetData<"NavCommand">(command_logger);
        }

    void StartNav()
    {
        std::cout << "Starting nevigation." << std::endl;
        this->start_nav_ = true;
    }

    private:
        /**
         * @brief 去除nan值，用上一次的值代替
         *
         * @param vec 待处理的数据
         * @param last_value 上一次的数据
         * @return ImuValVec 处理后的数据
         */
        // ImuValVec RemoveNan(ImuValVec &vec, const ImuValVec &last_value)
        // {
        //     vec.apply([&last_value](ImuPrecision &val, size_t idx)
        //               { val = std::isnan(val) ? last_value[idx] : val; });
        //     return vec;
        // }

        /**
         * @brief get and resolve ros msg
         */
        void estimatedStateCallback(const nav_msgs::Odometry::ConstPtr &msg)
        {
            // position(0) = msg->pose.pose.position.x;
            // position(1) = msg->pose.pose.position.y;
            // position(2) = msg->pose.pose.position.z;
            // velocity(0) = msg->twist.twist.linear.x;
            // velocity(1) = msg->twist.twist.linear.y;
            // velocity(2) = msg->twist.twist.linear.z;

            this->current_x_ = msg->pose.pose.position.x;
            this->current_y_ = msg->pose.pose.position.y;

            tf::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);

            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            this->current_yaw_ = yaw;
        }

        double normalizeAngle(double angle)
        {
            while (angle > M_PI)
            {
                angle -= 2. * M_PI;
            }
            while (angle < -M_PI)
            {
                angle += 2.0 * M_PI;
            }
            return angle;
        }

        void spinThread()
        {
            ros::Rate rate(2000);
            while (run)
            {
                ros::spinOnce();
                rate.sleep();
            }
        }

    private:
        bool run = true;
        ros::NodeHandle *nh;
        ros::Publisher pubImu;
        ros::Publisher pubJoint_State;
        ros::Publisher publFoot_Force;
        ros::Publisher pubrFoot_Force;
        ros::Subscriber get_estimated_state;
        bool imu_updated;
        sensor_msgs::Imu imu_msg;
        sensor_msgs::JointState joint_state_msg;
        geometry_msgs::WrenchStamped l_foot_force_msg;
        geometry_msgs::WrenchStamped r_foot_force_msg;
        std::thread spin_thread;
        bool start_nav_;

        // Eigen::Vector3d position, velocity;
        // Eigen::Quaterniond orientation_q;
        // Eigen::Matrix3d orientation_R;
        double current_x_;
        double current_y_;
        double current_yaw_;

        double target_x_;
        double target_y_;

        double angular_kp_;
        double linear_kp_;
        double angle_tolerance_;
        double pos_tolerance_;

        double angular_z;
        double linear_x;

        double max_lin_vel;
        double max_ang_vel;

        int count;

        ValVec3 pushed_command;

        enum State
        {
            ROTATING,
            MOVING_FORWARD,
            GOAL_REACHED
        };
        State state_;

        bool target_set_;
        /// @brief 加速度滤波器
        // std::unique_ptr<WeightFilter<ImuValVec>> AccFilter;

        // /// @brief 角速度滤波器
        // std::unique_ptr<WeightFilter<ImuValVec>> GyroFilter;

        // /// @brief 角度滤波器
        // std::unique_ptr<WeightFilter<ImuValVec>> MagFilter;
    };
};
