#pragma once
#include <array>

#include "Schedulers/AbstractScheduler.hpp"

#include "Utils/StaticStringUtils.hpp"
#include "Utils/MathTypes.hpp"

#include "Workers/AbstractWorker.hpp"
#include "Workers/AsyncLoggerWorker.hpp"
#include "Workers/ImuProcessWorker.hpp"
#include "Workers/MotorControlWorker.hpp"
#include "Workers/MotorResetPositionWorker.hpp"
#include "Workers/NetCmdWorker.hpp"
#include "Workers/RosProcessWorker.hpp"
#include "Workers/ForceSensorLeftWorker.hpp"
#include "Workers/ForceSensorRightWorker.hpp"

#include "Workers/NN/EraxLikeInferenceWorker.hpp"
#include "Workers/NN/HumanoidGymInferenceWorker.hpp"
#include "Workers/NN/UnitreeRlGymInferenceWorker.hpp"


class DeviceImu {};
class DeviceJoint {};


/************ basic definintion***********/
using RealNumber = float;
constexpr size_t JOINT_NUMBER = 12;
using Vec3 = z::math::Vector<RealNumber, 3>;
using Vec6 = z::math::Vector<RealNumber, 6>;
using MotorVec = z::math::Vector<RealNumber, JOINT_NUMBER>;

#ifdef BUILD_SIMULATION
class DeviceImu {};    

// 替代原 bitbot::MujocoJoint / CifxJoint
class DeviceJoint {};

// 替代原 bitbot::ForceSri6d

constexpr std::array<size_t, JOINT_NUMBER> JOINT_ID_MAP = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
constexpr size_t IMU_ID_MAP = 0;
#else
class DeviceForceSensor {};
constexpr std::array<size_t, JOINT_NUMBER> JOINT_ID_MAP = {11, 10, 19, 20, 22, 21, 12, 13, 18, 14, 15, 16};
constexpr std::array<bool, JOINT_NUMBER> JOINT_ALTER_TYPE = {false, false, false, true, true, false, false, false, false, true, true, false};
constexpr size_t IMU_ID_MAP = 8;
#endif

/********** IMU Data Pair******************/
constexpr z::CTSPair<"AccelerationRaw", Vec3> ImuAccRawPair;
constexpr z::CTSPair<"AngleVelocityRaw", Vec3> ImuGyroRawPair;
constexpr z::CTSPair<"AngleRaw", Vec3> ImuMagRawPair;

constexpr z::CTSPair<"AccelerationValue", Vec3> ImuAccFilteredPair;
constexpr z::CTSPair<"AngleValue", Vec3> ImuMagFilteredPair;
constexpr z::CTSPair<"AngleVelocityValue", Vec3> ImuGyroFilteredPair;

/********** Linear Velocity Pair ***********/
constexpr z::CTSPair<"LinearVelocityValue", Vec3> LinearVelocityValuePair;

/********** Motor control Pair ************/
constexpr z::CTSPair<"TargetMotorPosition", MotorVec> TargetMotorPosPair;
constexpr z::CTSPair<"TargetMotorVelocity", MotorVec> TargetMotorVelPair;
constexpr z::CTSPair<"TargetMotorTorque", MotorVec> TargetMotorTorquePair;
constexpr z::CTSPair<"CurrentMotorPosition", MotorVec> CurrentMotorPosPair;
constexpr z::CTSPair<"CurrentMotorVelocity", MotorVec> CurrentMotorVelPair;
constexpr z::CTSPair<"CurrentMotorTorque", MotorVec> CurrentMotorTorquePair;
constexpr z::CTSPair<"LimitTargetMotorTorque", MotorVec> LimitTargetMotorTorquePair;
constexpr z::CTSPair<"CurrentMotorPositionRaw", MotorVec> CurrentMotorPosRawPair;
constexpr z::CTSPair<"CurrentMotorVelocityRaw", MotorVec> CurrentMotorVelRawPair;

/********* NN pair ********************/
constexpr z::CTSPair<"NetLastAction", MotorVec> NetLastActionPair;
constexpr z::CTSPair<"NetUserCommand3", Vec3> NetCommand3Pair;
constexpr z::CTSPair<"NetProjectedGravity", Vec3> NetProjectedGravityPair;
constexpr z::CTSPair<"NetScaledAction", MotorVec> NetScaledActionPair;
constexpr z::CTSPair<"NetClockVector", z::math::Vector<RealNumber, 2>> NetClockVectorPair;
constexpr z::CTSPair<"InferenceTime", RealNumber> InferenceTimePair;

/********* Force Sensor pair ********************/
constexpr z::CTSPair<"ForceSensorLeft", Vec6> LeftForceSensorPair;
constexpr z::CTSPair<"ForceSensorRight", Vec6> RightForceSensorPair;

constexpr z::CTSPair<"NavCommand", Vec3> NavCommandPair;
// define scheduler
using SchedulerType = z::AbstractScheduler<NavCommandPair, ImuAccRawPair, ImuGyroRawPair, ImuMagRawPair, LinearVelocityValuePair,
                                           ImuAccFilteredPair, ImuGyroFilteredPair, ImuMagFilteredPair,
                                           TargetMotorPosPair, TargetMotorVelPair, CurrentMotorPosPair, CurrentMotorVelPair, CurrentMotorTorquePair,
                                           TargetMotorTorquePair, LimitTargetMotorTorquePair,
                                           CurrentMotorVelRawPair, CurrentMotorPosRawPair,
                                           NetLastActionPair, NetCommand3Pair, NetProjectedGravityPair, NetScaledActionPair, NetClockVectorPair, InferenceTimePair,
                                           LeftForceSensorPair, RightForceSensorPair>;

// define workers
using MotorResetWorkerType = z::MotorResetPositionWorker<SchedulerType, RealNumber, JOINT_NUMBER>;
using ImuWorkerType = z::ImuProcessWorker<SchedulerType, DeviceImu *, RealNumber>;
using RosWorkerType = z::RosProcessWorker<SchedulerType, RealNumber, JOINT_NUMBER>;
using MotorWorkerType = z::MotorControlWorker<SchedulerType, DeviceJoint *, RealNumber, JOINT_NUMBER>;
using MotorPDWorkerType = z::MotorPDControlWorker<SchedulerType, RealNumber, JOINT_NUMBER>;
using LoggerWorkerType = z::AsyncLoggerWorker<SchedulerType, RealNumber, NavCommandPair, ImuAccRawPair, ImuGyroRawPair, ImuMagRawPair, LinearVelocityValuePair,
                                              ImuAccFilteredPair, ImuGyroFilteredPair, ImuMagFilteredPair,
                                              TargetMotorPosPair, TargetMotorVelPair, CurrentMotorPosPair, CurrentMotorVelPair, CurrentMotorTorquePair,
                                              TargetMotorTorquePair, LimitTargetMotorTorquePair,
                                              NetLastActionPair, NetCommand3Pair, NetProjectedGravityPair, NetScaledActionPair, NetClockVectorPair, InferenceTimePair,
                                              LeftForceSensorPair, RightForceSensorPair>;

using CmdWorkerType = z::NetCmdWorker<SchedulerType, RealNumber, NetCommand3Pair>;
using FlexPatchWorkerType = z::SimpleCallbackWorker<SchedulerType>;
using ForceSensorRightType = z::ForceSensorRightWorker<SchedulerType, DeviceForceSensor *, RealNumber>;
using ForceSensorLeftType = z::ForceSensorLeftWorker<SchedulerType, DeviceForceSensor *, RealNumber>;

/******define actor net************/
constexpr size_t OBSERVATION_STUCK_LENGTH = 1;
constexpr size_t OBSERVATION_EXTRA_LENGTH = 5;
using HumanoidGymInferWorkerType = z::HumanoidGymInferenceWorker<SchedulerType, RealNumber,OBSERVATION_STUCK_LENGTH, JOINT_NUMBER>;
//  using EraxLikeInferWorkerType = z::EraxLikeInferenceWorker<SchedulerType, RealNumber, OBSERVATION_STUCK_LENGTH, OBSERVATION_EXTRA_LENGTH, JOINT_NUMBER>;
//using UnitreeRlGymInferWorkerType = z::UnitreeRlGymInferenceWorker<SchedulerType, RealNumber, OBSERVATION_STUCK_LENGTH, JOINT_NUMBER>;