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
    template <typename SchedulerType, typename ForceSensorType, typename ForceSensorPrecision>
    class ForceSensorLeftWorker : public AbstractWorker<SchedulerType>
    {
        ///@brief 传感器数据必须是数值类型
        static_assert(std::is_arithmetic<ForceSensorPrecision>::value, "ImuPrecision must be a arithmetic type");

        ///@brief 传感器数据类型
        using ForceSensorValVec = math::Vector<ForceSensorPrecision, 6>;

    public:
        /**
         * @brief 构造一个IMU数据处理工人类型
         *
         * @param scheduler 调度器的指针
         * @param ImuInstance IMU传感器实例指针
         * @param root_cfg 配置文件
         */
        ForceSensorLeftWorker(SchedulerType *scheduler, ForceSensorType ForceSensorInstancePtr, const nlohmann::json &root_cfg = nlohmann::json())
            : AbstractWorker<SchedulerType>(scheduler),
              ForceSensorInstance(ForceSensorInstancePtr)
        {
            this->PrintSplitLine();
            std::cout << "ForceSensorLeftWorker" << std::endl;

            // try
            // {
            //     std::vector<ImuValVec> AccFilterWeightVec;
            //     std::vector<ImuValVec> GyroFilterWeightVec;
            //     std::vector<ImuValVec> MagFilterWeightVec;
            //     nlohmann::json AccFilterWeight_cfg = cfg["AccFilterWeight"];
            //     nlohmann::json GyroFilterWeight_cfg = cfg["GyroFilterWeight"];
            //     nlohmann::json MagFilterWeight_cfg = cfg["MagFilterWeight"];

            //     for (auto &&val : AccFilterWeight_cfg)
            //     {
            //         ImuPrecision weight = val.get<ImuPrecision>();
            //         AccFilterWeightVec.emplace_back(ImuValVec::ones() * weight);
            //     }

            //     for (auto &&val : GyroFilterWeight_cfg)
            //     {
            //         ImuPrecision weight = val.get<ImuPrecision>();
            //         GyroFilterWeightVec.emplace_back(ImuValVec::ones() * weight);
            //     }

            //     for (auto &&val : MagFilterWeight_cfg)
            //     {
            //         ImuPrecision weight = val.get<ImuPrecision>();
            //         MagFilterWeightVec.emplace_back(ImuValVec::ones() * weight);
            //     }

            //     this->AccFilter = std::make_unique<WeightFilter<ImuValVec>>(AccFilterWeightVec);
            //     this->GyroFilter = std::make_unique<WeightFilter<ImuValVec>>(GyroFilterWeightVec);
            //     this->MagFilter = std::make_unique<WeightFilter<ImuValVec>>(MagFilterWeightVec);
            // }
            // catch (const std::exception &e)
            // {
            //     std::cerr << e.what() << std::endl;
            //     std::cerr << "Failed to get filter weight from config, use default value." << std::endl;
            //     this->AccFilter = std::make_unique<WeightFilter<ImuValVec>>(std::vector<ImuValVec>(1, ImuValVec::ones()));
            //     this->GyroFilter = std::make_unique<WeightFilter<ImuValVec>>(std::vector<ImuValVec>(1, ImuValVec::ones()));
            //     this->MagFilter = std::make_unique<WeightFilter<ImuValVec>>(std::vector<ImuValVec>(1, ImuValVec::ones()));
            // }

            this->PrintSplitLine();
        }

        /**
         * @brief 析构函数，虚函数，用于释放资源
         *
         */
        ~ForceSensorLeftWorker() {}

        /**
         * @brief TaskCycleBegin方法，在每次任务队列循环的开始会被调度器调用，用于获取IMU数据并进行滤波和去除异常值。
         *
         */
        void TaskCycleBegin() override
        {
            ForceSensorValVec data = {
                static_cast<ForceSensorPrecision>(this->ForceSensorInstance->GetForceX()),
                static_cast<ForceSensorPrecision>(this->ForceSensorInstance->GetForceY()),
                static_cast<ForceSensorPrecision>(this->ForceSensorInstance->GetForceZ()),
                static_cast<ForceSensorPrecision>(this->ForceSensorInstance->GetTorqueX()),
                static_cast<ForceSensorPrecision>(this->ForceSensorInstance->GetTorqueY()),
                static_cast<ForceSensorPrecision>(this->ForceSensorInstance->GetTorqueZ())};

            this->Scheduler->template SetData<"ForceSensorLeft">(data);

#ifndef BUILD_SIMUALTION
            // Mag *= (3.14159265358979323846 / 180);
#endif

            // ImuValVec LastMag;
            // this->Scheduler->template GetData<"AngleRaw">(LastMag);
            // Mag = RemoveNan(Mag, LastMag);
            // this->Scheduler->template SetData<"AngleRaw">(Mag);

            // this->Scheduler->template SetData<"AccelerationValue">((*AccFilter)(Acc)); // 滤波
            // this->Scheduler->template SetData<"AngleVelocityValue">((*GyroFilter)(Gyro));
            // this->Scheduler->template SetData<"AngleValue">((*MagFilter)(Mag));
        }

        /**
         * @brief TaskRun方法默认没有实现工作逻辑，因为对IMU数据的处理通常在流水线的开始阶段。
         *
         */
        void TaskRun() override
        {
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

    private:
        /// @brief force传感器实例指针
        ForceSensorType ForceSensorInstance;

        //     /// @brief 加速度滤波器
        //     std::unique_ptr<WeightFilter<ImuValVec>> AccFilter;

        //     /// @brief 角速度滤波器
        //     std::unique_ptr<WeightFilter<ImuValVec>> GyroFilter;

        //     /// @brief 角度滤波器
        //     std::unique_ptr<WeightFilter<ImuValVec>> MagFilter;
    };
};
