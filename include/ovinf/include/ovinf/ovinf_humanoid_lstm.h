/**
 * @file humanoid_lstm_policy.h
 * @brief LSTM-based OpenVINO inference for humanoid policy.
 * @author Siyuan Wang
 * @date 2025-6-18
 */

#ifndef OVINF_HUMANOID_LSTM_H
#define OVINF_HUMANOID_LSTM_H

#include <Eigen/Core>
#include <openvino/openvino.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "utils/csv_logger.hpp"
#include "readerwriterqueue/readerwriterqueue.h"

namespace ovinf {

/**
 * @brief LSTM-based humanoid policy using OpenVINO runtime.
 */
class HumanoidLSTM {
  using VectorT = Eigen::VectorXf;

 public:
  HumanoidLSTM() = delete;
  explicit HumanoidLSTM(const std::string& model_path, const std::string& device_name = "CPU");

  // == Main Interfaces ==
  void Reset();
  void StartThread();
  void StopThread();

  bool InferUnsync(const VectorT& obs);
  std::optional<VectorT> GetResult(size_t timeout_us = 300);

  void EnableLog(const std::string& log_file_path, size_t action_dim);

  VectorT Infer(const VectorT& obs);  // Synchronous fallback
  void SetObservation(const VectorT& obs);
  void Infer();                       // Low-level call
  VectorT GetAction() const;

 private:
  void InitModel(const std::string& model_path, const std::string& device_name);
  void WorkerThread();
  void WriteLog(const VectorT& action);

 private:
  // == OpenVINO inference ==
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;

  ov::Output<const ov::Node> obs_input_port_;
  ov::Output<const ov::Node> h_state_input_port_;
  ov::Output<const ov::Node> c_state_input_port_;
  ov::Output<const ov::Node> action_output_port_;
  ov::Output<const ov::Node> h_state_output_port_;
  ov::Output<const ov::Node> c_state_output_port_;

  // == Model data ==
  VectorT observation_;      ///< Current observation
  VectorT action_;           ///< Output action
  VectorT h_state_, c_state_;///< LSTM hidden/cell state
  VectorT latest_target_;    ///< Last inference result

  int input_dim = 0;
  int output_dim = 0;
  int lstm_hidden_size_ = 0;

  // == Threading ==
  std::thread worker_thread_;
  moodycamel::ReaderWriterQueue<VectorT> input_queue_;
  std::atomic<bool> inference_done_{false};
  std::atomic<bool> exiting_{false};

  std::chrono::high_resolution_clock::time_point infer_start_time_, infer_end_time_;
  float inference_time_ = 0.f;

  // == Gait clock ==
  bool gait_start_ = false;
  std::chrono::steady_clock::time_point gait_start_time_;
  double cycle_time_ = 1.0;

  // == Logging ==
  bool log_flag_ = false;
  CsvLogger::Ptr csv_logger_;
};

}  // namespace ovinf

#endif  // OVINF_HUMANOID_LSTM_H
