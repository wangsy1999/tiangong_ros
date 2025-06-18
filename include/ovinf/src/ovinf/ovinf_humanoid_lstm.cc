/**
 * @file humanoid_lstm_policy.h
 * @brief LSTM-based OpenVINO inference for humanoid policy.
 * @author Siyuan Wang
 * @date 2025-6-18
 */
#include "ovinf/humanoid_lstm_policy.h"
#include <stdexcept>
#include <cmath>
#include <chrono>
#include <thread>

namespace ovinf {

HumanoidLSTM::HumanoidLSTM(const std::string& model_path, const std::string& device_name) {
  InitModel(model_path, device_name);
}

void HumanoidLSTM::InitModel(const std::string& model_path, const std::string& device_name) {
  std::shared_ptr<ov::Model> model = core_.read_model(model_path);

  obs_input_port_ = model->input("observation");
  h_state_input_port_ = model->input("hidden_state");
  c_state_input_port_ = model->input("cell_state");

  action_output_port_ = model->output("action");
  h_state_output_port_ = model->output("new_hidden_state");
  c_state_output_port_ = model->output("new_cell_state");

  input_dim = obs_input_port_.get_shape()[1];
  output_dim = action_output_port_.get_shape()[1];
  lstm_hidden_size_ = h_state_input_port_.get_shape().back();

  compiled_model_ = core_.compile_model(model, device_name);
  infer_request_ = compiled_model_.create_infer_request();

  observation_.resize(input_dim);
  action_.resize(output_dim);
  h_state_.resize(lstm_hidden_size_);
  c_state_.resize(lstm_hidden_size_);
  Reset();
}

void HumanoidLSTM::Reset() {
  h_state_.setZero();
  c_state_.setZero();
}

void HumanoidLSTM::SetObservation(const VectorT& obs) {
  if (obs.size() != input_dim) {
    throw std::runtime_error("Observation size mismatch.");
  }
  observation_ = obs;
}

void HumanoidLSTM::Infer() {
  std::copy_n(observation_.data(), input_dim, infer_request_.get_tensor(obs_input_port_).data<float>());
  std::copy_n(h_state_.data(), h_state_.size(), infer_request_.get_tensor(h_state_input_port_).data<float>());
  std::copy_n(c_state_.data(), c_state_.size(), infer_request_.get_tensor(c_state_input_port_).data<float>());

  infer_request_.infer();

  std::copy_n(infer_request_.get_tensor(action_output_port_).data<float>(), output_dim, action_.data());
  std::copy_n(infer_request_.get_tensor(h_state_output_port_).data<float>(), h_state_.size(), h_state_.data());
  std::copy_n(infer_request_.get_tensor(c_state_output_port_).data<float>(), c_state_.size(), c_state_.data());
}

HumanoidLSTM::VectorT HumanoidLSTM::GetAction() const {
  return action_;
}

HumanoidLSTM::VectorT HumanoidLSTM::Infer(const VectorT& obs) {
  SetObservation(obs);
  Infer();
  return GetAction();
}

void HumanoidLSTM::StartThread() {
  exiting_.store(false);
  inference_done_.store(true);
  worker_thread_ = std::thread(&HumanoidLSTM::WorkerThread, this);
}

void HumanoidLSTM::StopThread() {
  exiting_.store(true);
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

bool HumanoidLSTM::InferUnsync(const VectorT& obs) {
  if (!gait_start_flag_) {
    gait_start_ = std::chrono::steady_clock::now();
    gait_start_flag_ = true;
  }
  if (!inference_done_.load()) {
    input_queue_.enqueue(obs);
    return false;
  }
  while (input_queue_.peek() != nullptr) {
    VectorT tmp;
    input_queue_.try_dequeue(tmp);
  }
  input_queue_.enqueue(obs);
  inference_done_.store(false);
  infer_start_ = std::chrono::high_resolution_clock::now();
  return true;
}

std::optional<HumanoidLSTM::VectorT> HumanoidLSTM::GetResult(size_t timeout_us) {
  if (inference_done_.load()) {
    return latest_target_;
  } else {
    std::this_thread::sleep_for(std::chrono::microseconds(timeout_us));
    if (inference_done_.load()) {
      return latest_target_;
    }
    return std::nullopt;
  }
}

void HumanoidLSTM::WorkerThread() {
  while (!exiting_.load()) {
    if (!inference_done_.load()) {
      VectorT obs;
      if (!input_queue_.try_dequeue(obs)) {
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        continue;
      }
      SetObservation(obs);
      Infer();
      latest_target_ = GetAction();
      infer_end_ = std::chrono::high_resolution_clock::now();
      inference_time_ms_ = std::chrono::duration<float, std::milli>(infer_end_ - infer_start_).count();

      if (log_flag_ && csv_logger_) {
        double phase = std::chrono::duration<double>(std::chrono::steady_clock::now() - gait_start_).count();
        double clock_val = 2.0 * M_PI * phase / cycle_time_;
        std::vector<float> log_data;
        log_data.push_back(std::sin(clock_val));
        log_data.push_back(std::cos(clock_val));
        for (int i = 0; i < latest_target_.size(); ++i)
          log_data.push_back(latest_target_[i]);
        log_data.push_back(inference_time_ms_);
        csv_logger_->Write(log_data);
      }
      inference_done_.store(true);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
}

void HumanoidLSTM::EnableLog(const std::string& path, size_t dim) {
  log_flag_ = true;
  std::vector<std::string> headers;
  headers.emplace_back("clock_sin");
  headers.emplace_back("clock_cos");
  for (size_t i = 0; i < dim; ++i) {
    headers.emplace_back("action_" + std::to_string(i));
  }
  headers.emplace_back("inference_time_ms");
  csv_logger_ = std::make_shared<CsvLogger>(path, headers);
}

}  // namespace ovinf
