#include "ovinf/ovinf_humanoid_stand.h"

namespace ovinf {

HumanoidStandPolicy::~HumanoidStandPolicy() {
  exiting_.store(true);
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

HumanoidStandPolicy::HumanoidStandPolicy(const YAML::Node &config)
    : BasePolicy(config) {
  // Read config
  size_t joint_counter = 0;
  for (auto const &name : config["policy_joint_names"]) {
    joint_names_[name.as<std::string>()] = joint_counter++;
  }

  cycle_time_ = config["cycle_time"].as<float>();
  stand_threshold_ = config["stand_threshold"].as<float>();
  single_obs_size_ = config["single_obs_size"].as<size_t>();
  obs_buffer_size_ = config["obs_buffer_size"].as<size_t>();
  action_size_ = config["action_size"].as<size_t>();
  if (action_size_ != joint_counter) {
    std::cerr << "Action size does not match joint size!" << std::endl;
  }
  action_scale_ = config["action_scale"].as<float>();
  obs_scale_ang_vel_ = config["obs_scales"]["ang_vel"].as<float>();
  obs_scale_lin_vel_ = config["obs_scales"]["lin_vel"].as<float>();
  obs_scale_command_ = config["obs_scales"]["command"].as<float>();
  obs_scale_dof_pos_ = config["obs_scales"]["dof_pos"].as<float>();
  obs_scale_dof_vel_ = config["obs_scales"]["dof_vel"].as<float>();
  obs_scale_proj_gravity_ = config["obs_scales"]["proj_gravity"].as<float>();
  clip_action_ = config["clip_action"].as<float>();
  joint_default_position_ = VectorT(joint_names_.size());

  for (auto const &pair : joint_names_) {
    joint_default_position_(pair.second, 0) =
        config["policy_default_position"][pair.first].as<float>();
  }

  // Create buffer
  obs_buffer_ = std::make_shared<HistoryBuffer<float>>(single_obs_size_,
                                                       obs_buffer_size_);
  input_queue_ = moodycamel::ReaderWriterQueue<VectorT>(obs_buffer_size_ * 2);
  last_action_ = VectorT(12).setZero();
  latest_target_ = VectorT(12).setZero();

  // Create logger
  log_flag_ = config["log_data"].as<bool>();
  if (log_flag_) {
    CreateLog(config);
  }

  // Create model
  compiled_model_ = ov::Core().compile_model(model_path_, device_);
  if (compiled_model_.input().get_element_type() != ov::element::f32) {
    throw std::runtime_error(
        "Model input type is not f32. Please convert the model to f32.");
  }

  infer_request_ = compiled_model_.create_infer_request();
  input_info_ = compiled_model_.input();

  inference_done_.store(true);
  exiting_.store(false);
  worker_thread_ = std::thread(&HumanoidStandPolicy::WorkerThread, this);
}

bool HumanoidStandPolicy::WarmUp(
    ProprioceptiveObservation<float> const &obs_pack, size_t num_itrations) {
  std::chrono::high_resolution_clock::time_point start_time =
      std::chrono::high_resolution_clock::now();
  VectorT obs(single_obs_size_);
  obs.setZero();
  obs.segment(0, 2) = Eigen::Vector2f{0.0, 1.0};
  VectorT command_scaled(3);
  command_scaled.segment(0, 2) =
      obs_pack.command.segment(0, 2) * obs_scale_lin_vel_;
  command_scaled(2) = obs_pack.command(2) * obs_scale_ang_vel_;
  obs.segment(2, 3) = command_scaled * obs_scale_command_;
  obs.segment(5, 12) =
      (obs_pack.joint_pos - joint_default_position_) * obs_scale_dof_pos_;
  obs.segment(17, 12) = obs_pack.joint_vel * obs_scale_dof_vel_;
  obs.segment(29, 12) = last_action_;
  obs.segment(41, 3) = obs_pack.ang_vel * obs_scale_ang_vel_;
  obs.segment(44, 3) = obs_pack.proj_gravity * obs_scale_proj_gravity_;

  for (size_t i = 0; i < obs_buffer_size_; ++i) {
    obs_buffer_->AddObservation(obs);
  }

  for (size_t i = 0; i < num_itrations; ++i) {
    auto input_eigen = obs_buffer_->GetObsHistory();
    ov::Tensor input_tensor(input_info_.get_element_type(),
                            input_info_.get_shape(), input_eigen.data());
    infer_request_.set_input_tensor(input_tensor);
    infer_request_.start_async();
    infer_request_.wait();

    auto action_tensor = infer_request_.get_output_tensor();
    VectorT action_eigen =
        Eigen::Map<VectorT>(action_tensor.data<float>(), action_size_);

    last_action_ = action_eigen;
    obs.segment(29, 12) = last_action_;
    obs_buffer_->AddObservation(obs);
  }

  std::chrono::duration<double> elapsed_seconds =
      std::chrono::high_resolution_clock::now() - start_time;
  // std::cout << "Warm up time: " << elapsed_seconds.count() * 1000 << "ms"
  //           << std::endl;
  return true;
}

bool HumanoidStandPolicy::InferUnsync(
    ProprioceptiveObservation<float> const &obs_pack) {
  if (gait_start_ == false && obs_pack.command.norm() > stand_threshold_) {
    gait_start_ = true;
    gait_start_time_ = std::chrono::steady_clock::now();
  } else if (gait_start_ == true &&
             obs_pack.command.norm() < stand_threshold_) {
    gait_start_ = false;
  }

  double gait_time_value = 0.0;
  if (gait_start_) {
    double current_gait_time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      gait_start_time_)
            .count();
    gait_time_value = 2 * M_PI * current_gait_time / cycle_time_;
  }

  VectorT obs(single_obs_size_);
  obs.setZero();
  obs.segment(0, 2) =
      Eigen::Vector2f{std::sin(gait_time_value), std::cos(gait_time_value)};
  VectorT command_scaled(3);
  command_scaled.segment(0, 2) =
      obs_pack.command.segment(0, 2) * obs_scale_lin_vel_;
  command_scaled(2) = obs_pack.command(2) * obs_scale_ang_vel_;
  obs.segment(2, 3) = command_scaled * obs_scale_command_;
  obs.segment(5, 12) =
      (obs_pack.joint_pos - joint_default_position_) * obs_scale_dof_pos_;
  obs.segment(17, 12) = obs_pack.joint_vel * obs_scale_dof_vel_;
  obs.segment(29, 12) = last_action_;
  obs.segment(41, 3) = obs_pack.ang_vel * obs_scale_ang_vel_;
  obs.segment(44, 3) = obs_pack.proj_gravity * obs_scale_proj_gravity_;

  if (!inference_done_.load()) {
    input_queue_.enqueue(obs);
    return false;
  } else {
    while (input_queue_.peek() != nullptr) {
      VectorT old_obs;
      input_queue_.try_dequeue(old_obs);
      obs_buffer_->AddObservation(old_obs);
    }
    obs_buffer_->AddObservation(obs);

    ov::Tensor input_tensor(input_info_.get_element_type(),
                            input_info_.get_shape(),
                            obs_buffer_->GetObsHistory().data());
    infer_start_time_ = std::chrono::high_resolution_clock::now();

    infer_request_.set_input_tensor(input_tensor);
    inference_done_.store(false);

    if (log_flag_) {
      WriteLog(obs_pack);
    }

    return true;
  }
}

std::optional<HumanoidStandPolicy::VectorT> HumanoidStandPolicy::GetResult(
    const size_t timeout) {
  if (inference_done_.load()) [[unlikely]] {
    return latest_target_;
  } else {
    std::this_thread::sleep_for(std::chrono ::microseconds(timeout));
    if (inference_done_.load()) [[likely]] {
      return latest_target_;
    }
    return std::nullopt;
  }
}

void HumanoidStandPolicy::PrintInfo() {
  std::cout << "Load model: " << this->model_path_ << std::endl;
  std::cout << "Device: " << device_ << std::endl;
  std::cout << "Single obs size: " << single_obs_size_ << std::endl;
  std::cout << "Obs buffer size: " << obs_buffer_size_ << std::endl;
  std::cout << "Action size: " << action_size_ << std::endl;
  std::cout << "Action scale: " << action_scale_ << std::endl;
  std::cout << "  - Obs scale ang vel: " << obs_scale_ang_vel_ << std::endl;
  std::cout << "  - Obs scale lin vel: " << obs_scale_lin_vel_ << std::endl;
  std::cout << "  - Obs scale command: " << obs_scale_command_ << std::endl;
  std::cout << "  - Obs scale dof pos: " << obs_scale_dof_pos_ << std::endl;
  std::cout << "  - Obs scale dof vel: " << obs_scale_dof_vel_ << std::endl;
  std::cout << "  - Obs scale proj gravity: " << obs_scale_proj_gravity_
            << std::endl;
  std::cout << "Clip action: " << clip_action_ << std::endl;
  std::cout << "Joint default position: " << joint_default_position_.transpose()
            << std::endl;
  std::cout << "Joint names: " << std::endl;
  for (const auto &pair : joint_names_) {
    std::cout << "  - " << pair.first << ": " << pair.second << std::endl;
  }
}

void HumanoidStandPolicy::WorkerThread() {
  if (realtime_) {
    if (!setProcessHighPriority(99)) {
      std::cerr << "Failed to set process high priority." << std::endl;
    }
    if (!StickThisThreadToCore(6)) {
      std::cerr << "Failed to stick thread to core." << std::endl;
    }
  }

  while (exiting_.load() == false) {
    if (!inference_done_.load()) {
      infer_request_.infer();
      auto action_tensor = infer_request_.get_output_tensor();
      infer_end_time_ = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_seconds =
          infer_end_time_ - infer_start_time_;
      // std::cout << "Inference time: " << elapsed_seconds.count() * 1000
      //           << "ms" << std::endl;
      inference_time_ = elapsed_seconds.count() * 1000;

      VectorT action_eigen =
          Eigen::Map<VectorT>(action_tensor.data<float>(), action_size_);

      last_action_ = action_eigen;
      latest_target_ = action_eigen * action_scale_ + joint_default_position_;
    }
    inference_done_.store(true);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
}

void HumanoidStandPolicy::CreateLog(YAML::Node const &config) {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm *now_tm = std::localtime(&now_time);
  std::stringstream ss;
  ss << std::put_time(now_tm, "%Y-%m-%d-%H-%M-%S");
  std::string current_time = ss.str();

  std::string log_dir = config["log_dir"].as<std::string>();
  std::filesystem::path config_file_path(log_dir);
  if (config_file_path.is_relative()) {
    config_file_path = canonical(config_file_path);
  }

  if (!exists(config_file_path)) {
    create_directories(config_file_path);
  }

  std::string logger_file =
      config_file_path.string() + "/" + current_time + "_humanoid_stand.csv";

  // Get headers
  std::vector<std::string> headers;

  headers.push_back("clock_sin");
  headers.push_back("clock_cos");
  headers.push_back("command_vel_x");
  headers.push_back("command_vel_y");
  headers.push_back("command_vel_w");
  for (size_t i = 0; i < action_size_; ++i) {
    headers.push_back("joint_pos_" + std::to_string(i));
  }
  for (size_t i = 0; i < action_size_; ++i) {
    headers.push_back("joint_vel_" + std::to_string(i));
  }
  for (size_t i = 0; i < action_size_; ++i) {
    headers.push_back("last_action_" + std::to_string(i));
  }
  headers.push_back("ang_vel_x");
  headers.push_back("ang_vel_y");
  headers.push_back("ang_vel_z");
  headers.push_back("prog_gravity_x");
  headers.push_back("prog_gravity_y");
  headers.push_back("prog_gravity_z");
  headers.push_back("inference_time_ms");

  csv_logger_ = std::make_shared<CsvLogger>(logger_file, headers);
}

void HumanoidStandPolicy::WriteLog(
    ProprioceptiveObservation<float> const &obs_pack) {
  std::vector<CsvLogger::Number> datas;

  double current_gait_time =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    gait_start_time_)
          .count();
  double gait_time_value = 2 * M_PI * current_gait_time / cycle_time_;

  datas.push_back(std::sin(gait_time_value));
  datas.push_back(std::cos(gait_time_value));

  for (size_t i = 0; i < 3; ++i) {
    datas.push_back(obs_pack.command(i));
  }
  for (size_t i = 0; i < action_size_; ++i) {
    datas.push_back(obs_pack.joint_pos(i));
  }
  for (size_t i = 0; i < action_size_; ++i) {
    datas.push_back(obs_pack.joint_vel(i));
  }
  for (size_t i = 0; i < action_size_; ++i) {
    datas.push_back(last_action_(i));
  }
  for (size_t i = 0; i < 3; ++i) {
    datas.push_back(obs_pack.ang_vel(i));
  }
  for (size_t i = 0; i < 3; ++i) {
    datas.push_back(obs_pack.proj_gravity(i));
  }
  datas.push_back(inference_time_);

  csv_logger_->Write(datas);
}

}  // namespace ovinf
