//
// Created by suyoung on 8/7/21.
//

#include <filesystem>
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "raisin_push_controller/raibot_learning_controller.hpp"

namespace raisin {

namespace controller {

using std::placeholders::_1;
using std::placeholders::_2;

raibotLearningController::raibotLearningController()
: Controller("raisin_push_controller"),
  actor_(72, 2, {256, 128}),
  estimator_(36, 1, {128, 128}),
  encoder_(96, 2),
  high_actor_({128, 64}),
  param_(parameter::ParameterContainer::getRoot()["raibotLearningController"])
//  button_press_buffer_(false)
{
  param_.loadFromPackageParameterFile("raisin_push_controller");

  raisin::DataLogger::setSaveDirectory(ament_index_cpp::get_package_share_directory("raisin_data_logger"));

  serviceSetCommand_ = this->create_service<raisin_interfaces::srv::Vector3>(
      "raisin_push_controller/set_command", std::bind(&raibotLearningController::setCommand, this, _1, _2)
      );
//  joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
//      "joy", 10, std::bind(&raibotLearningController::setCommandByTopic, this, _1)
//      );

//  rclcpp::QoS qos(rclcpp::KeepLast(1));
//  rclcpp::Rate limitPubRate(2.0);
//  {
//    limitPublisher_ = this->create_publisher<raisin_raibot::msg::VelocityLimit>(
//        "raisin_push_controller/vel_limit", qos);
//
//    limitTimer_ = this->create_wall_timer(
//        limitPubRate.period(), std::bind(&raibotLearningController::velocityLimitDisplayCallback, this)
//        );
//  }
}

bool raibotLearningController::create(raisim::World *world) {
  control_dt_ = 0.01;
  communication_dt_ = 0.00025;
  high_control_dt_ = 0.2;

  raibotController_.create(world);
  subgoal_command.setZero(3);

//  normal_limit_ = param_("command_normal_mode");
//  boost_limit_ = param_("command_boost_mode");

  std::string network_path = std::string(param_("network_path"));
  std::filesystem::path pack_path(ament_index_cpp::get_package_prefix("raisin_push_controller"));
  std::filesystem::path actor_path = pack_path / network_path / "actor.txt";
  std::filesystem::path estimator_path = pack_path / network_path / "estimator.txt";
  std::filesystem::path obs_mean_path = pack_path / network_path / "obs_mean.csv";
  std::filesystem::path obs_var_path = pack_path / network_path / "obs_var.csv";
  std::filesystem::path eout_mean_path = pack_path / network_path / "eout_mean.csv";
  std::filesystem::path eout_var_path = pack_path / network_path / "eout_var.csv";
  std::filesystem::path high_actor_path = pack_path / network_path / "high_actor.txt";
  std::filesystem::path encoder_path = pack_path / network_path / "encoder.txt";
  std::filesystem::path high_obs_mean_path = pack_path / network_path / "high_obs_mean.csv";
//  RSINFO(high_obs_mean_path)
  std::filesystem::path high_obs_var_path = pack_path / network_path / "high_obs_var.csv";

  actor_.readParamFromTxt(actor_path.string());
  estimator_.readParamFromTxt(estimator_path.string());
  high_actor_.readParamFromTxt(high_actor_path.string());
  encoder_.readParamFromTxt(encoder_path.string());

  std::string in_line;
  std::ifstream obsMean_file(obs_mean_path.string());
  std::ifstream obsVariance_file(obs_var_path.string());
  std::ifstream eoutMean_file(eout_mean_path.string());
  std::ifstream eoutVariance_file(eout_var_path.string());
  std::ifstream high_obsMean_file(high_obs_mean_path.string());
  std::ifstream high_obsVariance_file(high_obs_var_path.string());
  obs_.setZero(raibotController_.getObDim());
  obsMean_.setZero(raibotController_.getObDim());
  obsVariance_.setZero(raibotController_.getObDim());
  high_obsMean_.setZero(raibotController_.getHighObDim());
  high_obsVariance_.setZero(raibotController_.getHighObDim());
  eoutMean_.setZero(raibotController_.getEstDim());
  eoutVariance_.setZero(raibotController_.getEstDim());
  actor_input_.setZero(raibotController_.getObDim() + raibotController_.getEstDim());
/// TODO
  if (obsMean_file.is_open()) {
    for (int i = 0; i < obsMean_.size(); ++i) {
      std::getline(obsMean_file, in_line, ' ');
      obsMean_(i) = std::stof(in_line);
    }
  }
  if (obsVariance_file.is_open()) {
    for (int i = 0; i < obsVariance_.size(); ++i) {
      std::getline(obsVariance_file, in_line, ' ');
      obsVariance_(i) = std::stof(in_line);
    }
  }
  if (eoutMean_file.is_open()) {
    for (int i = 0; i < eoutMean_.size(); ++i) {
      std::getline(eoutMean_file, in_line, ' ');
      eoutMean_(i) = std::stof(in_line);
    }
  }
  if (eoutVariance_file.is_open()) {
    for (int i = 0; i < eoutVariance_.size(); ++i) {
      std::getline(eoutVariance_file, in_line, ' ');
      eoutVariance_(i) = std::stof(in_line);
    }
  }
  if (high_obsMean_file.is_open()) {
    for (int i = 0; i < high_obsMean_.size(); ++i) {
      std::getline(high_obsMean_file, in_line);
      high_obsMean_(i) = std::stof(in_line);
    }
  }
  RSINFO(high_obsMean_)
  if (high_obsVariance_file.is_open()) {
    for (int i = 0; i < high_obsVariance_.size(); ++i) {
      std::getline(high_obsVariance_file, in_line);
      high_obsVariance_(i) = std::stof(in_line);
    }
  }
//  RSINFO(high_obsVariance_)

  obsMean_file.close();
  obsVariance_file.close();
  eoutMean_file.close();
  eoutVariance_file.close();
  high_obsMean_file.close();
  high_obsVariance_file.close();
  estUnscaled_.setZero(raibotController_.getEstDim());
  high_obs_.setZero(raibotController_.getHighObDim());
  log_ = std::make_unique<raisin::DataLogger>(
      "control.raisin_data",
      "observation", raibotController_.getObservation(),
      "estimation", estUnscaled_,
      "targetPosition", raibotController_.getJointPTarget(),
      "subgoal_command", subgoal_command,
      "high_level_observation", high_obs_
  );
  return true;
}

bool raibotLearningController::init(raisim::World *world) {
  raibotController_.init(world);
  return true;
}

bool raibotLearningController::advance(raisim::World *world) {
  /// 100Hz controller
  auto* raibot = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot"));
  auto* raibot_vicon = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot_vicon"));

  /// For high level advance
  if(pd_clk_ >= 100)
  {
    if(clk_ % int(high_control_dt_ / communication_dt_ + 1e-10) == 0) {
      raibotController_.updateHighObservation(world);
      subgoal_command =
          raibotController_.high_advance(world, high_obsScalingAndGetAction().head(3));
//      RSINFO(subgoal_command);
      raibotController_.updateHighActionHistory(subgoal_command);
      raibotController_.setCommand(subgoal_command);
//      RSINFO(subgoal_command)
    }
//    /// For history update
//    if(clk_ % (int(control_dt_ * 5 / (communication_dt_ + 1e-10))) == 0) {
//
//    }
  }
  /// For low level advance
  if(clk_ % int(control_dt_ / communication_dt_ + 1e-10) == 0) {
    if (pd_clk_ < 100) {
      warmUp(world);
      ++pd_clk_;
    }

    else {
      raibot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
      raibot->setPdGains(raibotController_.getJointPGain(), raibotController_.getJointDGain());
//      raibot_vicon->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
//      raibot_vicon->setPdGains(raibotController_.getJointPGain(), raibotController_.getJointDGain());
      raibotController_.updateHighHistory();
      raibotController_.updateObservation(world);
      raibotController_.updateHighStateVariable(world);
      raibotController_.advance(world, obsScalingAndGetAction().head(12));
    }

    log_->append(raibotController_.getObservation(), estUnscaled_, raibotController_.getJointPTarget(), subgoal_command, high_obs_);
  }
  clk_++;
  return true;
}

Eigen::VectorXf raibotLearningController::high_obsScalingAndGetAction() {
  raibotController_.getHighObservation(high_obs_);


  for (int i = 0; i < high_obs_.size(); ++i) {
    high_obs_(i) = (high_obs_(i) - high_obsMean_(i)) / std::sqrt(high_obsVariance_(i) + 1e-8);
//    if (high_obs_(i) > 10) { high_obs_(i) = 10.0; }
//    else if (high_obs_(i) < -10) { high_obs_(i) = -10.0; }
  }

//  RSINFO(high_obs_)

  std::ofstream file("obs.txt");
  if (file.is_open())
  {
    file << high_obs_ << '\n';
//    file << "m" << '\n' <<  colm(m) << '\n';
  }

//  RSINFO(high_obs_)
  Eigen::Matrix<float, 780, 1> high_obs_in;
  Eigen::Matrix<float, 96, 1> high_latent_in;
  high_obs_in << high_obs_;
  Eigen::VectorXf high_latent = encoder_.forward(high_obs_in);
  high_latent_in << high_latent;
//  RSINFO(high_latent_in)
  Eigen::VectorXf high_action = high_actor_.forward(high_latent_in);
//  RSINFO(high_action)
  double high_action_norm = high_action.norm();
  high_action = high_action / (high_action_norm + 1e-8);
  high_action = high_action * 2.5 * (1/((1+exp(-high_action_norm)) + 1e-8));
  return high_action;
}

Eigen::VectorXf raibotLearningController::obsScalingAndGetAction() {

  /// normalize the obs
  obs_ = raibotController_.getObservation().cast<float>();
  for (int i = 0; i < obs_.size(); ++i) {
    obs_(i) = (obs_(i) - obsMean_(i)) / std::sqrt(obsVariance_(i) + 1e-8);
    if (obs_(i) > 10) { obs_(i) = 10.0; }
    else if (obs_(i) < -10) { obs_(i) = -10.0; }
  }
  /// forward the obs to the estimator
  Eigen::Matrix<float, 30, 1> e_in;
  e_in = obs_.tail(obs_.size() - 3);
  Eigen::VectorXf e_out = estimator_.forward(e_in);
  estUnscaled_ = e_out;
  /// normalize the output of estimator
  for (int i = 0; i < e_out.size(); ++i) {
    e_out(i) = (e_out(i) - eoutMean_(i)) / std::sqrt(eoutVariance_(i) + 1e-8);
    if (e_out(i) > 10) { e_out(i) = 10.0; }
    else if (e_out(i) < -10) { e_out(i) = -10.0; }
  }
  /// concat obs and e_out and forward to the actor
  Eigen::Matrix<float, 41, 1> actor_input;
  actor_input << obs_, e_out;
  Eigen::VectorXf action = actor_.forward(actor_input);
  return action;
}

bool raibotLearningController::warmUp(raisim::World *world) {
  auto* raibot = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot"));
  auto* raibot_vicon = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot_vicon"));

  Eigen::VectorXd jointPGain(raibot->getDOF());
  Eigen::VectorXd jointDGain(raibot->getDOF());
  Eigen::VectorXd gc_init;
  Eigen::VectorXd gv_init;

  jointPGain.setConstant(100.0);
  jointDGain.setConstant(0.5);
  raibotController_.getInitState(gc_init, gv_init);
  raibot->setPdGains(jointPGain, jointDGain);
  raibot->setPdTarget(gc_init, gv_init);
  raibot_vicon->setPdGains(jointPGain, jointDGain);
  raibot_vicon->setPdTarget(gc_init, gv_init);

  return true;
}

bool raibotLearningController::reset(raisim::World *world) {
  raibotController_.reset(world);
  actor_.initHidden();
  estimator_.initHidden();
  encoder_.initHidden();
  clk_ = 0;
  pd_clk_ = 0;
  return true;
}

bool raibotLearningController::terminate(raisim::World *world) { return true; }

bool raibotLearningController::stop(raisim::World *world) { return true; }

extern "C" Controller * create() {
  return new raibotLearningController;
}

extern "C" void destroy(Controller *p) {
  delete p;
}

void raibotLearningController::setCommand(const std::shared_ptr<raisin_interfaces::srv::Vector3::Request> request,
                                          std::shared_ptr<raisin_interfaces::srv::Vector3::Response> response)
try {
  Eigen::Vector3f command;
  command << request->x, request->y, request->z;
  raibotController_.setCommand(command);
  response->success = true;
} catch (const std::exception &e) {
  response->success = false;
  response->message = e.what();
}

//void raibotLearningController::setCommandByTopic(const sensor_msgs::msg::Joy::SharedPtr msg)
//try {
//  Eigen::Vector3f command;
//  auto limit = [this](auto msg, int commandType) {
//    return normal_limit_[commandType] + (-msg->axes[2] + 1) *
//    (boost_limit_[commandType] - normal_limit_[commandType]) / 2;
//  };
//
//  /// axes 1 is x. 0 is y. 2 is z.
//  if (msg->axes[1] > 0) { msg->axes[1] *= limit(msg, 0); }
//  if (msg->axes[1] < 0) { msg->axes[1] *= limit(msg, 1); }
//  msg->axes[0] *= limit(msg, 2);
//  msg->axes[3] *= limit(msg, 3);
//  command = {msg->axes[1], msg->axes[0], msg->axes[3]};
//  raibotController_.setCommand(command);
//
//  /// cmd limit modification
//  if ((std::abs(msg->axes[6]) > 0.5 || std::abs(msg->axes[7]) > 0.5) && !button_press_buffer_) {
//    if (msg->buttons[1] > 0.5) {
//      boost_limit_[0] += 0.1 * msg->axes[7];
//      boost_limit_[3] += -0.1 * msg->axes[6];
//    } else {
//      normal_limit_[0] += 0.1 * msg->axes[7];
//      normal_limit_[3] += -0.1 * msg->axes[6];
//    }
//    button_press_buffer_ = true;
//  } else {
//    button_press_buffer_ = false;
//  }
//} catch (const std::exception &e) {
//  std::cout << e.what();
//}

//void raibotLearningController::velocityLimitDisplayCallback()
//{
//  auto lim_msg = raisin_raibot::msg::VelocityLimit();
//
//  lim_msg.v_lim1 = normal_limit_[0];
//  lim_msg.v_lim2 = normal_limit_[1];
//  lim_msg.v_lim3 = normal_limit_[2];
//  lim_msg.v_lim4 = normal_limit_[3];
//
//  lim_msg.b_lim1 = boost_limit_[0];
//  lim_msg.b_lim2 = boost_limit_[1];
//  lim_msg.b_lim3 = boost_limit_[2];
//  lim_msg.b_lim4 = boost_limit_[3];
//
//  limitPublisher_->publish(lim_msg);
//}

}

}