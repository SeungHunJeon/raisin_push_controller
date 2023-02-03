//
// Created by suyoung on 8/7/21.
//

#include <filesystem>
#include "ament_index_cpp/get_package_prefix.hpp"
#include "raisin_jsh_controller/raibot_learning_controller.hpp"

namespace raisin {

namespace controller {

using std::placeholders::_1;
using std::placeholders::_2;

raibotLearningController::raibotLearningController()
: Controller("JSH_controller"),
  actor_({512, 400, 128}),
  param_(parameter::ParameterContainer::getRoot()["raibotLearningController"]),
  raibotParam_(parameter::ParameterContainer::getRoot()["Raibot"])
{
  param_.loadFromPackageParameterFile("JSH_controller");
  raibotParam_.loadFromPackageParameterFile("raisin_raibot");

  serviceSetCommand_ = this->create_service<raisin_interfaces::srv::Vector3>(
      "JSH_controller/set_command", std::bind(&raibotLearningController::setCommand, this, _1, _2)
      );
      RSINFO(1)
}

bool raibotLearningController::create(raisim::World *world) {
  control_dt_ = 0.005;
  communication_dt_ = 0.001;
  raibotController_.create(world);
  raibotController_.reset();
  isRealRobot_ = raibotParam_("real_robot");

  std::filesystem::path pack_path(ament_index_cpp::get_package_prefix("JSH_controller"));
  std::filesystem::path actor_path = pack_path / std::string(param_("actor_path"));
  std::filesystem::path obs_mean_path = pack_path / std::string(param_("obs_mean_path"));
  std::filesystem::path obs_var_path = pack_path / std::string(param_("obs_var_path"));

  RSINFO(actor_path.string())
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  actor_.readParamFromTxt(actor_path.string());



  std::string in_line;

  std::ifstream obsMean_file(obs_mean_path.string());
  std::ifstream obsVariance_file(obs_var_path.string());

  obs_.setZero(raibotController_.getObDim());
  obsMean_.setZero(raibotController_.getObDim());
  obsVariance_.setZero(raibotController_.getObDim());
  actor_input_.setZero(raibotController_.getObDim());
  if (obsMean_file.is_open()) {
    for (int i = 0; i < obsMean_.size(); ++i) {
      std::getline(obsMean_file, in_line);
      obsMean_(i) = std::stof(in_line);
    }
  }

  if (obsVariance_file.is_open()) {
    for (int i = 0; i < obsVariance_.size(); ++i) {
      std::getline(obsVariance_file, in_line);
      obsVariance_(i) = std::stof(in_line);
    }
  }

  obsMean_file.close();
  obsVariance_file.close();
  return true;
}

bool raibotLearningController::init(raisim::World *world) {
  raibotController_.init(world);
//  raibotController_.create(world);
//  raibotController_.reset();

  return true;
}

bool raibotLearningController::advance(raisim::World *world) {
  /// 100Hz controller
  if(clk_ == 0) {
    Eigen::Vector3f command;
    command << 2,1,0;
    raibotController_.setCommand(command);
  }

  if(clk_ % int(control_dt_ / communication_dt_ + 1e-10) == 0) {
    raibotController_.updateObservation();
    raibotController_.advance(obsScalingAndGetAction().head(12));
  }

  raibotController_.updateHistory();
  raibotController_.updateStateVariables();
//  raibotController_.updateFilter(world, isRealRobot_);
  if (pd_clk_ < 100) {
    warmUp(world);
    ++pd_clk_;
  }
  clk_++;
  return true;
}

bool raibotLearningController::warmUp(raisim::World *world) {
  auto* raibot = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot"));

  Eigen::VectorXd jointPGain(raibot->getDOF());
  Eigen::VectorXd jointDGain(raibot->getDOF());
  Eigen::VectorXd gc_init;
  Eigen::VectorXd gv_init;

  jointPGain.setConstant(100.0);
  jointDGain.setConstant(0.5);
  raibotController_.getStateInit(gc_init, gv_init);
  raibot->setPdGains(jointPGain, jointDGain);
  raibot->setPdTarget(gc_init, gv_init);

  return true;
}

Eigen::VectorXf raibotLearningController::obsScalingAndGetAction() {

  /// normalize the obs
  obs_ = raibotController_.getObservation().cast<float>();
  for (int i = 0; i < obs_.size(); ++i) {
    obs_(i) = (obs_(i) - obsMean_(i)) / std::sqrt(obsVariance_(i) + 1e-8);
  }
  /// concat obs and e_out and forward to the actor
  Eigen::Matrix<float, 133, 1> actor_input;
  actor_input << obs_;
  Eigen::VectorXf action = actor_.forward(actor_input);

  return action;
}

bool raibotLearningController::reset(raisim::World *world) {
  raibotController_.reset();
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

// torch::Tensor raibotLearningController::eigenVectorToTorchTensor(const Eigen::VectorXf &e) {
//   auto t = torch::empty({1, e.size()});
//   Eigen::Map<Eigen::VectorXf> ef(t.data_ptr<float>(), t.size(1), t.size(0));
//   ef = e.cast<float>();
//   t.requires_grad_(false);
//   return t;
// }

// Eigen::VectorXf raibotLearningController::torchTensorToEigenVector(const torch::Tensor &t) {
//   Eigen::Map<Eigen::VectorXf> e(t.data_ptr<float>(), t.size(1), t.size(0));
//   return e;
// }

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

}

}
