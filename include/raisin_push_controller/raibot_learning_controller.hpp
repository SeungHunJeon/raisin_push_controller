//
// Created by suyoung on 8/7/21.
//
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "raisim/World.hpp"
#include "raisin_jsh_controller/RaiboPositionController.hpp"
#include "raisin_parameter/parameter_container.hpp"
#include "raisin_controller/controller.hpp"
#include "raisin_interfaces/srv/vector3.hpp"
#include "helper/BasicEigenTypes.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "raisin_interfaces/srv/vector3.hpp"
#include "raisin_data_logger/raisin_data_logger.hpp"
#include "helper/neuralNet.hpp"
#include "raisin_raibot/msg/velocity_limit.hpp"

namespace raisin {

namespace controller {

class raibotLearningController : public Controller {

 public:
  raibotLearningController();
  bool create(raisim::World *world) final;
  bool init(raisim::World *world) final;
  Eigen::VectorXf obsScalingAndGetAction();
  bool advance(raisim::World *world) final;
  bool reset(raisim::World *world) final;
  bool terminate(raisim::World *world) final;
  bool stop(raisim::World *world) final;
  bool warmUp(raisim::World *world);
  // torch::Tensor eigenVectorToTorchTensor(const Eigen::VectorXf &e);
  // Eigen::VectorXf torchTensorToEigenVector(const torch::Tensor &t);

 private:
  void setCommand(
      const std::shared_ptr<raisin_interfaces::srv::Vector3::Request> request,
      std::shared_ptr<raisin_interfaces::srv::Vector3::Response> response
      );

  rclcpp::Service<raisin_interfaces::srv::Vector3>::SharedPtr serviceSetCommand_;

  raisim::RaiboPositionController raibotController_;
  Eigen::VectorXf obs_, obsMean_, obsVariance_, actor_input_;
  bool isRealRobot_;
  int pd_clk_ = 0;
  int clk_ = 0;
  double control_dt_, communication_dt_;
  raisim::nn::Linear<float, 133, 12, raisim::nn::ActivationType::leaky_relu> actor_;

  parameter::ParameterContainer & param_;
  parameter::ParameterContainer & raibotParam_;

};

}

}


