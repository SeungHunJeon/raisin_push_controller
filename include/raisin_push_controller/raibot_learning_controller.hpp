//
// Created by suyoung on 8/7/21.
//
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "raisim/World.hpp"
#include "helper/BasicEigenTypes.hpp"
#include "raisin_push_controller/raibotController.hpp"
#include "raisin_parameter/parameter_container.hpp"
#include "raisin_controller/controller.hpp"
#include "raisin_interfaces/srv/vector3.hpp"
#include "raisin_data_logger/raisin_data_logger.hpp"
#include "helper/neuralNet.hpp"
// #include "raisin_raibot/msg/velocity_limit.hpp"

namespace raisin {

namespace controller {

class raibotLearningController : public Controller {

 public:
  raibotLearningController();
  bool create(raisim::World *world) final;
  bool init(raisim::World *world) final;
  Eigen::VectorXf obsScalingAndGetAction();
  Eigen::VectorXf high_obsScalingAndGetAction();
  bool advance(raisim::World *world) final;
  bool warmUp(raisim::World *world);
  bool reset(raisim::World *world) final;
  bool terminate(raisim::World *world) final;
  bool stop(raisim::World *world) final;

 private:
  void setCommand(
      const std::shared_ptr<raisin_interfaces::srv::Vector3::Request> request,
      std::shared_ptr<raisin_interfaces::srv::Vector3::Response> response
      );
//  void setCommandByTopic(const sensor_msgs::msg::Joy::SharedPtr msg);
//  void velocityLimitDisplayCallback();

  rclcpp::Service<raisin_interfaces::srv::Vector3>::SharedPtr serviceSetCommand_;
//  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber_;

//  rclcpp::Publisher<raisin_raibot::msg::VelocityLimit>::SharedPtr limitPublisher_;
//  rclcpp::TimerBase::SharedPtr limitTimer_;

  raisim::raibotController raibotController_;
  Eigen::VectorXf obs_;
  Eigen::VectorXf estUnscaled_;
  Eigen::VectorXf actor_input_;

  Eigen::VectorXf obsMean_;
  Eigen::VectorXf high_obsMean_;
  Eigen::VectorXf obsVariance_;
  Eigen::VectorXf high_obsVariance_;
  Eigen::VectorXf eoutMean_;
  Eigen::VectorXf eoutVariance_;
  Eigen::VectorXf subgoal_command;
  Eigen::VectorXf high_obs_;

  raisim::nn::LSTM_MLP<float, 41, 12, raisim::nn::ActivationType::leaky_relu> actor_;
  raisim::nn::LSTM_MLP<float, 30, 8, raisim::nn::ActivationType::leaky_relu> estimator_;

  raisim::nn::LSTM<float, 720, 96, raisim::nn::ActivationType::leaky_relu> encoder_;
  raisim::nn::Linear<float, 96, 3, raisim::nn::ActivationType::leaky_relu> high_actor_;


  int clk_ = 0;
  int pd_clk_ = 0;
  double high_control_dt_;
  double control_dt_;
  double communication_dt_;
//  Eigen::VectorXd normal_limit_;
//  Eigen::VectorXd boost_limit_;
//  bool button_press_buffer_;

  parameter::ParameterContainer & param_;
  std::unique_ptr<raisin::DataLogger> log_;

};

}

}


