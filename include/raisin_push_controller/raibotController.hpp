// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#ifndef RSG_RAIBOT__RAIBOTCONTROLLER_HPP_
#define RSG_RAIBOT__RAIBOTCONTROLLER_HPP_

#include <set>
#include "raisim/World.hpp"
#include "raisin_parameter/parameter_container.hpp"

namespace raisim {

class raibotController {

 public:

  bool create(raisim::World *world) {
    auto* raibot = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot"));
    auto* raibot_vicon = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot_vicon"));
    raisin::parameter::ParameterContainer & param_(raisin::parameter::ParameterContainer::getRoot()["Raibot"]);
    param_.loadFromPackageParameterFile("raisin_raibot");
    useVicon_ = param_("use_vicon");


    Obj_ = reinterpret_cast<raisim::SingleBodyObject*>(world->getObject("Object"));
    Obj_vicon_ = reinterpret_cast<raisim::SingleBodyObject*>(world->getObject("Object_vicon"));
    if(useVicon_)
    {
      Obj_->setPosition(10, 20, 1);
    }

    auto Target_ = reinterpret_cast<raisim::SingleBodyObject*>(world->getObject("Target"));
    auto Target_box = reinterpret_cast<raisim::Box*>(world->getObject("Target"));
    obj_geometry = Target_box->getDim();
    RSINFO(obj_geometry.e())
    raisim::Vec<3> obj_pos;
    double phi = 0.2;
    Obj_vicon_->getPosition(obj_pos);
    obj_pos[0] = obj_pos[0] + sqrt(2)*cos(phi*2*M_PI);
    obj_pos[1] = obj_pos[1] + sqrt(2)*sin(phi*2*M_PI);
    Target_->setPosition(obj_pos[0], obj_pos[1], obj_pos[2]);
    Target_->setAppearance("1, 0, 0, 0.3");
//    high_command_ = obj_pos.e() * 2;
    high_command_ = obj_pos.e();
    /// get robot data
    gcDim_ = raibot->getGeneralizedCoordinateDim();
    gvDim_ = raibot->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_);
    gv_.setZero(gvDim_);
    gc_init_.setZero(gcDim_);
    gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_);
    vTarget_.setZero(gvDim_);
    pTarget12_.setZero(nJoints_);

    /// this is nominal configuration of raibot
    gc_init_ << 0, 0, 0.4725, 1, 0.0, 0.0, 0.0,
                0.0, 0.559836, -1.119672, -0.0, 0.559836, -1.119672, 0.0, 0.559836, -1.119672, -0.0, 0.559836, -1.119672;
    raibot->setState(gc_init_, gv_init_);
    raibot_vicon->setState(gc_init_, gv_init_);
    /// set pd gains
    jointPGain_.setZero(gvDim_);
    jointDGain_.setZero(gvDim_);
    jointPGain_.tail(nJoints_).setConstant(50.0);
    jointDGain_.tail(nJoints_).setConstant(0.5);
    raibot->setPdGains(jointPGain_, jointDGain_);
    raibot_vicon->setPdGains(jointPGain_, jointDGain_);
    raibot->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));
    raibot_vicon->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// vector dimensions
    obDim_ = 33;
    estDim_ = 8;
    actionDim_ = nJoints_;
    high_proDim_ = 15;
    high_extDim_ = 18;
    high_actionDim_ = 3;
    high_historyNum_ = 19;
    high_blockDim_ = high_proDim_ + high_extDim_ + high_actionDim_;
    high_obDim_ = high_blockDim_ * (high_historyNum_ + 1);

    high_pro_history_.resize(high_historyNum_);
    high_ext_history_.resize(high_historyNum_);
    high_act_history_.resize(high_historyNum_+1);

    for (int i = 0; i < high_historyNum_; i++)
    {
      high_ext_history_[i].setZero(high_extDim_);
      high_pro_history_[i].setZero(high_proDim_);
    }

    for (int i = 0; i < high_historyNum_ + 1; i++)
    {
      high_act_history_[i].setZero(high_actionDim_);
    }

    high_pro_obDouble_.setZero(high_proDim_);
    high_ext_obDouble_.setZero(high_extDim_);
    high_act_obDouble_.setZero(high_actionDim_);

    high_obDouble_.setZero((high_historyNum_+1)*(high_blockDim_));
    actionMean_.setZero(actionDim_);
    actionStd_.setZero(actionDim_);
    high_actionMean_.setZero(high_actionDim_);
    high_actionStd_.setZero(high_actionDim_);
    obDouble_.setZero(obDim_);
    command_.setZero();


    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);
    actionStd_.setConstant(0.3);

    high_actionMean_.setConstant(0.0);
    high_actionStd_.setConstant(1.0);

    updateObservation(world);
    updateHighObservation(world);

    return true;
  }

  bool init(raisim::World *world) { return true; }

  bool reset(raisim::World *world) {
    command_ << 0., 0., 0.;
    updateObservation(world);
    updateHighObservation(world);
    return true;
  }

  Eigen::VectorXf high_advance(raisim::World *world, const Eigen::Ref<EigenVec>& action) {
    Eigen::VectorXf subgoal_command;
    subgoal_command = action.cwiseQuotient(high_actionStd_.cast<float>());
    /// Use this command via Controller.setCommand
    return subgoal_command;
  }

  bool advance(raisim::World *world, const Eigen::Ref<EigenVec>& action) {
    auto* raibot = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot"));
    auto* raibot_vicon = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot_vicon"));
    /// action scaling
    pTarget12_ = action.cast<double>();
    pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    pTarget12_ += actionMean_;
    pTarget_.tail(nJoints_) = pTarget12_;
    raibot->setPdTarget(pTarget_, vTarget_);
    raibot_vicon->setPdTarget(pTarget_, vTarget_);
    return true;
  }

  void getposdist(Eigen::Vector3d &vec_dist, Eigen::Vector2d &pos, double &dist_min) {
    double dist;
    dist = vec_dist.head(2).norm() + 1e-8;
    pos = vec_dist.head(2) * (1.0/dist);
    dist_min = std::min(2.0, dist);
  }

  void updateHighActionHistory(const Eigen::Ref<EigenVec> &action) {
    std::rotate(high_act_history_.begin(), high_act_history_.begin()+1, high_act_history_.end());
    high_act_history_[(high_historyNum_ + 1) - 1] = action.cast<double>();
  }

  void updateHighHistory() {
    std::rotate(high_ext_history_.begin(), high_ext_history_.begin()+1, high_ext_history_.end());
    high_ext_history_[high_historyNum_ - 1] = high_ext_obDouble_;
    std::rotate(high_pro_history_.begin(), high_pro_history_.begin()+1, high_pro_history_.end());
    high_pro_history_[high_historyNum_ - 1] = high_pro_obDouble_;
  }

  void updateHighObservation(raisim::World *world) {

    raisim::Vec<3> ee_pos_w, ee_vel_w, obj_pos, obj_vel, obj_avel;
    Eigen::Vector3d ee_to_obj, obj_to_target, ee_to_target;

    auto* raibot = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot"));

    if(useVicon_)
    {
      raibot = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot_vicon"));
    }

    raibot->getState(gc_, gv_);
    raisim::Vec<4> quat;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot_vicon);
    bodyAngularVel_ = rot_vicon.e().transpose() * gv_.segment(3, 3);
    bodyLinearVel_ = rot_vicon.e().transpose() * gv_.segment(0,3);
    raibot->getFramePosition(raibot->getFrameIdxByLinkName("arm_link"), ee_pos_w);
    raibot->getFrameVelocity(raibot->getFrameIdxByLinkName("arm_link"), ee_vel_w);

    raisim::Vec<3> LF_FOOT_Pos_w_, RF_FOOT_Pos_w_;
    raibot->getFramePosition(raibot->getFrameIdxByLinkName("LF_FOOT"), LF_FOOT_Pos_w_);
    raibot->getFramePosition(raibot->getFrameIdxByLinkName("RF_FOOT"), RF_FOOT_Pos_w_);

    high_pro_obDouble_.segment(0,3) = rot_vicon.e().row(2);
    high_pro_obDouble_.segment(3,3) << bodyLinearVel_;
    high_pro_obDouble_.segment(6,3) << bodyAngularVel_;
    high_pro_obDouble_.segment(9,3) = rot_vicon.e().transpose()*(LF_FOOT_Pos_w_.e() - ee_pos_w.e());
    high_pro_obDouble_.segment(12,3) = rot_vicon.e().transpose()*(RF_FOOT_Pos_w_.e() - ee_pos_w.e());
    /// TODO add arm_link in the URDF

    if(useVicon_)
    {
      Obj_vicon_->getPosition(obj_pos);
      Obj_vicon_->getLinearVelocity(obj_vel);
      Obj_vicon_->getAngularVelocity(obj_avel);
    }

    else
    {
      Obj_->getPosition(obj_pos);
      Obj_->getLinearVelocity(obj_vel);
      Obj_->getAngularVelocity(obj_avel);
    }



    /// TODO add high_command subscriber
    ee_to_obj = (obj_pos.e() - ee_pos_w.e());
    obj_to_target = (high_command_ - obj_pos.e());
    ee_to_target = (high_command_ - ee_pos_w.e());

    /// Set height as 0
    ee_to_obj(2) = 0;
    obj_to_target(2) = 0;
    ee_to_target(2) = 0;

    /// Into robot frame
    ee_to_obj = rot_vicon.e().transpose() * ee_to_obj;
    obj_to_target = rot_vicon.e().transpose() * obj_to_target;
    ee_to_target = rot_vicon.e().transpose() * ee_to_target;

    Eigen::Vector2d pos_temp;
    double dist_temp_min;
    getposdist(ee_to_obj, pos_temp, dist_temp_min);
    high_ext_obDouble_.segment(0,2) << pos_temp;
    high_ext_obDouble_.segment(2,1) << dist_temp_min;
    getposdist(obj_to_target, pos_temp, dist_temp_min);
    high_ext_obDouble_.segment(3,2) << pos_temp;
    high_ext_obDouble_.segment(5,1) << dist_temp_min;
    getposdist(ee_to_target, pos_temp, dist_temp_min);
    high_ext_obDouble_.segment(6,2) << pos_temp;
    high_ext_obDouble_.segment(8,1) << dist_temp_min;
    high_ext_obDouble_.segment(9,3) << rot_vicon.e().transpose() * obj_vel.e();
    high_ext_obDouble_.segment(12,3) = rot_vicon.e().row(0) - Obj_->getOrientation().e().row(0);
    high_ext_obDouble_.segment(15,3) << obj_geometry.e(); /// Only for Box
    // update History
    for (int i=0; i< high_historyNum_; i++) {
      high_obDouble_.segment(high_blockDim_*i,
                             high_proDim_) = high_pro_history_[i];
      high_obDouble_.segment(high_blockDim_*i + high_proDim_,
                        high_extDim_) = high_ext_history_[i];

      high_obDouble_.segment(high_blockDim_*i + high_proDim_ + high_extDim_,
                             high_actionDim_) = high_act_history_[i];

    }
    // current state
    high_obDouble_.segment(high_blockDim_*high_historyNum_, high_proDim_) =
        high_pro_obDouble_;
    high_obDouble_.segment(high_blockDim_*high_historyNum_ + high_proDim_, high_extDim_)
        = high_ext_obDouble_;
    high_obDouble_.segment(high_blockDim_*high_historyNum_ + high_proDim_ + high_extDim_, high_actionDim_)
        = high_act_history_.back();

  }

  void updateObservation(raisim::World *world) {
    auto* raibot = reinterpret_cast<raisim::ArticulatedSystem*>(world->getObject("robot"));
    raibot->getState(gc_, gv_);

    raisim::Vec<4> quat;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot_);
    bodyAngularVel_ = rot_.e().transpose() * gv_.segment(3, 3);
    obDouble_ << command_, /// command 3
        rot_.e().row(2).transpose(), /// body orientation: z-axis 3
        bodyAngularVel_, /// body angular velocity 3
        gc_.tail(12), /// joint angles 12
        gv_.tail(12); /// joint velocity 12
  }

  void setCommand(const Eigen::Ref<EigenVec>& command) {
    command_ = command.cast<double>();

    // project to centrifugal accel. vxy * wz = 0.3 * g
    if (std::abs(command_(2)) - 2.943 / (command_.head(2).norm() + 1e-8) > 0) {
      command_(2) = std::copysign(2.943 / (command_.head(2).norm() + 1e-8), command_(2));
    }
  }

  Eigen::Vector3d getCommand() { return command_; }

  const Eigen::VectorXd& getObservation() { return obDouble_; }

  void getHighObservation(Eigen::VectorXf &observation) {
    observation = high_obDouble_.cast<float>();
//    RSINFO(observation)
  }
//    return high_obDouble_;}

  int getHighHistoryNum() const {return high_historyNum_;}

  int getHighObDim() const {return high_obDim_;}

  int getObDim() const { return obDim_; }

  int getEstDim() const { return estDim_; }

  int getActionDim() const { return actionDim_; }

  Eigen::VectorXd getJointPGain() const { return jointPGain_; }

  Eigen::VectorXd getJointDGain() const { return jointDGain_; }

  Eigen::VectorXd getJointPTarget() const { return pTarget12_; }

  void getInitState(Eigen::VectorXd &gc, Eigen::VectorXd &gv) {
    gc.resize(gcDim_);
    gv.resize(gvDim_);
    gc << gc_init_;
    gv << gv_init_;
  }

private:
  int gcDim_;
  int gvDim_;
  int nJoints_;
  Eigen::VectorXd gc_init_;
  Eigen::VectorXd gv_init_;
  Eigen::VectorXd jointPGain_;
  Eigen::VectorXd jointDGain_;

  Eigen::VectorXd gc_;
  Eigen::VectorXd gv_;
  raisim::Mat<3,3> rot_;
  raisim::Mat<3,3> rot_vicon;
  Eigen::Vector3d bodyAngularVel_;
  Eigen::Vector3d bodyLinearVel_;
  raisim::Vec<3> obj_geometry;

  int obDim_;
  int high_obDim_;
  int estDim_;
  int actionDim_;
  int high_historyNum_;
  int high_proDim_;
  int high_extDim_;
  int high_actionDim_;
  int high_blockDim_;
  Eigen::Vector3d command_;
  Eigen::Vector3d high_command_;
  Eigen::VectorXd obDouble_;
  Eigen::VectorXd high_obDouble_;
  Eigen::VectorXd high_pro_obDouble_;
  Eigen::VectorXd high_ext_obDouble_;
  Eigen::VectorXd high_act_obDouble_;
  std::vector<Eigen::VectorXd> high_pro_history_;
  std::vector<Eigen::VectorXd> high_ext_history_;
  std::vector<Eigen::VectorXd> high_act_history_;

  bool useVicon_ = false;
  raisim::SingleBodyObject* Obj_;
  raisim::SingleBodyObject* Obj_vicon_;
  Eigen::VectorXd pTarget_;
  Eigen::VectorXd pTarget12_;
  Eigen::VectorXd vTarget_;
  Eigen::VectorXd actionMean_;
  Eigen::VectorXd actionStd_;
  Eigen::VectorXd high_actionMean_;
  Eigen::VectorXd high_actionStd_;
};

} // namespace raisim

#endif    // RSG_RAIBOT__RAIBOTCONTROLLER_HPP_