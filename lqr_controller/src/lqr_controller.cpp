//  March/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "lqr_controller/lqr_controller.h"

using namespace Eigen;
using namespace std;
//Constructor
StateDependentLqr::StateDependentLqr(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  geometric_controller_(nh, nh_private) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &StateDependentLqr::CmdLoopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &StateDependentLqr::StatusLoopCallback, this); // Define timer for constant loop rate

  double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;

  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_z_, -Kvel_z_;
  tau << tau_x, tau_y, tau_z;

  q_.resize(3);
  p_.resize(3);
  for (int i = 0; i < 3; ++i) {
    q_.at(i) << 0.0, 0.0;
    p_.at(i) << 0.0, 0.0;
  }
  a0 << a0_x, a0_y, a0_z;
  a1 << a1_x, a1_y, a1_z;

  a_fb << 0.0, 0.0, 0.0;
  a_dob << 0.0, 0.0, 0.0;

}
StateDependentLqr::~StateDependentLqr() {
  //Destructor
}

void StateDependentLqr::CmdLoopCallback(const ros::TimerEvent& event){
  // /// Compute BodyRate commands using disturbance observer
  // /// From Hyuntae Kim
  geometric_controller_.getErrors(pos_error, vel_error);
  
  a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error; //feedforward term for trajectory error
  a_des = a_fb - a_dob - g_;

  geometric_controller_.setFeedthrough(true);
  geometric_controller_.setDesiredAcceleration(a_des);

}

void StateDependentLqr::StatusLoopCallback(const ros::TimerEvent& event){

}