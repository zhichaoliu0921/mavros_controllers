//
// Created by jalim on 05.05.18.
//

#include "trajectory_publisher/auto_offboard_arm.h"

AutoOffboardArm::AutoOffboardArm(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
  nh_(nh),
  nh_private_(nh_private) {

  loop_timer_ = nh_.createTimer(ros::Duration(1), &AutoOffboardArm::LoopCallback, this); // Define timer for constant loop rate
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

void AutoOffboardArm::LoopCallback(const ros::TimerEvent& event){
  // Enable OFFBoard mode and arm automatically
  // This is only run if the vehicle is simulated
  arm_cmd_.request.value = true;
  offb_set_mode_.request.custom_mode = "OFFBOARD";
  if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
    if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent){
      ROS_INFO("Offboard enabled");
    }
    last_request_ = ros::Time::now();
  } else {
    if( !current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
      if( arming_client_.call(arm_cmd_) && arm_cmd_.response.success){
        ROS_INFO("Vehicle armed");
      }
      last_request_ = ros::Time::now();
    }
  }
}