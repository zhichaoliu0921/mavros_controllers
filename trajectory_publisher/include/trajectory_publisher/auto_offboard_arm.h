//  May/2018, ETHZ, Jaeyoung Lim, jalim@ethz.ch

#ifndef AUTOOFFBOARDARM_H
#define AUTOOFFBOARDARM_H

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>


class AutoOffboardArm
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer loop_timer_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::Time last_request_;

  mavros_msgs::State current_state_;
  mavros_msgs::SetMode offb_set_mode_;
  mavros_msgs::CommandBool arm_cmd_;

  void LoopCallback(const ros::TimerEvent& event);


public:
  AutoOffboardArm(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  };


#endif
