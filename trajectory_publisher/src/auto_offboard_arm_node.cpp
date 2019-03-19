
#include "trajectory_publisher/auto_offboard_arm.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    AutoOffboardArm autoarming(nh, nh_private);
    ros::spin();
    return 0;
}
