#include "jaco_control/JacoRos.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "enable_torque");
    jaco_control::JacoRos jaco;
    jaco.initialize();
    jaco.setTorqueMode(true);
    return 0;
}
