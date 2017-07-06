/** @file VrepInterface.hpp

 * Acts as the interface between ROS and V-REP for controlling the Jaco arm.
 * Uses V-REP's built in inverse kinematics by moving a target dummy
 * and uses V-REP's C++ api as this is faster than V-REP's ROS api.
 */
#ifndef JACO_CONTROL_VREP_IK_INTERFACE_HPP_
#define JACO_CONTROL_VREP_IK_INTERFACE_HPP_

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

class VrepIKInterface {

public:
    VrepIKInterface();
    ~VrepIKInterface();

    /** Connect to V-REP and ROS */
    void initialize(ros::NodeHandle& n);

private:
    /** VREP connection id */
    int clientID_;
    /** Target dummy handle */
    int dummyHandle_;
    /** Subscriber to velocity commands */
    ros::Subscriber velSub_;
    /** Last velocity command received */
    geometry_msgs::Twist::ConstPtr vel_;

    ros::WallTimer timer_;
    float stepTime_;
    void timerCb(const ros::WallTimerEvent& e);

    float pos_[3];

    /** Callback for velocity commands */
    void velCb(const geometry_msgs::Twist::ConstPtr& msg);
    ros::Time commandTime_;
};


#endif /* JACO_CONTROL_VREP_IK_INTERFACE_HPP_ */
