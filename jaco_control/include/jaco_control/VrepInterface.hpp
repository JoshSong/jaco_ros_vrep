/** @file VrepInterface.hpp
 * Acts as the interface between ROS and V-REP for controlling the Jaco arm.
 * Implements ROS's control_msgs::FollowJointTrajectoryAction action interface,
 * and uses V-REP's C++ api as this is faster than V-REP's ROS api.
 */
#ifndef JACO_CONTROL_VREP_INTERFACE_HPP_
#define JACO_CONTROL_VREP_INTERFACE_HPP_

#include <unordered_map>
#include <string>
#include <iostream>
#include <queue>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>

class VrepInterface {

public:
    typedef std::vector<float> (*TorqueCallback)(
            const sensor_msgs::JointState& jointState);

    VrepInterface();

    /** Set to torque mode. Activates synchronous mode with V-REP.
     * Requires pointer to function to calculate torques. */
    void setTorqueMode(TorqueCallback calcTorque);

    /** Connect to V-REP and ROS */
    void initialize(ros::NodeHandle& n);

    /** Publish state info and send commands to V-REP. Executed at a fix rate. */
    void publishWorker(const ros::WallTimerEvent& e);

    /** Actionlib callback for trajectory command */
    void trajCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

private:
    /** Initialises jointStates and jointHandles. Return true if success.
     *  Part of the process is getting V-REP handles. The suffixCode can be
     *  given, which should match the number after # if used.
     */
    bool initJoints(std::string inPrefix, std::string outPrefix, int numJoints,
            sensor_msgs::JointState& jointState, std::vector<int>& jointHandles,
            int suffixCode = -1);

    /** Get joint info through V-REP remote api and update jointState_ msg */
    void updateJointState();
    /** Publish joint state/feedback to ROS */
    void publishJointInfo();

    /** V-REP remote api wrappers */
    /** Get current joint positions */
    std::vector<float> getVrepPosition();
    /** Set joint torques */
    void setVrepTorque(const std::vector<float>& torques);
    /** Set joint positions */
    void setVrepPosition(const std::vector<float>& pos);
    /** Disable V-REP's joint position controller */
    void disableVrepControl();
    /** Enable V-REP's joint position controller */
    void enableVrepControl();

    std::vector<double> interpolate( const std::vector<double>& last,
            const std::vector<double>& current, double alpha);

    /** ROS actionlib server for trajectory commands */
    std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> trajAS_;

    /** VREP connection id */
    int clientID_;
    /** VREP handles of arm joints */
    std::vector<int> jointHandles_;
    /** Offsets in radians to make urdf match simulated arm in VREP */
    std::vector<float> jointOffsets_;
    /** Direction modifiers for joints that rotate in the opposite direction */
    std::vector<int> jointDirs_;
    /** Maximum torque values for torque mode in Nm */
    std::vector<float> maxTorques_;
    /** Maximum joint velocity values for torque mode in rad/s */
    std::vector<float> maxVels_;

    /** Publisher for transformed current joint states */
    ros::Publisher jointPub_;
    /** Publisher for feedback states */
    ros::Publisher feedbackPub_;
    /** Subscriber to target torques */
    ros::Subscriber torqueSub_;
    ros::Publisher tempPub_;

    /** Stores joint state */
    sensor_msgs::JointState jointState_;
    control_msgs::FollowJointTrajectoryFeedback feedback_;

    /** Number of joints in jaco arm = 6 */
    int numArmJoints_;
    /** Number of finger joints = 3 */
    int numFingerJoints_;
    /** Rate for publishing joint info in Hz */
    double feedbackRate_;
    /** Rate for setting joint position commands */
    ros::Rate posUpdateRate_;

    /** Timer for publishWorker */
    ros::WallTimer publishWorkerTimer_;

    /** Torque mode for robot control. */
    bool torqueMode_;
    /** Pointer to function to calculate torques if using torque mode. */
    TorqueCallback calcTorque_;
    /** Whether to use synchronous mode with V-REP */
    bool sync_;
};

#endif /* JACO_CONTROL_VREP_INTERFACE_HPP_ */
