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
#include <mutex>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

class VrepInterface {

public:
    typedef std::vector<float> (*TorqueCallback)(
            const sensor_msgs::JointState& jointState);

    VrepInterface();
    ~VrepInterface();

    /** These mode setters should be called before initialize */
    /** Set to torque mode. Activates synchronous mode with V-REP.
     * Requires pointer to function to calculate torques. */
    void setTorqueMode(TorqueCallback calcTorque);
    /** Set to use V-REP's IK for velocity commands. Step time is how long to
     *  run each velocity command for. Should match with POMDP step time. */
    void setVelMode(float stepTime);

    /** Connect to V-REP and ROS */
    void initialize(ros::NodeHandle& n);
    /** Connect to V-REP and ROS with a private node and callback queue */
    void initialize();

    /** V-REP remote api wrappers */
    int getVrepHandle(std::string name);
    tf::Vector3 getVrepPosition(int handle, bool startStream);
    tf::Vector3 getVrepOrientation(int handle, bool startStream);
    void setVrepPosition(int handle, const tf::Vector3& v);
    void setVrepOrientation(int handle, const tf::Vector3& v);

    /** Jaco arm related functions */
    /** Get current joint positions */
    std::vector<float> getVrepJointPosition();
    /** Set joint torques */
    void setVrepJointTorque(const std::vector<float>& torques);
    /** Set joint target positions */
    void setVrepJointPosition(const std::vector<float>& pos);
    /** Set velocity command */
    void setVrepEefVel(geometry_msgs::Twist vel);
    /** Disable V-REP's joint position controller */
    void disableVrepControl();
    /** Enable V-REP's joint position controller */
    void enableVrepControl();

private:
    /** Private node */
    std::unique_ptr<ros::NodeHandle> node_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    ros::CallbackQueue callbackQueue_;

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
    /** Publish state info and send commands to V-REP. Executed at a fix rate. */
    void publishWorker(const ros::WallTimerEvent& e);

    /** Update the target dummy position if using V-REP's IK */
    void updateTargetDummy();

    /** Actionlib callback for trajectory command */
    void trajCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

    std::vector<double> interpolate( const std::vector<double>& last,
            const std::vector<double>& current, double alpha);

    /** ROS actionlib server for trajectory commands */
    std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> trajAS_;

    /** VREP connection id */
    int clientID_;
    /** Target dummy handle for V-REP IK mode */
    int dummyHandle_;
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
    ros::Publisher tempPub_;

    /** Last velocity command received */
    geometry_msgs::Twist vel_;
    bool validVel_;
    /** Position of dummy */
    tf::Vector3 dummyPos_;
    /** Time at last vel command */
    ros::Time velTime_;
    /** Stores joint state */
    sensor_msgs::JointState jointState_;
    control_msgs::FollowJointTrajectoryFeedback feedback_;

    /** Number of joints in jaco arm = 6 */
    int numArmJoints_;
    /** Number of finger joints = 3 */
    int numFingerJoints_;
    /** Rate for publishing joint info in Hz */
    float feedbackRate_;
    /** Rate for setting joint position commands */
    float posUpdateRate_;

    /** Timer for publishWorker */
    ros::WallTimer publishWorkerTimer_;

    /** Torque mode for robot control. */
    bool torqueMode_;
    /** Pointer to function to calculate torques if using torque mode. */
    TorqueCallback calcTorque_;
    /** Whether to use synchronous mode with V-REP */
    bool sync_;
    /** Enable velocity commands */
    bool velMode_;
    /** Step time for vel action, should match POMDP step */
    float velStepTime_;

    /** Mutex for V-REP api calls */
    std::mutex mutex_;
};

#endif /* JACO_CONTROL_VREP_INTERFACE_HPP_ */
