#ifndef JACO_CONTROL_ROS_INTERFACE_HPP_
#define JACO_CONTROL_ROS_INTERFACE_HPP_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kinova_msgs/CartesianForce.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/ArmPoseAction.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <mutex>
#include <memory>
#include <string>

namespace jaco_control {

class JacoRos {

public:
    JacoRos();

    void initialize();

    geometry_msgs::PoseStamped::ConstPtr getPose() const;
    geometry_msgs::WrenchStamped::ConstPtr getWrench() const;

    bool setTorqueMode(bool enabled);
    bool setAdmittanceMode(bool enabled);
    void setForce(kinova_msgs::CartesianForce forceMsg);
    void setVel(kinova_msgs::PoseVelocity velMsg);
    void setPose(geometry_msgs::Pose pose, bool wait=true, float timeout=10.f);
    void setFingers(float f1, float f2, float f3, bool wait=true, float timeout=10.f);

private:
    std::vector<float> loadGravParams() const;
    void timerCb(const ros::TimerEvent& e);

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void wrenchCb(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::CallbackQueue callbackQueue_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    ros::Publisher forcePub_;
    ros::Publisher velPub_;
    ros::Timer timer_;
    ros::ServiceClient torqueModeClient_;
    std::unique_ptr<actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction>> poseClient_;
    std::unique_ptr<actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>> fingersClient_;

    kinova_msgs::CartesianForce forceMsg_;
    kinova_msgs::PoseVelocity velMsg_;
    ros::Time commandTime_;
    std::mutex mutex_;

    ros::Subscriber poseSub_;
    ros::Subscriber wrenchSub_;
    geometry_msgs::PoseStamped::ConstPtr pose_;
    geometry_msgs::WrenchStamped::ConstPtr wrench_;

    bool torqueModeEnabled_;
    bool forceModeEnabled_;
};

} /* namespace jaco_control */


#endif /* JACO_CONTROL_ROS_INTERFACE_HPP_ */
