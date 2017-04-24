#ifndef JACO_CONTROL_ROS_INTERFACE_HPP_
#define JACO_CONTROL_ROS_INTERFACE_HPP_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <kinova_msgs/CartesianForce.h>
#include <kinova_msgs/PoseVelocity.h>
#include <mutex>

namespace jaco_control {

class JacoRos {

public:
    JacoRos();

    void initialize();
    bool setTorqueMode(bool enabled);
    bool setAdmittanceMode(bool enabled);
    void setForce(kinova_msgs::CartesianForce forceMsg);
    void setVel(kinova_msgs::PoseVelocity velMsg);

private:
    std::vector<float> loadGravParams() const;
    void timerCb(const ros::TimerEvent& e);

    ros::NodeHandle nh_;
    ros::CallbackQueue callbackQueue_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    ros::Publisher forcePub_;
    ros::Publisher velPub_;
    ros::Timer timer_;
    ros::ServiceClient torqueModeClient_;
    kinova_msgs::CartesianForce forceMsg_;
    kinova_msgs::PoseVelocity velMsg_;
    ros::Time commandTime_;
    std::mutex mutex_;

    bool torqueModeEnabled_;
    bool forceModeEnabled_;
};

} /* namespace jaco_control */


#endif /* JACO_CONTROL_ROS_INTERFACE_HPP_ */
