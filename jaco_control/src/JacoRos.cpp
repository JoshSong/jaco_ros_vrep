#include <kinova_msgs/SetTorqueControlMode.h>
#include <kinova_msgs/Start.h>
#include <ros/console.h>
#include <ros/package.h>

#include <fstream>
#include <assert.h>
#include <jaco_control/JacoRos.hpp>

namespace jaco_control {

JacoRos::JacoRos() {

}

void JacoRos::initialize() {
    nh_.setCallbackQueue(&callbackQueue_);
    spinner_.reset(new ros::AsyncSpinner(1, &callbackQueue_));
    spinner_->start();

    torqueModeClient_ = nh_.serviceClient<kinova_msgs::SetTorqueControlMode>(
            "/j2n6s300_driver/in/set_torque_control_mode");

    // Wait for jaco ROS client to start, then set to position control
    ROS_INFO_STREAM("Waiting for jaco");
    torqueModeClient_.waitForExistence();
    ros::Duration(2.0).sleep();
    setTorqueMode(false);
    ros::Duration(1.0).sleep();

    ROS_INFO_STREAM("Setting params");
    nh_.setParam("/j2n6s300_driver/torque_parameters/safety_factor", 0.8);
    std::vector<float> grav = loadGravParams();
    if (grav.size() == 0) {
        // If calibrated params aren't available, use defaults
        nh_.deleteParam("/j2n6s300_driver/torque_parameters/com_parameters");
    } else {
        ROS_INFO_STREAM("Using calibrated gravity parameters");
        nh_.setParam("/j2n6s300_driver/torque_parameters/use_estimated_COM_parameters", true);
        nh_.setParam("/j2n6s300_driver/torque_parameters/com_parameters", grav);
    }

    forcePub_ = nh_.advertise<kinova_msgs::CartesianForce>(
            "/j2n6s300_driver/in/cartesian_force", 1);
    velPub_ = nh_.advertise<kinova_msgs::PoseVelocity>(
            "/j2n6s300_driver/in/cartesian_velocity", 1);

    timer_ = nh_.createTimer(ros::Duration(0.01), &JacoRos::timerCb, this);
}

bool JacoRos::setTorqueMode(bool enabled) {
    mutex_.lock();
    kinova_msgs::SetTorqueControlMode srv;
    if (enabled) {
        srv.request.state = 1;
    } else {
        srv.request.state = 0;
    }
    bool result = torqueModeClient_.call(srv);
    if (result) {
        torqueModeEnabled_ = enabled;
    } else {
        ROS_WARN_STREAM("JacoRos setTorqueMode failed");
    }
    mutex_.unlock();
    return result;
}

bool JacoRos::setAdmittanceMode(bool enabled) {
    mutex_.lock();
    bool result;
    if (enabled) {
        kinova_msgs::Start start;
        result = ros::service::call("/j2n6s300_driver/in/start_force_control", start);
    } else {
        kinova_msgs::Start start;
        result = ros::service::call("/j2n6s300_driver/in/start_force_control", start);
    }
    if (result) {
        forceModeEnabled_ = enabled;
    } else {
        ROS_WARN_STREAM("JacoRos setForceMode failed");
    }
    mutex_.unlock();
    return result;
}

void JacoRos::setForce(kinova_msgs::CartesianForce forceMsg) {
    mutex_.lock();
    forceMsg_ = forceMsg;
    commandTime_ = ros::Time::now();
    mutex_.unlock();
}

void JacoRos::setVel(kinova_msgs::PoseVelocity velMsg) {
    mutex_.lock();
    velMsg_ = velMsg;
    commandTime_ = ros::Time::now();
    mutex_.unlock();
}

std::vector<float> JacoRos::loadGravParams() const {
    std::vector<float> grav;
    std::string path = ros::package::getPath("jaco_control") + "/ParametersOptimal_Z.txt";
    std::ifstream stream(path);
    if (!stream.is_open()) {
        ROS_ERROR_STREAM("Can't open " << path);
        return grav;
    }
    float f;
    while (stream >> f) {
        grav.push_back(f);
    }
    stream.close();
    assert(grav.size() == 16);
    return grav;
}

void JacoRos::timerCb(const ros::TimerEvent& e) {
    mutex_.lock();
    ros::Duration d = ros::Time::now() - commandTime_;
    if (torqueModeEnabled_) {
        // If msg is older than 0.25 sec set to zero
        if (d.toSec() > 0.250) {
            forceMsg_ = kinova_msgs::CartesianForce();
        }
        forcePub_.publish(forceMsg_);
    } else {
        // If msg is older than 0.25 sec set to zero
        if (d.toSec() > 0.250) {
            velMsg_ = kinova_msgs::PoseVelocity();
        }
        velPub_.publish(velMsg_);
    }
    mutex_.unlock();
}

} /* namespace jaco_control */
