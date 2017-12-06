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

    // Start subscribers
    poseSub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/j2n6s300_driver/out/tool_pose", 1, &JacoRos::poseCb, this);
    wrenchSub_ = nh_.subscribe<geometry_msgs::WrenchStamped>("/j2n6s300_driver/out/tool_wrench", 1, &JacoRos::wrenchCb, this);

    // Start service/actionlib clients
    torqueModeClient_ = nh_.serviceClient<kinova_msgs::SetTorqueControlMode>(
            "/j2n6s300_driver/in/set_torque_control_mode");
    poseClient_.reset(new actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction>(
            nh_, "/j2n6s300_driver/pose_action/tool_pose", false));
    fingersClient_.reset(new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>(
            nh_, "/j2n6s300_driver/fingers_action/finger_positions", false));

    // Wait for jaco ROS client to start, then set to position control
    ROS_INFO_STREAM("Waiting for Jaco's ROS server");
    torqueModeClient_.waitForExistence();
    poseClient_->waitForServer();
    ros::Duration(2.0).sleep();
    setTorqueMode(false);
    ros::Duration(1.0).sleep();
    ROS_INFO_STREAM("Connected");

    /*
    // Upload gravity compensation parameters from calibration procedure
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
    */

    forcePub_ = nh_.advertise<kinova_msgs::CartesianForce>(
            "/j2n6s300_driver/in/cartesian_force", 1);
    velPub_ = nh_.advertise<kinova_msgs::PoseVelocity>(
            "/j2n6s300_driver/in/cartesian_velocity", 1);

    timer_ = nh_.createTimer(ros::Duration(0.01), &JacoRos::timerCb, this);
}

geometry_msgs::PoseStamped::ConstPtr JacoRos::getPose() const {
    return pose_;
}

geometry_msgs::WrenchStamped::ConstPtr JacoRos::getWrench() const {
    return wrench_;
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

void JacoRos::setPose(geometry_msgs::Pose pose, bool wait, float timeout) {
    mutex_.lock();
    kinova_msgs::ArmPoseGoal goal;
    goal.pose.header.frame_id = "/j2n6s300_link_base";
    goal.pose.pose = pose;
    poseClient_->sendGoal(goal);
    if (wait) {
        if (!poseClient_->waitForResult(ros::Duration(timeout))) {
            poseClient_->cancelAllGoals();
        }
    }
    mutex_.unlock();
}

void JacoRos::setFingers(float f1, float f2, float f3, bool wait, float timeout) {
    mutex_.lock();
    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = f1;
    goal.fingers.finger2 = f2;
    goal.fingers.finger3 = f3;
    fingersClient_->sendGoal(goal);
    if (wait) {
        if (!fingersClient_->waitForResult(ros::Duration(timeout))) {
            fingersClient_->cancelAllGoals();
        }
    }
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

void JacoRos::poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose_ = msg;
}

void JacoRos::wrenchCb(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    wrench_ = msg;
}

} /* namespace jaco_control */
