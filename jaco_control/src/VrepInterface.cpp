#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include "jaco_control/VrepInterface.hpp"
#include <ros/console.h>

// VREP remote api
extern "C" {
    #include "vrep_api/extApi.h"
    #include "vrep_api/v_repConst.h"
}

VrepInterface::VrepInterface() :
        numArmJoints_(6), numFingerJoints_(3), feedbackRate_(20.f),
        posUpdateRate_(50.f), clientID_(-1), torqueMode_(false), sync_(false),
        calcTorque_(nullptr), velMode_(false), validVel_(false) {
    // V-REP joint offsets and directions don't line up with urdf
    //jointOffsets_ = {M_PI, -1.5 * M_PI, 0.5 * M_PI, M_PI, M_PI, 1.5 * M_PI};
    //jointDirs_ = {-1, 1, -1, -1, -1, -1};
    jointOffsets_ = {0, 0, 0, 0, 0, 0};
    jointDirs_ = {1, 1, 1, 1, 1, 1};

    maxTorques_ = {15.f, 15.f, 15.f, 4.f, 4.f, 4.f};
    maxVels_ = {48.f, 48.f, 48.f, 60.f, 60.f, 60.f};
    for (std::size_t i = 0; i < maxVels_.size(); i++) {
        maxVels_[i] *= M_PI/180.f;
    }
}

VrepInterface::~VrepInterface() {
    mutex_.lock();
    if (clientID_ != -1) {
        simxFinish(clientID_);
    }
    mutex_.unlock();
}

void VrepInterface::setTorqueMode(TorqueCallback calcTorque) {
    calcTorque_ = calcTorque;
    torqueMode_ = true;
    sync_ = true;
}

void VrepInterface::setVelMode(float stepTime) {
    velMode_ = true;
    velStepTime_ = stepTime;
}

void VrepInterface::initialize(ros::NodeHandle& n) {
    // Connect to V-REP via remote api
    mutex_.lock();
    ROS_INFO("Waiting for valid time. Is V-REP running?");
    ros::Time::waitForValid();
    while(clientID_ == -1 && ros::ok()) {
        clientID_ = simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
        if (clientID_ == -1) {
            ROS_ERROR_STREAM("Couldn't connect to V-REP remote api");
        }
    }
    ROS_INFO("Connected to V-REP");
    mutex_.unlock();

    // Enable synchronous mode if needed
    if (sync_) {
        mutex_.lock();
        simxSynchronous(clientID_, true);
        simxSynchronousTrigger(clientID_);
        ROS_INFO("Enabled V-REP sync mode");
        mutex_.unlock();
    }

    // Get dummy handle used for telling V-REP's IK target
    if (velMode_) {
        dummyHandle_ = getVrepHandle("Jaco_target#0");
        dummyPos_ = getVrepPosition(dummyHandle_, true);
        dummyAng_ = getVrepOrientation(dummyHandle_, true);
    }

    // Initialise jointState_ message with joint names and get V-REP handles
    std::string vrepArmPrefix = "Jaco_joint";
    std::string vrepFingerPrefix = "JacoHand_joint1_finger";
    std::string urdfArmPrefix = "j2n6s300_joint_";
    std::string urdfFingerPrefix = "j2n6s300_joint_finger_";
    bool success = initJoints(vrepArmPrefix, urdfArmPrefix, numArmJoints_,
            jointState_, jointHandles_);
    success = success && initJoints(vrepFingerPrefix, urdfFingerPrefix,
            numFingerJoints_, jointState_, jointHandles_);
    if (!success) {
        ROS_WARN_STREAM("initJoints failed, trying suffix #0");
        jointState_ = sensor_msgs::JointState();
        jointHandles_.clear();
        success = initJoints(vrepArmPrefix, urdfArmPrefix, numArmJoints_,
                jointState_, jointHandles_, 0);
        success = success && initJoints(vrepFingerPrefix, urdfFingerPrefix,
                numFingerJoints_, jointState_, jointHandles_, 0);
        if (!success) {
            ROS_ERROR_STREAM("initJoints failed.");
            return;
        }
    }

    // Initialise feedback message */
    for (int i = 0; i < numArmJoints_; i++) {
        feedback_.joint_names.push_back(jointState_.name[i]);
        feedback_.actual.positions.push_back(0);
    }

    // Disable V-REP's position control loop if in torque mode
    if (torqueMode_) {
        disableVrepControl();
    }

    // Initialise ROS subscribers & publishers
    jointPub_ = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    feedbackPub_ = n.advertise<control_msgs::FollowJointTrajectoryFeedback>(
            "feedback_states", 1);

    publishWorkerTimer_ = n.createWallTimer(ros::WallDuration(1.0 / feedbackRate_),
            &VrepInterface::publishWorker, this);
    ROS_INFO("VrepInterface initialized");

    // Start action servers
    trajAS_.reset(new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(
            n, "/j2n6s300/follow_joint_trajectory", boost::bind(&VrepInterface::trajCB, this, _1), false));
    trajAS_->start();
}

void VrepInterface::initialize() {
    node_.reset(new ros::NodeHandle());
    node_->setCallbackQueue(&callbackQueue_);
    spinner_.reset(new ros::AsyncSpinner(1, &callbackQueue_));
    initialize(*node_);
    spinner_->start();
}

void VrepInterface::publishWorker(const ros::WallTimerEvent& e) {
    int ping;
    if (sync_) {
        mutex_.lock();
        simxGetPingTime(clientID_, &ping);   // Block until step is done
        mutex_.unlock();
    }
    updateJointState();
    publishJointInfo();
    if (torqueMode_) {
        std::vector<float> torques = calcTorque_(jointState_);
        if (!torques.empty()) {
            setVrepJointTorque(torques);
        }
    }
    if (velMode_) {
        updateTargetDummy();
    }
    if (sync_) {
        mutex_.lock();
        simxSynchronousTrigger(clientID_);   // Trigger next simulation step
        mutex_.unlock();
    }
}

void VrepInterface::updateTargetDummy() {
    mutex_.lock();
    if (!validVel_) {
        mutex_.unlock();
        return;
    }
    ros::Duration d = ros::Time::now() - velTime_;
    float t = d.toSec();
    if (t > velStepTime_) {
        t = velStepTime_;
    }
    tf::Vector3 linVel;
    tf::vector3MsgToTF(vel_.linear, linVel);
    tf::Vector3 newPos = dummyPos_ + t * linVel;

    tf::Vector3 angVel;
    tf::vector3MsgToTF(vel_.angular, angVel);
    tf::Vector3 newAng = dummyAng_ + t * angVel;
    newAng[0] = normAngle(newAng[0]);
    newAng[1] = normAngle(newAng[1]);
    newAng[2] = normAngle(newAng[2]);
    mutex_.unlock();

    setVrepPosition(dummyHandle_, newPos);
    setVrepOrientation(dummyHandle_, newAng);
}

void VrepInterface::trajCB(
        const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {
    ROS_INFO("VrepInterface received trajectory");
    control_msgs::FollowJointTrajectoryResult result;
    const auto& points = goal->trajectory.points;
    ros::Time startTime = ros::Time::now();
    std::vector<double> startPos = jointState_.position;
    int i = 0;
    ros::Rate rate(posUpdateRate_);
    while (ros::ok()) {
        // Check that preempt has not been requested by the client
        if (trajAS_->isPreemptRequested()) {
            trajAS_->setPreempted();
            break;
        }

        ros::Duration fromStart = ros::Time::now() - startTime;
        while (i < points.size() - 1 && points[i + 1].time_from_start < fromStart) {
            i++;
        }

        std::vector<double> target;
        if (i == points.size() - 1) {
            // Should've reached goal by now
            bool reachedGoal = true;
            for (std::size_t j = 0; j < numArmJoints_; j++) {
                float tolerance = 0.1;
                if (goal->goal_tolerance.size() > 0) {
                    tolerance = goal->goal_tolerance[j].position;
                }
                if (std::abs(jointState_.position[j] - points[i].positions[j]) > tolerance) {
                    reachedGoal = false;
                    break;
                }
            }
            ros::Duration timeTolerance(std::max(goal->goal_time_tolerance.toSec(), 0.1));
            if (reachedGoal) {
                result.error_code = result.SUCCESSFUL;
                trajAS_->setSucceeded(result);
                break;
            } else if (fromStart > points[i].time_from_start + timeTolerance) {
                result.error_code = result.GOAL_TOLERANCE_VIOLATED;
                trajAS_->setAborted(result);
                break;
            }
            target = points[i].positions;
        } else {
            // Interpolate a point between trajectory points
            ros::Duration segmentDuration;
            std::vector<double> prev;
            if (i == 0) {
                segmentDuration = points[i].time_from_start;
                prev = startPos;
            } else {
                segmentDuration = points[i].time_from_start - points[i - 1].time_from_start;
                prev = points[i - 1].positions;
            }
            if (segmentDuration.toSec() <= 0) {
                target = points[i].positions;
            } else {
                ros::Duration d = fromStart - points[i].time_from_start;
                float alpha = d.toSec() / segmentDuration.toSec();
                target = interpolate(prev, points[i].positions, alpha);
            }
        }
        // Cast to from doubles to floats
        std::vector<float> targetf(target.begin(), target.end());

        setVrepJointPosition(targetf);
        rate.sleep();
    }
}

bool VrepInterface::initJoints(std::string inPrefix, std::string outPrefix,
        int numJoints, sensor_msgs::JointState& jointState,
        std::vector<int>& jointHandles, int suffixCode) {
    for (int i = 0; i < numJoints; i++) {
        // Get V-REP handle
        std::string inName;
        if (suffixCode == -1) {
            inName = inPrefix + std::to_string(i + 1);
        } else {
            inName = inPrefix + std::to_string(i + 1) + "#" + std::to_string(suffixCode);
        }
        int handle = getVrepHandle(inName);
        if (handle == -1) {
            return false;
        }
        jointHandles.push_back(handle);

        // Ask V-REP to send joint angles continuously from now on
        float pos;
        mutex_.lock();
        simxGetJointPosition(clientID_, handle, &pos, simx_opmode_streaming);
        mutex_.unlock();

        // Add joint's urdf name to jointState msg
        std::string outName = outPrefix + std::to_string(i + 1);
        jointState.name.push_back(outName);
        jointState.position.push_back(pos);
        jointState.velocity.push_back(0);
        jointState.effort.push_back(0);
    }
    return true;
}

void VrepInterface::updateJointState() {
    std::vector<float> pos = getVrepJointPosition();
    for (int i = 0; i < numArmJoints_; i++) {
        jointState_.position[i] = pos[i];
    }
    jointState_.header.stamp = ros::Time::now();
}

void VrepInterface::publishJointInfo() {
    jointPub_.publish(jointState_);
    feedback_.header.stamp = ros::Time::now();
    for (int i = 0; i < numArmJoints_; i++) {
        feedback_.actual.positions[i] = jointState_.position[i];
    }
    feedbackPub_.publish(feedback_);
}

int VrepInterface::getVrepHandle(std::string name) {
    mutex_.lock();
    int handle;
    int code = simxGetObjectHandle(clientID_, name.c_str(), &handle,
            simx_opmode_blocking);
    mutex_.unlock();
    if (code != simx_return_ok) {
        ROS_ERROR_STREAM("Get handle " << name << " failed, code: " << code);
        return -1;
    }
    return handle;
}

tf::Vector3 VrepInterface::getVrepPosition(int handle, int mode) {
    mutex_.lock();
    float f[3];
    switch (mode) {
    case 0:
        simxGetObjectPosition(clientID_, handle, -1, f, simx_opmode_blocking);
        break;
    case 1:
        simxGetObjectPosition(clientID_, handle, -1, f, simx_opmode_streaming);
        break;
    case 2:
        simxGetObjectPosition(clientID_, handle, -1, f, simx_opmode_buffer);
    }
    mutex_.unlock();
    return tf::Vector3(f[0], f[1], f[2]);
}

tf::Vector3 VrepInterface::getVrepOrientation(int handle, int mode) {
    mutex_.lock();
    float f[3];
    switch (mode) {
    case 0:
        simxGetObjectOrientation(clientID_, handle, -1, f, simx_opmode_blocking);
        break;
    case 1:
        simxGetObjectOrientation(clientID_, handle, -1, f, simx_opmode_streaming);
        break;
    case 2:
        simxGetObjectOrientation(clientID_, handle, -1, f, simx_opmode_buffer);
    }
    mutex_.unlock();
    return tf::Vector3(f[0], f[1], f[2]);
}

void VrepInterface::setVrepPosition(int handle, const tf::Vector3& v) {
    mutex_.lock();
    float f[3];
    f[0] = v[0];
    f[1] = v[1];
    f[2] = v[2];
    simxSetObjectPosition(clientID_, handle, -1, f, simx_opmode_oneshot);
    mutex_.unlock();
}

void VrepInterface::setVrepOrientation(int handle, const tf::Vector3& v) {
    mutex_.lock();
    float f[3];
    f[0] = v[0];
    f[1] = v[1];
    f[2] = v[2];
    simxSetObjectOrientation(clientID_, handle, -1, f, simx_opmode_oneshot);
    mutex_.unlock();
}

std::vector<float> VrepInterface::getVrepJointPosition() {
    std::vector<float> result(numArmJoints_, 0.0);
    for (int i = 0; i < numArmJoints_; i++) {
        float pos;
        mutex_.lock();
        int code = simxGetJointPosition(clientID_, jointHandles_[i], &pos,
                simx_opmode_buffer);
        mutex_.unlock();
        if (code != simx_return_ok) {
            ROS_ERROR_STREAM("getVrepJointPosition error code: " << code);
        }
        pos = jointDirs_[i] * pos + jointOffsets_[i];
        result[i] = pos;
    }
    return result;
}

void VrepInterface::setVrepJointTorque(const std::vector<float>& targets) {
    for (int i = 0; i < numArmJoints_; i++) {
        float torque = std::max(maxTorques_[i], std::abs(targets[i]));
        mutex_.lock();
        simxSetJointForce(clientID_, jointHandles_[i], torque,
                simx_opmode_oneshot);
        float velDir;
        if (targets[i] < 0) {
            velDir = -maxVels_[i] * jointDirs_[i];
        } else if (targets[i] > 0) {
            velDir = maxVels_[i] * jointDirs_[i];
        } else {
            velDir = 0;
        }
        simxSetJointTargetVelocity(clientID_, jointHandles_[i], velDir,
                simx_opmode_oneshot);
        mutex_.unlock();
    }
}

void VrepInterface::setVrepJointPosition(const std::vector<float>& targets) {
    mutex_.lock();
    for (int i = 0; i < numArmJoints_; i++) {
        // Transform to V-REP convention
        float target = jointDirs_[i] * (targets[i] - jointOffsets_[i]);
        simxSetJointTargetPosition(clientID_, jointHandles_[i], target,
                simx_opmode_oneshot);
    }
    mutex_.unlock();
}

void VrepInterface::setVrepEefVel(geometry_msgs::Twist vel) {
    dummyPos_ = getVrepPosition(dummyHandle_, false);
    dummyAng_ = getVrepOrientation(dummyHandle_, false);
    mutex_.lock();
    vel_ = vel;
    velTime_ = ros::Time::now();
    validVel_ = true;
    mutex_.unlock();
}

void VrepInterface::disableVrepControl() {
    mutex_.lock();
    for (int i = 0; i < numArmJoints_; i++) {
        simxSetObjectIntParameter(clientID_, jointHandles_[i],
                sim_jointintparam_ctrl_enabled, 0, simx_opmode_oneshot);
    }
    mutex_.unlock();
}

void VrepInterface::enableVrepControl() {
    mutex_.lock();
    for (int i = 0; i < numArmJoints_; i++) {
        simxSetObjectIntParameter(clientID_, jointHandles_[i],
                sim_jointintparam_ctrl_enabled, 1, simx_opmode_oneshot);
    }
    mutex_.unlock();
}

std::vector<double> VrepInterface::interpolate(const std::vector<double>& last,
            const std::vector<double>& current, double alpha) {
    ROS_WARN_STREAM_COND(alpha < 0, "Negative alpha in interpolate? " << alpha);
    std::vector<double> intermediate;
    for (int i = 0; i < last.size(); i++) {
        intermediate.push_back(last[i] + alpha * (current[i] - last[i]));
    }
    return intermediate;
}

double VrepInterface::normAngle(double a) {
    while (a <= -M_PI) {
        a += 2 * M_PI;
    }
    while (a > M_PI) {
        a -= 2 * M_PI;
    }
    return a;
}

// This is here for running as a standalone node
int main(int argc, char **argv) {
    ros::init(argc, argv, "vrep_interface");
    VrepInterface vrep;
    vrep.initialize();
    ros::waitForShutdown();
    return 0;
}
