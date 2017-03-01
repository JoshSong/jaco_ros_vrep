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
        numArmJoints_(6), numFingerJoints_(3), feedbackRate_(50.0),
        posUpdateRate_(50.0), clientID_(-1), torqueMode_(false), sync_(false),
        calcTorque_(nullptr) {
    // V-REP joint offsets and directions don't line up with urdf
    jointOffsets_ = {M_PI, -1.5 * M_PI, 0.5 * M_PI, M_PI, M_PI, 1.5 * M_PI};
    jointDirs_ = {-1, 1, -1, -1, -1, -1};

    maxTorques_ = {15.f, 15.f, 15.f, 4.f, 4.f, 4.f};
    maxVels_ = {48.f, 48.f, 48.f, 60.f, 60.f, 60.f};
    for (std::size_t i = 0; i < maxVels_.size(); i++) {
        maxVels_[i] *= M_PI/180.f;
    }

}

VrepInterface::~VrepInterface() {
    if (clientID_ != -1) {
        simxFinish(clientID_);
    }
}

void VrepInterface::setTorqueMode(TorqueCallback calcTorque) {
    calcTorque_ = calcTorque;
    torqueMode_ = true;
    sync_ = true;
}

void VrepInterface::initialize(ros::NodeHandle& n) {
    // Connect to V-REP via remote api
    ROS_INFO("Waiting for valid time. Is V-REP running?");
    ros::Time::waitForValid();
    while(clientID_ == -1 && ros::ok()) {
        clientID_ = simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
        if (clientID_ == -1) {
            ROS_ERROR_STREAM("Couldn't connect to V-REP remote api");
        }
    }
    ROS_INFO("Connected to V-REP");

    // Enable synchronous mode if needed
    if (sync_) {
        simxSynchronous(clientID_, true);
        simxSynchronousTrigger(clientID_);
        ROS_INFO("Enabled V-REP sync mode");
    }

    // Initialise jointState_ message with joint names and get V-REP handles
    std::string vrepArmPrefix = "Jaco_joint";
    std::string vrepFingerPrefix = "JacoHand_joint1_finger";
    std::string urdfArmPrefix = "jaco_joint_";
    std::string urdfFingerPrefix = "jaco_joint_finger_";
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
            n, "joint_trajectory_action", boost::bind(&VrepInterface::trajCB, this, _1), false));
    trajAS_->start();
}

void VrepInterface::publishWorker(const ros::WallTimerEvent& e) {
    int ping;
    if (sync_) {
        simxGetPingTime(clientID_, &ping);   // Block until step is done
    }
    updateJointState();
    publishJointInfo();
    if (torqueMode_) {
        std::vector<float> torques = calcTorque_(jointState_);
        if (!torques.empty()) {
            setVrepTorque(torques);
        }
    }
    if (sync_) {
        simxSynchronousTrigger(clientID_);   // Trigger next simulation step
    }
}

void VrepInterface::trajCB(
        const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {
    ROS_INFO("VrepInterface received trajectory");
    control_msgs::FollowJointTrajectoryResult result;
    const auto& points = goal->trajectory.points;
    ros::Time startTime = ros::Time::now();
    std::vector<double> startPos = jointState_.position;
    int i = 0;
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

        setVrepPosition(targetf);
        posUpdateRate_.sleep();
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
        int handle;
        int code = simxGetObjectHandle(clientID_, inName.c_str(), &handle,
                simx_opmode_blocking);
        if (code != simx_return_ok) {
            ROS_ERROR_STREAM("Get handle " << inName << " failed, code: " << code);
            return false;
        }
        jointHandles.push_back(handle);

        // Ask V-REP to send joint angles continuously from now on
        float pos;
        simxGetJointPosition(clientID_, handle, &pos, simx_opmode_streaming);

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
    std::vector<float> pos = getVrepPosition();
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

std::vector<float> VrepInterface::getVrepPosition() {
    std::vector<float> result(numArmJoints_, 0.0);
    for (int i = 0; i < numArmJoints_; i++) {
        float pos;
        int code = simxGetJointPosition(clientID_, jointHandles_[i], &pos,
                simx_opmode_buffer);
        if (code != simx_return_ok) {
            ROS_ERROR_STREAM("getVrepPosition error code: " << code);
        }
        pos = jointDirs_[i] * pos + jointOffsets_[i];
        result[i] = pos;
    }
    return result;
}

void VrepInterface::setVrepTorque(const std::vector<float>& targets) {
    for (int i = 0; i < numArmJoints_; i++) {
        float torque = std::max(maxTorques_[i], std::abs(targets[i]));
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
    }
}

void VrepInterface::setVrepPosition(const std::vector<float>& targets) {
    for (int i = 0; i < numArmJoints_; i++) {
        // Transform to V-REP convention
        float target = jointDirs_[i] * (targets[i] - jointOffsets_[i]);
        simxSetJointTargetPosition(clientID_, jointHandles_[i], target,
                simx_opmode_oneshot);
    }
}

void VrepInterface::disableVrepControl() {
    for (int i = 0; i < numArmJoints_; i++) {
        simxSetObjectIntParameter(clientID_, jointHandles_[i],
                sim_jointintparam_ctrl_enabled, 0, simx_opmode_oneshot);
    }
}

void VrepInterface::enableVrepControl() {
    for (int i = 0; i < numArmJoints_; i++) {
        simxSetObjectIntParameter(clientID_, jointHandles_[i],
                sim_jointintparam_ctrl_enabled, 1, simx_opmode_oneshot);
    }
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
int main(int argc, char **argv) {
    ros::init(argc, argv, "vrep_interface");
    ros::NodeHandle n;
    VrepInterface vrep;
    vrep.initialize(n);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
