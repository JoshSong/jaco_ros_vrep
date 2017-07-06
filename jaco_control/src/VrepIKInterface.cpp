#include "jaco_control/VrepIKInterface.hpp"

// VREP remote api
extern "C" {
    #include "vrep_api/extApi.h"
    #include "vrep_api/v_repConst.h"
}

VrepIKInterface::VrepIKInterface() :
        clientID_(-1), stepTime_(0.25) {
}

VrepIKInterface::~VrepIKInterface() {
    if (clientID_ != -1) {
        simxFinish(clientID_);
    }
}

void VrepIKInterface::initialize(ros::NodeHandle& n) {

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

    // Get dummy handle
    int code = simxGetObjectHandle(clientID_, "Jaco_target#0", &dummyHandle_,
            simx_opmode_blocking);
    if (code != simx_return_ok) {
        ROS_ERROR_STREAM("Get handle failed, code: " << code);
        return;
    }
    simxGetObjectPosition(clientID_, dummyHandle_, -1, pos_, simx_opmode_streaming);

    // Initialise ROS subscribers & publishers
    velSub_ = n.subscribe<geometry_msgs::Twist>("/vrep_vel",
            1, &VrepIKInterface::velCb, this);

    // Start timer
    timer_ = n.createWallTimer(ros::WallDuration(0.1),
            &VrepIKInterface::timerCb, this);
}

void VrepIKInterface::timerCb(const ros::WallTimerEvent& e) {
    if (!vel_) {
        return;
    }
    ros::Duration d = ros::Time::now() - commandTime_;
    float t = d.toSec();
    if (t > stepTime_) {
        t = stepTime_;
    }
    float x = pos_[0] + t * vel_->linear.x;
    float y = pos_[1] + t * vel_->linear.y;
    float z = pos_[2] + t * vel_->linear.z;
    float newPos[3] = {x, y, z};
    simxSetObjectPosition(clientID_, dummyHandle_, -1, newPos, simx_opmode_oneshot);
}

void VrepIKInterface::velCb(const geometry_msgs::Twist::ConstPtr& msg) {
    vel_ = msg;
    commandTime_ = ros::Time::now();
    simxGetObjectPosition(clientID_, dummyHandle_, -1, pos_, simx_opmode_buffer);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vrep_ik_interface");
    ros::NodeHandle n;
    VrepIKInterface vrep;
    vrep.initialize(n);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
