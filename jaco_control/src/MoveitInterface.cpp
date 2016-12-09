
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <memory>
#include <iostream>
#include <string>

std::unique_ptr<moveit::planning_interface::MoveGroup> group;
const moveit::core::JointModelGroup* jointModelGroup;
geometry_msgs::PoseStamped target;

ros::Publisher torquePub;
ros::Publisher markerPub;
visualization_msgs::Marker marker;

int ndof = 6;
double kp = 20;
double kv = 1;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    target = *msg;
}

void timerCallback(const ros::TimerEvent& e) {
    // Calculate jacobian for current joint state
    auto state = group->getCurrentState();
    Eigen::MatrixXd jt = state->getJacobian(jointModelGroup);
    jt.transposeInPlace();

    // Calculate force
    geometry_msgs::PoseStamped current = group->getCurrentPose();
    Eigen::VectorXd force = Eigen::VectorXd::Zero(6);
    force(0) = kp * (target.pose.position.x - current.pose.position.x);
    force(1) = kp * (target.pose.position.y - current.pose.position.y);
    force(2) = kp * (target.pose.position.z - current.pose.position.z);

    // Publish marker
    marker.header.frame_id = current.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.points.clear();
    marker.points.push_back(current.pose.position);
    geometry_msgs::Point p;
    p.x = current.pose.position.x + force(0);
    p.y = current.pose.position.y + force(1);
    p.z = current.pose.position.z + force(2);
    marker.points.push_back(p);
    markerPub.publish(marker);

    // Convert force to joint torques
    Eigen::VectorXd torque = jt * force;

    // Publish target torques
    std::vector<double> v(torque.size());
    for (std::size_t i = 0; i < torque.size(); i++) {
        v[i] = torque[i];
    }
    std_msgs::Float64MultiArray out;
    out.data = v;
    torquePub.publish(out);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "moveit_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Marker pub for visualising force
    markerPub = nh.advertise<visualization_msgs::Marker>("force_marker", 1);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.02;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    // Connect to MoveIt and get robot model
    group.reset(new moveit::planning_interface::MoveGroup("arm"));
    auto robotModel = group->getRobotModel();
    jointModelGroup = robotModel->getJointModelGroup("arm");

    // Get the initial end effector pose
    group->startStateMonitor();
    ros::Duration(3).sleep();
    target = group->getCurrentPose();

    // Subscribe to target pose commands
    ros::Subscriber poseSub = nh.subscribe("target_pose", 1, poseCallback);

    // Publisher for target torques
    torquePub = nh.advertise<std_msgs::Float64MultiArray>("target_torques", 1);

    // Start timer loop
    double pubRate = 50.0;
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / pubRate),
            timerCallback);
    ros::waitForShutdown();
    return 0;
}
