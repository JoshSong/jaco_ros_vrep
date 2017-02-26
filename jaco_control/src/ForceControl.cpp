/** @file ForceControl.cpp
 * A quick force control test. Gets position goal from RVIZ interactive marker,
 * and uses Jacobian to calculate torques.
 */

#include "jaco_control/VrepInterface.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <moveit/move_group_interface/move_group.h>
#include <sensor_msgs/JointState.h>

#include <vector>
#include <memory>

using namespace visualization_msgs;
using namespace interactive_markers;

std::unique_ptr<moveit::planning_interface::MoveGroup> group;
std::unique_ptr<moveit::core::RobotState> robotState;
const moveit::core::JointModelGroup* jointModelGroup;
std::unique_ptr<InteractiveMarkerServer> server;
geometry_msgs::Point goal;
bool moveitReady = false;

void processMarker(const InteractiveMarkerFeedbackConstPtr &feedback) {
    goal = feedback->pose.position;
}

void createMarker(const geometry_msgs::Point startPos) {
    InteractiveMarker intMarker;
    intMarker.header.frame_id = "root";
    intMarker.header.stamp = ros::Time::now();
    intMarker.name = "my_marker";
    intMarker.description = "control";
    intMarker.pose.position = startPos;

    // Create a grey box marker
    Marker boxMarker;
    boxMarker.type = Marker::CUBE;
    boxMarker.scale.x = 0.25;
    boxMarker.scale.y = 0.25;
    boxMarker.scale.z = 0.25;
    boxMarker.color.r = 0.5;
    boxMarker.color.g = 0.5;
    boxMarker.color.b = 0.5;
    boxMarker.color.a = 1.0;

    // Create a non-interactive control which contains the box
    InteractiveMarkerControl boxControl;
    boxControl.always_visible = true;
    boxControl.markers.push_back(boxMarker);
    intMarker.controls.push_back(boxControl);

    // MOVE_3D marker didn't seem to work in ROS Indigo, use MOVE_AXIS instead
    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    intMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    intMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    intMarker.controls.push_back(control);

    server->insert(intMarker, &processMarker);
    server->applyChanges();
}

std::vector<float> calcTorque(const sensor_msgs::JointState& jointState) {
    if (!moveitReady) {
        return std::vector<float>();
    }

    // Calculate jacobian for joint state
    robotState->setVariableValues(jointState);
    Eigen::MatrixXd jt = robotState->getJacobian(jointModelGroup);
    jt.transposeInPlace();

    // Forward kinematics to get eef pos
    const Eigen::Affine3d& eef = robotState->getGlobalLinkTransform(
            jointModelGroup->getLinkModels().back());

    // Calculate force
    Eigen::VectorXd force = Eigen::VectorXd::Zero(6);
    float kp = 50;
    force(0) = kp * (goal.x - eef.translation().x());
    force(1) = kp * (goal.y - eef.translation().y());
    force(2) = kp * (goal.z - eef.translation().z());

    // Convert force to joint torques
    Eigen::VectorXd torque = jt * force;
    Eigen::VectorXf torquef = torque.cast<float>();
    return std::vector<float>(torque.data(), torque.data() + torque.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dumb_control");
    ros::NodeHandle nh;

    // Start V-REP interface
    VrepInterface vrep;
    vrep.setTorqueMode(&calcTorque);
    vrep.initialize(nh);

    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Connect to MoveIt and get robot model
    group.reset(new moveit::planning_interface::MoveGroup("arm"));
    robotState.reset(new moveit::core::RobotState(group->getRobotModel()));
    auto robotModel = group->getRobotModel();
    jointModelGroup = robotModel->getJointModelGroup("arm");
    group->startStateMonitor();
    ros::Duration(3).sleep();
    goal = group->getCurrentPose().pose.position;
    moveitReady = true;

    // Create RVIZ control
    server.reset(new InteractiveMarkerServer("goal_marker"));
    createMarker(goal);

    ros::waitForShutdown();
}
