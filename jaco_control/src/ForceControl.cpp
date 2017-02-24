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

#include <memory>

using namespace visualization_msgs;
using namespace interactive_markers;

std::unique_ptr<moveit::planning_interface::MoveGroup> group;
const moveit::core::JointModelGroup* jointModelGroup;
std::unique_ptr<InteractiveMarkerServer> server;
geometry_msgs::Point goal;

void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback) {
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
    boxMarker.scale.x = 0.45;
    boxMarker.scale.y = 0.45;
    boxMarker.scale.z = 0.45;
    boxMarker.color.r = 0.5;
    boxMarker.color.g = 0.5;
    boxMarker.color.b = 0.5;
    boxMarker.color.a = 1.0;

    // Create a non-interactive control which contains the box
    InteractiveMarkerControl boxControl;
    boxControl.always_visible = true;
    boxControl.markers.push_back(boxMarker);
    intMarker.controls.push_back(boxControl);

    InteractiveMarkerControl moveControl;
    moveControl.name = "move_3d";
    moveControl.interaction_mode = InteractiveMarkerControl::MOVE_3D;
    intMarker.controls.push_back(moveControl);

    server->insert(intMarker, &processFeedback);
    server->applyChanges();
}

std::vector<float> calculateTorques() {


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dumb_control");
    ros::NodeHandle nh;
    //VrepInterface vrep(nh);

    // Connect to MoveIt and get robot model
    group.reset(new moveit::planning_interface::MoveGroup("arm"));
    auto robotModel = group->getRobotModel();
    jointModelGroup = robotModel->getJointModelGroup("arm");
    group->startStateMonitor();
    ros::Duration(3).sleep();
    goal = group->getCurrentPose().pose.position;

    // Create RVIZ control
    server.reset(new InteractiveMarkerServer("simple_marker"));
    createMarker(goal);

    // Start the ROS main loop
    ros::spin();
}
