#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

void loadTraj(const std::string& fname) {
    // Open file
    std::ifstream stream(fname);
    std::string line;
    std::getline(stream, line);
    std::vector<std::string> header = split(line, ',');

    // First col is time. Then each frame's x, y, z, qx, qy, qz, qw
    std::vector<std::string> frames = {"head", "torso", "right_shoulder",
            "right_elbow", "right_hand", "left_shoulder", "left_elbow",
            "left_hand"};
    int numFrames = (header.size() - 1) / 7;
    if (numFrames != frames.size()) {
        ROS_ERROR("Num frames mismatch");
    }

    while(std::getline(stream, line)) {
        // Get row's values as double
        std::vector<std::string> splitted = split(line, ',');
        if (splitted.size() < 2) {
            break;
        }
        std::vector<double> dsplit;
        for (const std::string& s : splitted) {
            dsplit.push_back(std::stod(s));
        }

        // Parse into transforms and broadcast
        for (int i = 0; i < numFrames; i++) {
            tf::Transform transform;
            int j = 1 + i * 7;
            tf::Vector3 o(dsplit[j], dsplit[j + 1], dsplit[j + 2]);
            tf::Quaternion q(dsplit[j + 3], dsplit[j + 4], dsplit[j + 5], dsplit[j + 6]);
            transform.setOrigin(o);
            transform.setRotation(q);
        }
    }
    stream.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "jaco_human_playback");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Get file name and origin frame param
    std::string fname;
    ros::NodeHandle private_node("~");
    private_node.param("fname", fname, std::string("output.csv"));

    ros::Publisher targetPub = node.advertise<geometry_msgs::PoseStamped>(
            "target_pose", 1);

    //loadTraj(fname);

    tf::TransformListener listener;
    ros::Rate rate(5.0);
    while (node.ok()) {
        tf::StampedTransform t;
        std::vector<double> row = {0.0};
        try {
            listener.lookupTransform("jaco_link_base", "right_hand_1", ros::Time(0), t);
            geometry_msgs::PoseStamped pose;
            const auto& o = t.getOrigin();
            pose.pose.position.x = o.x();
            pose.pose.position.y = o.y();
            pose.pose.position.z = o.z();
            const auto& q = t.getRotation();
            pose.pose.orientation.w = q.w();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "jaco_link_base";
            targetPub.publish(pose);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }
    return 0;
}

