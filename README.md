# jaco_ros_vrep
This repo contains:  

1. A simple V-REP scenario with the Jaco arm  
2. The MoveIt! configuration for the Jaco arm. MoveIt! is used for solving kinematics.  
3. A node that exposes a ROS FollowJointTrajectoryAction interface in order to forward MoveIt! trajectory commands to VREP through VREP's C++ remote api.  

Should work on Ubuntu 16.04 + ROS Kinetic or Ubuntu 14.04 + ROS Indigo  

## Installation:   

1. Install ROS following instructions on ROS website  
2. Install V-REP and compile the RosInterface plugin following instructions on V-REP website  
    Alternatively use this script that downloads V-REP and compiles the plugin in a new catkin_ws directory (only for ROS Kinetic):  

        wget -q -O vrep_rosinterface_installer.sh https://git.io/vPwnt  
        bash vrep_rosinterface_installer.sh  
  
3. Install dependencies:  

        sudo apt-get install ros-kinetic-moveit libspatialindex-dev  
4. Compile using ROS's catkin build system:  
    E.g.  
    
        mkdir -p ~/workspace/src  
        cd ~/workspace/src  
        git clone --recursive https://github.com/JoshSong/jaco_ros_vrep.git  
        cd ..  
        catkin_make

5. Source environment variables:

        echo "source ~/workspace/devel/setup.bash" >> ~/.bashrc

## Running:
    
    roscore  
 
In another terminal, cd to VREP directory and run ./vrep.sh, load jaco.ttt, and press start button  
Then in another terminal:
 
    roslaunch jaco_control vrep_interface.launch
      
After that has connected, in another terminal run:  

    roslaunch jaco_control moveit_planning_execution.launch
 
 RVIZ should come up. You can drag the end effector marker to set the target position.  
 Then, go to the planning tab. It is recommended to reduce velocity scaling to e.g. 0.1.  
 Finally press the plan and execute button
