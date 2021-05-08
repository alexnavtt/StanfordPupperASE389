#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

#include "ase389/PupperModel.h"
#include "ase389/PupperWBC.hpp"


// Global variables
namespace {
    VectorNd joint_positions_(12);
    VectorNd joint_velocities_(12);
    Eigen::Vector3d body_pos_;
    Eigen::Quaterniond robot_quaternion_;
}

// This is called every time we receive a message from the Python node
void pupperStateCallBack(const sensor_msgs::JointStateConstPtr &msg){
    // Record the robot data
    std::copy(msg->position.begin(), msg->position.end(), joint_positions_.data());
    std::copy(msg->velocity.begin(), msg->velocity.end(), joint_velocities_.data());
}



int main(int argc, char** argv){
    // Initialize ROS node
    ros::init(argc, argv, "pupper_control_node");
    ros::NodeHandle nh;

    // Create publisher and subscriber to communicate with pupper
    ros::Subscriber RobotStateSubscriber = nh.subscribe("pupper_state", 1, &pupperStateCallBack);
    ros::Publisher  RobotCommandPub      = nh.advertise<std_msgs::Float64MultiArray>("pupper_commands", 10, false);

    // Create the Whole Body Controller
    PupperWBC Pup;
    Pup.Load(*createPupperModel());

    // Run controller at 100 Hz
    ros::Rate rate(100);  

    // Zero the globals
    joint_positions_.setZero();
    joint_velocities_.setZero();
    body_pos_.setZero();
    robot_quaternion_.setIdentity();

    // Foot contacts
    std::array<bool, 4> contacts = {true, true, true, true};

    // Create the ROS message that we will be sending back to the Python node
    std_msgs::Float64MultiArray command_msg;
    command_msg.data.resize(12);

    // Main loop
    while(nh.ok()){
        // Look for new ROS messages
        ros::spinOnce();

        // Run the Whole Body Controller
        Pup.updateController(joint_positions_, joint_velocities_, body_pos_, robot_quaternion_, contacts);
        auto tau = Pup.calculateOutputTorque();

        // Send commands
        std::copy(tau.begin(), tau.end(), command_msg.data.data());
        RobotCommandPub.publish(command_msg);

        rate.sleep();
    }
}

