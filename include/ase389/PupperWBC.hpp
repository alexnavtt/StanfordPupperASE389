#ifndef PUPPER_CONTROLLER_HH_
#define PUPPER_CONTROLLER_HH_

#include <vector>
#include <array>
#include <string>
#include <unordered_map>

#include "eigen3/Eigen/Dense"
#include "ase389/WBCTask.hpp"

#define NUM_JOINTS 18   // 12 motors plus 6 floating base joints

class PupperWBC{
public:
    // Constructor
    PupperWBC();

    // Take in the current robot data
    void updateController(const std::array<float, NUM_JOINTS>& joint_angles, 
                          const std::array<float, NUM_JOINTS>& joint_velocities,
                          const std::array<float, NUM_JOINTS>& joint_torques);

    // Add a task to the robot's task list
    void addTask(unsigned priority, std::string name, Task task);

    // Get the torque command fulfilling the current tasks
    std::array<float, 12> calculateOutputTorque();

private:
    // Retrieve the task Jacobian for specific task
    Eigen::MatrixXf getTaskJacobian_(std::string task_name) const;
    Eigen::MatrixXf getTaskJacobian_(unsigned priority) const;

    // Retrieve the null space for a specific task
    Eigen::MatrixXf getTaskNullSpace_(std::string task_name) const;
    Eigen::MatrixXf getTaskNullSpace_(unsigned priority) const;

    // Joint angles in radians
    std::array<float, NUM_JOINTS> joint_angles_;

    // Joint velocities in rad/s
    std::array<float, NUM_JOINTS> joint_velocities_;

    // Joint torques in Nm
    std::array<float, NUM_JOINTS> joint_torques_;

    // Robot orientation from IMU (maybe RPY?)
    Eigen::Quaternion<float> robot_orientation_;

    // Map of robot tasks organized by name
    std::vector<Task> robot_tasks_;
    std::unordered_map<std::string, unsigned> task_indices_;
};

#endif
