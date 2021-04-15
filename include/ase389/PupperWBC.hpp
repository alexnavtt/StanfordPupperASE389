#ifndef PUPPER_CONTROLLER_HH_
#define PUPPER_CONTROLLER_HH_

#include <vector>
#include <array>
#include <string>
#include <unordered_map>

#include "eigen3/Eigen/Dense"
#include "ase389/WBCTask.hpp"

class PupperWBC{
public:
    // Constructor
    PupperWBC();

    // Take in the current robot data
    void updateController(const std::array<float, 12>& joint_angles, 
                          const std::array<float, 12>& joint_velocities,
                          const std::array<float, 12>& joint_torques);

    // Add a task to the robot's task list
    void addTask(unsigned priority, std::string name, Task task);

    // Retrieve the task Jacobian for specific task
    Eigen::MatrixXf getTaskJacobian(std::string task_name) const;
    Eigen::MatrixXf getTaskJacobian(unsigned priority) const;

    // Retrieve the null space for a specific task
    Eigen::MatrixXf getTaskNullSpace(std::string task_name) const;
    Eigen::MatrixXf getTaskNullSpace(unsigned priority) const;

    // Get the torque command fulfilling the current tasks
    std::array<float, 12> calculateOutputTorque();

private:

    // Joint angles in radians
    std::array<float, 12> joint_angles_;

    // Joint velocities in rad/s
    std::array<float, 12> joint_velocities_;

    // Joint torques in Nm
    std::array<float, 12> joint_torques_;

    // Robot orientation from IMU (maybe RPY?)
    Eigen::Quaternion<float> robot_orientation_;

    // Map of robot tasks organized by name
    std::vector<Task> robot_tasks_;
    std::unordered_map<std::string, unsigned> task_indices_;
};

#endif
