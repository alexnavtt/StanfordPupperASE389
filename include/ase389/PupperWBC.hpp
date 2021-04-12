#ifndef PUPPER_CONTROLLER_HH_
#define PUPPER_CONTROLLER_HH_

#include <vector>
#include <array>

#include "eigen3/Eigen/Dense"

class PupperWBC{
public:
    PupperWBC();

    // Take in the current robot data
    void updateController(const std::array<float, 12>& joint_angles, 
                          const std::array<float, 12>& joint_velocities,
                          const std::array<float, 12>& joint_torques);

    // TODO:
    // Get jacobian (maybe contact Jacobian?) for the current joint angles
    Eigen::MatrixXf getJacobian();

private:
    // Joint angles in radians
    std::array<float, 12> joint_angles_;

    // Joint velocities in rad/s
    std::array<float, 12> joint_velocities_;

    // Joint torques in Nm
    std::array<float, 12> joint_torques_;

    // Robot orientation from IMU (maybe RPY?)
    Eigen::Quaternion<float> robot_orientation_;

    // 
};

#endif
