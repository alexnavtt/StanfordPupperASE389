#include "ase389/PupperWBC.hpp"

using std::vector;
using std::array;

void PupperWBC::updateController(const array<float, 12>& joint_angles, 
                                 const array<float, 12>& joint_velocities,
                                 const array<float, 12>& joint_torques)
{
    joint_angles_     = joint_angles;
    joint_velocities_ = joint_velocities;
    joint_torques_    = joint_torques;
}