#include "ase389/PupperWBC.hpp"
#include "rbdl/rbdl.h"

using std::vector;
using std::array;
using std::string;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// Constructor
PupperWBC::PupperWBC(){
    // Init stuff here
}

void PupperWBC::updateController(const array<float, NUM_JOINTS>& joint_angles, 
                                 const array<float, NUM_JOINTS>& joint_velocities,
                                 const array<float, NUM_JOINTS>& joint_torques)
{
    joint_angles_     = joint_angles;
    joint_velocities_ = joint_velocities;
    joint_torques_    = joint_torques;
}

void PupperWBC::addTask(unsigned priority, string name, Task T){
    if (robot_tasks_.size() <= priority){
        robot_tasks_.resize(priority + 1);
    }
    robot_tasks_[priority] = T;
    task_indices_[name] = priority;
}
