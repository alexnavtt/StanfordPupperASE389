#include <iostream>
#include "ase389/PupperWBC.hpp"

using std::array;

static std::string urdf_relative_path = "pupper_description/pupper.urdf";

int main(int argc, char** argv){
    PupperWBC Pup;
    Pup.Load(urdf_relative_path);

    // Task for Body center of mass to be 10cm high
    Task CoM_Position_Task;
    CoM_Position_Task.body_id = "bottom_PCB";
    CoM_Position_Task.type    = "body_pos";
    CoM_Position_Task.active_targets = {false, false, true};    // only account for z-position
    CoM_Position_Task.targets = {0.10};

    // Task for Body center of mass to be flat
    Task CoM_Orientation_Task;
    CoM_Orientation_Task.body_id = "bottom_PCB";
    CoM_Orientation_Task.type    = "body_ori";
    CoM_Orientation_Task.targets = {0, 0, 0, 1};    // identity quaternion 

    // Add the task with priority 0 (highest)
    Pup.addTask(0, "COM_pos", &CoM_Position_Task);
    Pup.addTask(1, "COM_ori", &CoM_Orientation_Task);

    // Find the necessary torque to achieve this task
    array<float, 12> Tau = Pup.calculateOutputTorque();

    return 0;
}