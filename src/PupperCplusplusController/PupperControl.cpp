#include <iostream>
#include "ase389/PupperWBC.hpp"

using std::array;
using std::cout;
using std::endl;

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

    RigidBodyDynamics::Math::MatrixNd G = RigidBodyDynamics::Math::MatrixNd::Zero(3, Pup.Pupper_.qdot_size);
    const RigidBodyDynamics::Math::Vector3d body_contact_point_left(0.0, -.11, 0.0095);
    RigidBodyDynamics::CalcPointJacobian(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("back_lower_left_link"), body_contact_point_left, G);
    cout << "Foot Jacobian: \n" << G.transpose() << endl;

    // Find the necessary torque to achieve this task
    array<float, 12> Tau = Pup.calculateOutputTorque();

    return 0;
}