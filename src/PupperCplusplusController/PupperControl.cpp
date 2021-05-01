#include <iostream>
#include "ase389/PupperWBC.hpp"
#include "ase389/PupperUrdfString.hpp"

using std::array;
using std::cout;
using std::endl;

static std::string urdf_relative_path = "pupper_description/pupper.urdf";

int main(int argc, char** argv){
    PupperWBC Pup;
    Pup.Load(pupper_urdf_string);

    // Task for Body center of mass to be 10cm high
    Task CoM_Position_Task;
    CoM_Position_Task.body_id = "bottom_PCB";
    CoM_Position_Task.type    = "body_pos";
    CoM_Position_Task.task_weight = 1;
    CoM_Position_Task.active_targets = {false, false, true};    // only account for z-position
    CoM_Position_Task.targets = {0.10};

    // Task for Body center of mass to be flat
    Task CoM_Orientation_Task;
    CoM_Orientation_Task.body_id = "bottom_PCB";
    CoM_Orientation_Task.type    = "body_ori";
    CoM_Orientation_Task.task_weight = 1;
    CoM_Orientation_Task.targets = {0, 0, 0, 1};    // identity quaternion 

    // Add the task with priority 0 (highest)
    // Pup.addTask(0, "COM_pos", &CoM_Position_Task);
    // Pup.addTask(1, "COM_ori", &CoM_Orientation_Task);

    //                // X,Y,Z, Q1,Q2,Q3, BL1,BL2,BL3,    BR1,BR2,BR3,      FL1,FL2,FL3,      FR1,FR2,FR3     Q4
    Pup.joint_angles_ << 0,0,0,   .7071068,.7071068,0,   0,0,0,           0,0,0,           0, 0,0,            0,0,0,      0; 
    RigidBodyDynamics::Math::MatrixNd G = RigidBodyDynamics::Math::MatrixNd::Zero(3, Pup.Pupper_.qdot_size);
    const RigidBodyDynamics::Math::Vector3d body_contact_point_left(0.02, 0.02, 0.02);
    const char* body_name = "back_right_leg_hub";
    // RigidBodyDynamics::CalcPointJacobian(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId(body_name), body_contact_point_left, G);
    
    cout << body_name << " Jacobian, id: " << Pup.Pupper_.GetBodyId(body_name) << "\n" << G.transpose() << endl;
    static Eigen::IOFormat f(3);
    cout << "JOINT ANGLES: \n" << Pup.joint_angles_.transpose().format(f) << endl;
    // Find the necessary torque to achieve this task
    // array<float, 12> Tau = Pup.calculateOutputTorque();

    // Print body name and joint index
    // for(int i = 0; i < Pup.Pupper_.mBodies.size(); i++){
    //     cout << "Body " << Pup.Pupper_.GetBodyName(i) << " Joint q index: "<< Pup.Pupper_.mJoints[i].q_index << endl;
    //     cout << "                           DOF: " << Pup.Pupper_.mJoints[i].mDoFCount << endl;
    // }
    return 0;
}