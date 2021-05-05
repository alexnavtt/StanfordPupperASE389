#include <iostream>
#include "ase389/PupperWBC.hpp"
#include "ase389/PupperUrdfString.hpp"

using std::array;
using std::cout;
using std::endl;

// Set floating precision to 3 sig figs
static Eigen::IOFormat f(3);

// Note: For debug only, function is very inefficient
static void printMatrix(Eigen::MatrixXd A, std::string title = ""){
    
    // Clean up values close to zero
    for (Eigen::Index i = 0; i < A.rows(); i++){
        for (Eigen::Index j = 0; j < A.cols(); j++){
            if (abs(A(i,j)) < 1e-6) A(i,j) = 0;
        }
    }

    // Print
    if (title != "") cout << title << endl;
    cout << A.format(f) << endl;
}


int main(int argc, char** argv){
    PupperWBC Pup;
    Pup.Load(pupper_urdf_string);

    // Task for Body center of mass to be 10cm high
    static Task CoM_Position_Task;
    CoM_Position_Task.body_id = "bottom_PCB";
    CoM_Position_Task.type    = BODY_POS;
    CoM_Position_Task.task_weight = 0.1;
    CoM_Position_Task.active_targets = {false, false, true};    // only account for z-position
    CoM_Position_Task.pos_target << 0, 0, 0.10;
    CoM_Position_Task.Kp = 100;
    CoM_Position_Task.Kd = 0;

    // Task for Body center of mass to be flat
    static Task CoM_Orientation_Task;
    CoM_Orientation_Task.body_id = "bottom_PCB";
    CoM_Orientation_Task.type    = BODY_ORI;
    CoM_Orientation_Task.task_weight = 1;
    CoM_Orientation_Task.quat_target = Eigen::Quaternion<double>::Identity();
    CoM_Orientation_Task.Kp = 500;
    CoM_Orientation_Task.Kd = 0;

    // Add the tasks with priority 0 and 1
    Pup.addTask("COM_elevation", &CoM_Position_Task);
    Pup.addTask("COM_orientation", &CoM_Orientation_Task);

    // Contact Jacobian Test
    //                // X,Y,Z, Q1,Q2,Q3, BL1,BL2,BL3,    BR1,BR2,BR3,      FL1,FL2,FL3,      FR1,FR2,FR3     Q4
    // Pup.joint_angles_ << 0,0,0,   .7071068,.7071068,0,   0,0,0,           0,0,0,           0, 0,0,            0,0,0,      0; 
    // RigidBodyDynamics::Math::MatrixNd Jc = RigidBodyDynamics::Math::MatrixNd::Zero(3, Pup.Pupper_.qdot_size);
    // const RigidBodyDynamics::Math::Vector3d body_contact_point_left(0.02, 0.02, 0.02);
    // RigidBodyDynamics::CalcPointJacobian(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId(body_name), body_contact_point_left, Jc);
    // cout << "JOINT ANGLES: \n" << Pup.joint_angles_.transpose().format(f) << endl;

    // Body Jacobian Test
    // const char* body_name = "front_right_hub";
    // RigidBodyDynamics::Math::MatrixNd Jb = RigidBodyDynamics::Math::MatrixNd::Zero(6, Pup.Pupper_.qdot_size);
    // RigidBodyDynamics::CalcBodySpatialJacobian(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId(body_name), Jb, true);
    // printMatrix(Jb.rightCols(12).transpose(), "Jacobian:");

    // Find the necessary torque to achieve this task
    // array<float, 12> Tau = Pup.calculateOutputTorque();

    // Print body name and joint index
    // for(int i = 0; i < Pup.Pupper_.mBodies.size(); i++){
    //     cout << "Body " << Pup.Pupper_.GetBodyName(i) << " Joint q index: "<< Pup.Pupper_.mJoints[i].q_index << endl;
    //     cout << "                           DOF: " << Pup.Pupper_.mJoints[i].mDoFCount << endl;
    // }

    // Load constraint set
    // Pup.initConstraintSets_();
    //Pup.joint_angles_ << 0,0,0,   1,0,0,   0,0,0,           0,0,0,           0, 0,0,            0,0,0,      0; 
    // Pup.getContactJacobian_();

    VectorNd joint_positions  = VectorNd::Zero(12);
    VectorNd joint_velocities = VectorNd::Zero(12);
    Eigen::Quaterniond robot_quat = Eigen::Quaterniond::Identity();
    std::array<bool,4> feet_in_contact = {true, true, true, true};
    Eigen::Vector3d body_position;
    body_position << 0, 0, 0.12;
    Pup.updateController(joint_positions, joint_velocities, body_position, robot_quat, feet_in_contact);
    Pup.calculateOutputTorque();

    //Test body to base coordinates 
    const RigidBodyDynamics::Math::Vector3d body_contact_point_left(0.0, -.11, 0.0095);
    RigidBodyDynamics::Math::Vector3d body_point_pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("front_left_lower_link"), body_contact_point_left, false);
    cout << "Front left contact point in base coord: \n" << body_point_pos.format(f) << endl;
    return 0;
}