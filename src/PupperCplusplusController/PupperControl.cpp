#include <iostream>
#include "ase389/PupperWBC.hpp"
#include "ase389/PupperUrdfString.hpp"

using std::array;
using std::cout;
using std::endl;
using std::string;

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
    Task CoM_Position_Task;
    CoM_Position_Task.body_id = "bottom_PCB";
    CoM_Position_Task.type    = BODY_POS;
    CoM_Position_Task.task_weight = 0.1;
    CoM_Position_Task.active_targets = {false, false, true};    // only account for z-position
    CoM_Position_Task.pos_target << 0, 0, 0.10;
    CoM_Position_Task.Kp = 100;
    CoM_Position_Task.Kd = 0;

    // Task for Body center of mass to be flat
    Task CoM_Orientation_Task;
    CoM_Orientation_Task.body_id = "bottom_PCB";
    CoM_Orientation_Task.type    = BODY_ORI;
    CoM_Orientation_Task.task_weight = 1;
    CoM_Orientation_Task.quat_target = Eigen::Quaternion<double>::Identity();
    CoM_Orientation_Task.Kp = 500;
    CoM_Orientation_Task.Kd = 0;

    // Task for joints to avoid limits
    Task JointPositionTask;
    JointPositionTask.type = JOINT_POS;
    JointPositionTask.task_weight = 0.1;
    JointPositionTask.joint_target = VectorNd::Zero(12);
    JointPositionTask.active_targets = {true, false, false, true, false, false, true, false, false, true, false, false};
    JointPositionTask.Kp = 1000;
    JointPositionTask.Kd = 0;

    // Foot contact tasks
    Task FrontLeftContactTask;
    FrontLeftContactTask.type = BODY_POS;
    FrontLeftContactTask.task_weight = 0.1;
    FrontLeftContactTask.active_targets = {false, false, true};
    FrontLeftContactTask.pos_target << 0, 0, 0; 
    FrontLeftContactTask.Kp = 100;
    FrontLeftContactTask.Kd = 0;

    // Add the tasks with priority 0 and 1
    Pup.addTask("COM_elevation", &CoM_Position_Task);
    Pup.addTask("COM_orientation", &CoM_Orientation_Task);
    Pup.addTask("JointPos", &JointPositionTask);

    VectorNd joint_positions  =  VectorNd::Zero(12);
    joint_positions << 0, 0.7, 0.7,  0, 0.7, 0.7,  0, 0.7, 0.7,  0, 0.7, 0.7; // Init in squatted position
    VectorNd joint_velocities          = VectorNd::Zero(12);
    Eigen::Quaterniond robot_quat      = Eigen::Quaterniond::Identity();
    Eigen::Vector3d body_position      = Eigen::Vector3d::Zero();
    std::array<bool,4> feet_in_contact = {true, true, true, true}; // BL, BR, FL, FR
    body_position << 0, 0, 0.12;
    Pup.updateController(joint_positions, joint_velocities, body_position, robot_quat, feet_in_contact);

    // Testing positions
    Pup.joint_angles_.setZero();
    Pup.robot_orientation_.w() = 1;
    Pup.robot_orientation_.x() = 0;
    Pup.robot_orientation_.y() = 0;
    Pup.robot_orientation_.z() = 0;
    Pup.Pupper_.SetQuaternion(Pup.Pupper_.GetBodyId("bottom_PCB"), Pup.robot_orientation_, Pup.joint_angles_);

    for (int i = 0; i < 4; i++){
        cout << Pup.robot_orientation_[i];
    }
    cout << Pup.joint_angles_ << endl;
    
    // Make sure the model seems right
    const char* body_name = "front_left_foot";
    auto& model = Pup.Pupper_;
    auto id = model.GetBodyId(body_name);
    cout << model.GetBodyName(model.GetParentBodyId(id)) << endl;

    // Ensure that the hip really is joint 12
    printMatrix(Pup.getBodyJacobian_("front_left_hip"), "front_left_hip Jacobian");

    printMatrix(Pup.getBodyJacobian_("front_left_upper_link"), "front_left_upper_link Jacobian");

    printMatrix(Pup.getBodyJacobian_("front_left_lower_link"), "front_left_lower_link Jacobian");

    unsigned int hip_id = model.GetBodyId("front_left_hip");

    // Test at different hip angles
    auto pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Pup.joint_angles_, id,  VectorNd::Zero(3));
    cout << "Pos: \n" << pos  << endl;
    Pup.joint_angles_(12) = M_PI_4;
    pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Pup.joint_angles_, id,  VectorNd::Zero(3));
    cout << "Pos:\n" << pos << endl;
    Pup.joint_angles_(12) = -M_PI_4;
    pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Pup.joint_angles_, id,  VectorNd::Zero(3));
    cout << "Pos:  \n" << pos   << endl;
    Pup.joint_angles_(12) = M_PI;
    pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Pup.joint_angles_, id,  VectorNd::Zero(3));
    cout << "Pos: \n" << pos  << endl;

    cout << "The model has " << model.mJoints.size() << " joints" << endl;

    Matrix BaseOri = RigidBodyDynamics::CalcBodyWorldOrientation(model, Pup.joint_angles_, model.GetBodyId("bottom_PCB"));
    cout << "PCB Orientation: \n" << BaseOri << endl;

    Matrix HipOri = RigidBodyDynamics::CalcBodyWorldOrientation(model, Pup.joint_angles_, model.GetBodyId("front_left_hip"));
    cout << "Hip Orientation: \n" << HipOri << endl;

    Matrix HubOri = RigidBodyDynamics::CalcBodyWorldOrientation(model, Pup.joint_angles_, model.GetBodyId("front_left_hub"));
    cout << "Hub Orientation: \n" << HubOri << endl;

    for (int i = 0; i < 15; i++){
        RigidBodyDynamics::Math::SpatialTransform S = model.GetJointFrame(i);
        cout << i << ":\n" << S << endl;
    }
    
    // Pup.calculateOutputTorque();

    //Test height calculation
    //////////////////////////////////////////////////////////////////////////////////////
    // //Test height calculation
    // //////////////////////////////////////////////////////////////////////////////////////
 
    // // Pupper rotated with front right leg and back right leg in contact
    // // From Gazebo: height = .0965
    // // Calculated:  h0: 0.100148
    // //              h1: 0.0945443
    // // Values read from gazebo
    // // joint_positions  = {-.113567,.75649,1.60267,.05627936,-.81907876,-1.805184605,-.116226,.780257,1.58085,.0454920574,-.87633305,-1.77936025};
    // // robot_quat.x() = 0.000888; 
    // // robot_quat.y() = -0.1009519;
    // // robot_quat.z() = -0.0096908;
    // // robot_quat.w() = 0.9948437;
    // joint_positions  << 0,0,0,.2,.2,.2,0,0,0,.2,.2,.2;
    // robot_quat.x() = 0; // Rotate about y -.7 radians (pick up left feet): 
    // robot_quat.y() = -0.3428978;
    // robot_quat.z() = 0;
    // robot_quat.w() = 0.9393727;
    // // robot_quat.normalize();
    // Pup.updateController(joint_positions, joint_velocities, body_position, robot_quat, feet_in_contact);
    
    // const RigidBodyDynamics::Math::Vector3d body_contact_point_left(0.0, -.11, 0.0095);
    // const RigidBodyDynamics::Math::Vector3d body_contact_point_right(0.0, -.11, -0.0095);
    // RigidBodyDynamics::Math::Vector3d r0 = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("front_right_lower_link"), body_contact_point_right, true);
    // RigidBodyDynamics::Math::Vector3d r1 = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("back_right_lower_link"), body_contact_point_right, true);
    // // Retrieve Orientation of pupper base
    // Eigen::Matrix3d Rsb = robot_quat.toRotationMatrix();
    // cout << "Rsb: \n" << Rsb.format(f) << endl;
    // cout << "Rsb': \n" << Rsb.transpose().format(f) << endl;
 
    // cout << "r0 Front right contact point in base coord: \n" << r0.format(f) << endl;
    // cout << "r1 Back right contact point in base coord: \n" << r1.format(f) << endl;

    // Eigen::Vector3d r0_s = Rsb * r0;
    // Eigen::Vector3d r1_s = Rsb * r1;
    // cout << "r0 in world frame: \n" << r0_s.format(f) << endl;
    // cout << "r1 in world frame: \n" << r1_s.format(f) << endl;

    // cout << "h0: " << -r0_s(2) << endl;
    // cout << "h1: " << -r1_s(2) << endl;
    // return 0;

    ///////////////////////  Other Tests ///////////////////////////////////
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
}