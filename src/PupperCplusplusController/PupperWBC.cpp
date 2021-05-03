#include <memory>
#include <iostream>
#include "ase389/PupperWBC.hpp"
#include "rbdl/addons/urdfreader/urdfreader.h"

using std::vector;
using std::array;
using std::string;
using std::cout;
using std::endl;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

static Eigen::IOFormat f(3);

// Constructor
PupperWBC::PupperWBC(){
    joint_angles_     = VectorNd::Zero(NUM_Q);
    joint_velocities_ = VectorNd::Zero(NUM_JOINTS);
    control_torques_  = VectorNd::Zero(NUM_JOINTS);

    QP_settings_ = std::make_unique<OSQPSettings>();
    QP_data_     = std::make_unique<OSQPData>();

    // Defaul to identity quaternion
    robot_orientation_.x() = 0;
    robot_orientation_.y() = 0;
    robot_orientation_.z() = 0;
    robot_orientation_.w() = 1;

    // testQPSolver();
}

// Update the controller with the current state of the robot
void PupperWBC::updateController(const array<float, ROBOT_NUM_JOINTS>& joint_angles, 
                                 const array<float, ROBOT_NUM_JOINTS>& joint_velocities,
                                 const Eigen::Quaternion<float>& body_quaternion){
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++){
        joint_angles_[i+6]     = joint_angles[i];
        joint_velocities_[i+6] = joint_velocities[i];
    }

    robot_orientation_.w() = body_quaternion.w();
    robot_orientation_.x() = body_quaternion.x();
    robot_orientation_.y() = body_quaternion.y();
    robot_orientation_.z() = body_quaternion.z();

    Pupper_.SetQuaternion(Pupper_.GetBodyId("bottom_PCB"), robot_orientation_, joint_angles_);
}

// Add a task to the IHWBC controller
void PupperWBC::addTask(unsigned priority, string name, Task* T){
    if (robot_tasks_.size() <= priority){
        robot_tasks_.resize(priority + 1);
    }
    robot_tasks_[priority] = T;
    task_indices_[name] = priority;
}

void PupperWBC::Load(string urdf_file_string){
    // Load the model
    RigidBodyDynamics::Addons::URDFReadFromString(urdf_file_string.c_str(), &Pupper_, true, false);

    // Summarize model characteristics
    printf("Loaded model with %d DOFs\n", Pupper_.dof_count);
    printf("*\tQ Count:   \t%d\n", Pupper_.q_size);
    printf("*\tQdot Count:\t%d\n", Pupper_.qdot_size);
    printf("*\tBody Count:\t%zd\n", Pupper_.mBodies.size());
    printf("*\tJoint Count:\t%zd\n", Pupper_.mJoints.size());
    printf("*\tBody Names:\n");
    for (int i = 0; i < Pupper_.mBodies.size(); i++)
        cout << "*\t|----" << i << "\t=> " << Pupper_.GetBodyName(i) << endl;

    Pupper_.SetQuaternion(Pupper_.GetBodyId("bottom_PCB"), robot_orientation_, joint_angles_);

    // // Confirm masses have been read correctly (since verbose output of urdf reader is incorrect):
    // for(int i = 0; i < Pupper_.mBodies.size(); i++)
    //     cout << "Body " << Pupper_.GetBodyName(i) << " mass: "<< Pupper_.mBodies[i].mMass << endl;
    // // Confirm intertias have been read correctly:
    // for(int i = 0; i < Pupper_.mBodies.size(); i++)
    //     cout << "Body " << Pupper_.GetBodyName(i) << " inertia: \n"<< Pupper_.mBodies[i].mInertia.format(f) << endl;

    //Test
    // array<bool, 4> feet_in_contact = {1,1,1,1};
    // setContacts(feet_in_contact);
}

// Set rbdl model contact constraints
void PupperWBC::setContacts(const array<bool, 4> feet_in_contact){
    
    //feet_in_contact is a boolean mask with the order:
    // [back left, back right, front left, front right]
    uint back_lower_left_link_id = Pupper_.GetBodyId("back_lower_left_link");
    uint back_lower_right_link_id = Pupper_.GetBodyId("back_lower_right_link");
    uint front_lower_left_link_id = Pupper_.GetBodyId("front_lower_left_link");
    uint front_lower_right_link_id = Pupper_.GetBodyId("front_lower_right_link");

    const Math::Vector3d body_contact_point_left(0.0, -.11, 0.0095);
    const Math::Vector3d body_contact_point_right(0.0, -.11, -0.0095);
    // Contact normal direction in base coordinates
    const Math::Vector3d world_normal(0.0, 0.0, 1.0); // TODO: modify based on orientation data
    const char* constraint_name = "lower_left_contact";
    
    pup_constraints_.AddContactConstraint(back_lower_left_link_id, body_contact_point_left, world_normal, "back_left_contact");
    pup_constraints_.AddContactConstraint(back_lower_right_link_id, body_contact_point_right, world_normal, "back_right_contact");
    pup_constraints_.AddContactConstraint(front_lower_left_link_id, body_contact_point_left, world_normal, "front_left_contact");
    pup_constraints_.AddContactConstraint(front_lower_right_link_id, body_contact_point_right, world_normal, "front_right_contact");

    cout << "CONTACTS ADDED" << endl;
    pup_constraints_.Bind(Pupper_);
    cout << "CONSTRAINTS BOUND" << endl;
    // Test
    //            // X,Y,Z, Q1,Q2,Q3, BL1,BL2,BL3, BR1,BR2,BR3, FL1,FL2,FL3,  FR1,FR2,FR3, Q4
    joint_angles_ << 0,0,0,   0,0,0,1,    0, 0, 0,     0, 0, 0,    0, 0, 0,     0, 0, 0; 
    cout << "JOINT ANGLES INIT: \n" << joint_angles_.transpose().format(f) << endl;
    
    Math::Vector3d p_contact_in_base = CalcBodyToBaseCoordinates(Pupper_, joint_angles_, back_lower_left_link_id, body_contact_point_left, false);
    cout << "Contact BL3 in base coord: \n" << p_contact_in_base.transpose().format(f) << endl;
    
    uint bottom_PCB_id = Pupper_.GetBodyId("bottom_PCB"); 
    const Math::Vector3d bottom_pcb_point(0., 0., 0.);
    Math::Vector3d p_bottom_pcb_in_base = CalcBodyToBaseCoordinates(Pupper_, joint_angles_, bottom_PCB_id, bottom_pcb_point, false);
    cout << "Bottom pcb in base coord: \n" << p_bottom_pcb_in_base.transpose().format(f) << endl;

    MatrixNd J_constraints = Eigen::MatrixXd::Zero(4, Pupper_.qdot_size);
    CalcConstraintsJacobian(Pupper_, joint_angles_, pup_constraints_, J_constraints);
    cout << "JACOBIAN CALCULATED" << endl;
    cout << "Jacobian for constraints is: \n" << J_constraints.transpose().format(f) << endl;

    if (feet_in_contact[0] == true){
        // Return only rows of jacobian for feet in contact
    }

}

// Retrieve body Jacobian by ID
MatrixNd PupperWBC::getBodyJacobian_(string body_id) {
    // Create output
    MatrixNd J = MatrixNd::Zero(6, Pupper_.qdot_size);

    // Fill the Jacobian matrix
    CalcBodySpatialJacobian(Pupper_, joint_angles_, Pupper_.GetBodyId(body_id.c_str()), J, true);
    return J;
}


// Get the derivative of a task with respect to the joint angles
MatrixNd PupperWBC::getTaskJacobian_(unsigned priority){
    Task* T = robot_tasks_[priority];
    MatrixNd Jt;

    if (T->type == "body_pos"){
        MatrixNd Jb = getBodyJacobian_(T->body_id);

        // Create selection matrix to zero out rows we don't care about
        MatrixNd U  = MatrixNd::Zero(3,3);
        for (int i = 0; i < 3; i++){
            U(i, i) = T->active_targets[i];
        }

        Jt = U * Jb.topRows(3);

        cout << "Position Only: " << endl;
        cout << Jt.transpose() << endl;

    }else if (T->type == "body_ori"){
        MatrixNd Jb = getBodyJacobian_(T->body_id);

        // In general we care about all 4 parts of a quaternion, so 
        // there's no selection matrix for this one
        Jt = Jb.bottomRows(3);
        cout << "Orientation only: \n" << Jt.transpose() << endl;

    }else if(T->type == "joint_pos"){
        Jt = MatrixNd::Zero(Pupper_.qdot_size, Pupper_.qdot_size);

        // For joint position Jacobians the value is either 1 or zero
        for (int i = 0; i < T->active_targets.size(); i++){
            Jt(i, i) = T->active_targets[i];
        }

        cout << "Joint position Jacobian: \n" << Jt.format(f) << endl;
    }else{
        throw(std::runtime_error("Unrecognized WBC Task format"));
    }

    return Jt;
}

// Overloaded version of getTaskJacobian_
MatrixNd PupperWBC::getTaskJacobian_(std::string task_name){
    unsigned index = task_indices_.at(task_name);
    getTaskJacobian_(index);
}

array<float, 12> PupperWBC::calculateOutputTorque(){
    for (int i = 0; i < robot_tasks_.size(); i++){
        getTaskJacobian_(i);
    }

    return array<float,12>();
}

void PupperWBC::testQPSolver(){
    // Load problem data
    c_float P_x[3] = {4.0, 1.0, 2.0, };
    c_int P_nnz = 3;
    c_int P_i[3] = {0, 0, 1, };
    c_int P_p[3] = {0, 1, 3, };
    c_float q[2] = {1.0, 1.0, };
    // c_float A_x[4] = {1.0, 1.0, 1.0, 1.0, };
    // c_int A_nnz = 4;
    // c_int A_i[4] = {0, 1, 0, 2, };
    // c_int A_p[3] = {0, 2, 4, };
    c_float l[3] = {1.0, 0.0, 0.0, };
    c_float u[3] = {1.0, 0.7, 0.7, };
    c_int n = 2;
    c_int m = 3;

    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = QP_settings_.get();
    OSQPData      *data     = QP_data_.get();

    MatrixNd A(3,2);
    A << 1, 1, 1, 0, 0, 1;
    vector<c_float> A_x;
    vector<c_int> A_p, A_i;
    convertEigenToCSC_(A, A_x, A_p, A_i);

    // Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(n, n, P_nnz, P_x, P_i, P_p);
        data->q = q;
        data->A = csc_matrix(m, n, A_p.back(), A_x.data(), A_i.data(), A_p.data());
        data->l = l;
        data->u = u;
    }

    // Define solver settings as default
    if (settings) {
        osqp_set_default_settings(settings);
        settings->alpha = 1.0; // Change alpha parameter
    }

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);

    // Cleanup
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);

    cout << exitflag << endl;
}

void PupperWBC::convertEigenToCSC_(const MatrixNd &P, vector<c_float> &P_x, vector<c_int> &P_p, vector<c_int> &P_i){
    // Clear any existing data from the vectors
    P_x.clear();
    P_i.clear();

    P_p.clear();
    P_p.push_back(0);
    
    const int num_rows = P.rows();
    const int num_cols = P.cols();
    for (Eigen::Index c = 0; c < num_cols; c++){
        // Look through the matrix column by column
        const double* col = P.col(c).data();

        // Iterate through the column to look for non-zero elements
        for (int i = 0; i < num_rows; i++){
            if (col[i] != 0){
                // Store the value of the element in P_x and its row index in P_i
                P_x.push_back(col[i]);
                P_i.push_back(i);
            }
        }

        P_p.push_back(P_x.size());
    }
}
