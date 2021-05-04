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
    feet_in_contact_ = {true, true, true, true};

    QP_settings_ = std::make_unique<OSQPSettings>();
    QP_data_     = std::make_unique<OSQPData>();

    // Default to identity quaternion
    robot_orientation_.x() = 0;
    robot_orientation_.y() = 0;
    robot_orientation_.z() = 0;
    robot_orientation_.w() = 1;

    // testQPSolver();
}

// Update the controller with the current state of the robot
void PupperWBC::updateController(const array<float, ROBOT_NUM_JOINTS>& joint_angles, 
                                 const array<float, ROBOT_NUM_JOINTS>& joint_velocities,
                                 const Eigen::Quaternion<float>& body_quaternion,
                                 const array<bool, 4>& feet_in_contact){
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++){
        joint_angles_[i+6]     = joint_angles[i];
        joint_velocities_[i+6] = joint_velocities[i];
    }

    robot_orientation_.w() = body_quaternion.w();
    robot_orientation_.x() = body_quaternion.x();
    robot_orientation_.y() = body_quaternion.y();
    robot_orientation_.z() = body_quaternion.z();

    Pupper_.SetQuaternion(Pupper_.GetBodyId("bottom_PCB"), robot_orientation_, joint_angles_);

    feet_in_contact_ = feet_in_contact;
}

// Add a task to the IHWBC controller
void PupperWBC::addTask(unsigned priority, string name, Task* T){
    if (robot_tasks_.size() <= priority){
        robot_tasks_.resize(priority + 1);
    }
    robot_tasks_[priority] = T;
    task_indices_[name] = priority;
}

void PupperWBC::Load(std::string urdf_file_string){
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
}

void PupperWBC::initConstraintSets_(){
    // Perform initialization of RBDL constraints
    back_left_lower_link_id_ = Pupper_.GetBodyId("back_left_lower_link");
    back_right_lower_link_id_ = Pupper_.GetBodyId("back_right_lower_link");
    front_left_lower_link_id_ = Pupper_.GetBodyId("front_left_lower_link");
    front_right_lower_link_id_ = Pupper_.GetBodyId("front_right_lower_link");

    const Math::Vector3d body_contact_point_left(0.0, -.11, 0.0095);
    const Math::Vector3d body_contact_point_right(0.0, -.11, -0.0095);

    // Contact normal direction in global coordinates
    const Math::Vector3d world_normal(0.0, 0.0, 1.0); 
    
    pup_constraints_.AddContactConstraint(back_left_lower_link_id_, body_contact_point_left, world_normal, "back_left_contact");
    pup_constraints_.AddContactConstraint(back_right_lower_link_id_, body_contact_point_right, world_normal, "back_right_contact");
    pup_constraints_.AddContactConstraint(front_left_lower_link_id_, body_contact_point_left, world_normal, "front_left_contact");
    pup_constraints_.AddContactConstraint(front_right_lower_link_id_, body_contact_point_right, world_normal, "front_right_contact");

    pup_constraints_.Bind(Pupper_);
    cout << "Constraints Initialized..." << endl;
}


MatrixNd PupperWBC::getContactJacobian_(){
    // Retrieve contact Jacobian using CalcConstraintJacobian
    // Note: feet_in_contact_ is a boolean mask with the order: [back left, back right, front left, front right]
    
    MatrixNd J_c = Eigen::MatrixXd::Zero(4, Pupper_.qdot_size);
    CalcConstraintsJacobian(Pupper_, joint_angles_, pup_constraints_, J_c);
    cout << "Contact Jacobian is: \n" << J_c.transpose().format(f) << endl;

    // // Retrieve contact Jacobian using CalcPointJacobian
    // const Math::Vector3d body_contact_point_left(0.0, -.11, 0.0095);
    // const Math::Vector3d body_contact_point_right(0.0, -.11, -0.0095);

    // MatrixNd J_c_manual = MatrixNd::Zero(4, Pupper_.qdot_size); 
    // MatrixNd J_ci = MatrixNd::Zero(3, Pupper_.qdot_size); 
    // for (int i = 0; i < 4; i++){
    //     J_ci.setZero();
    //     if (i==0) CalcPointJacobian(Pupper_, joint_angles_, back_left_lower_link_id_, body_contact_point_left, J_ci);
    //     if (i==1) CalcPointJacobian(Pupper_, joint_angles_, back_right_lower_link_id_, body_contact_point_right, J_ci);
    //     if (i==2) CalcPointJacobian(Pupper_, joint_angles_, front_left_lower_link_id_, body_contact_point_left, J_ci);
    //     if (i==3) CalcPointJacobian(Pupper_, joint_angles_, front_right_lower_link_id_, body_contact_point_right, J_ci);
    //     // TODO: Rotate Jacobian to align with world z
    //     J_c_manual.row(i) = J_ci.row(2);
    // }
    // cout << "Contact Jacobian Manual: \n" << J_c_manual.transpose().format(f) << endl;
    return J_c;
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

void PupperWBC::formQP(){
    // Optimization problem form:
    // min .5(x'Px) + q'x
    // st l <= Ax <= u

    // x is a concatenated vector of qdotdot and Fr 
    // P is a sparse block matrix 
    // P = [P_a,  0  ; 
    //       0 , P_b];

    // TODO: Determine if we need to be careful with sparsity so that we can still update coefficients in solver. 
    //          -OSQP update requires the sparsity to remain the same. 
    //          -There may be zeros in the P matrix one solve that can be non-zero in the next solve. 
    //       
    //       Determine how to neglect the non-contacting portions of the opt. problem. 
    
    // Form task cost matrix and vector
    MatrixNd cost_t_mat = MatrixNd::Zero(NUM_JOINTS,NUM_JOINTS);
    VectorNd cost_t_vec = VectorNd::Zero(NUM_JOINTS);

    for (int i = 0; i < robot_tasks_.size(); i++ ){
        Task* T = robot_tasks_[i];

        MatrixNd j = getTaskJacobian_(i);
        //int n_t = T[i].targets.size(); // Note: task target doesn't match jacobian row size for 1st task
        int n_t = j.rows();
        VectorNd x_ddot_desired = VectorNd::Zero(n_t);

        cost_t_mat += T[i].task_weight * j.transpose() * j; // nq x nq
        cost_t_vec += T[i].task_weight * j.transpose() * x_ddot_desired; // nq x 1
    }
    
    // Form reaction force cost matrix and vector
    MatrixNd cost_rf_mat = MatrixNd::Identity(4,4);
    VectorNd cost_rf_vec = VectorNd::Zero(4);
    float lambda_rf = 1.0; // Reaction force penalty
    cost_rf_mat *= lambda_rf;
    
    // Form P matrix and q vector
    MatrixNd P = MatrixNd::Zero(NUM_JOINTS+4,NUM_JOINTS+4);
    VectorNd q = VectorNd::Zero(NUM_JOINTS+4);
    P.topLeftCorner(NUM_JOINTS,NUM_JOINTS) = cost_t_mat;
    P.bottomRightCorner(4,4) = cost_rf_mat;
    q.head(NUM_JOINTS) = cost_t_vec;

    // Form equality and inequality constraints 
    // Enforce floating base dynamics:
    MatrixNd J_c = getContactJacobian_();
    cout << "J_c = \n" << J_c.transpose().format(f) << endl ;
    // Calculate mass matrix 
    Math::MatrixNd M = MatrixNd::Zero(NUM_JOINTS,NUM_JOINTS);
    CompositeRigidBodyAlgorithm(Pupper_, joint_angles_, M, false); // update kinematics set to false for speed
    // Calculate coriolis and gravity forces (note: gravity is included by default)
    VectorNd b_g = VectorNd::Zero(NUM_JOINTS);
    NonlinearEffects(Pupper_, joint_angles_, joint_velocities_, b_g);

    MatrixNd eq_mat_0(6,NUM_JOINTS+4);
    VectorNd eq_vec_0(6);
    cout << "start debug" << endl;
    eq_mat_0.leftCols(NUM_JOINTS) = M.topRows(6);
    cout << "o0" << endl;
    cout << "-J_c.transpose().topRows(6) size " << -J_c.transpose().topRows(6).rows() << "x" << -J_c.transpose().topRows(6).cols() << endl;
    eq_mat_0.rightCols(4) = -J_c.transpose().topRows(6); // stacked horizontally [ M, -J_c']

    cout << "o1" << endl;
    eq_vec_0 << b_g.head(6); // Is this correct for vectors??
    cout << "o2" << endl;
    // Constraints currently only floating-based dynamics 
    MatrixNd A(eq_mat_0.rows(),eq_mat_0.cols());
    cout << "o3" << endl;
    A << eq_mat_0;
    cout << "o4" << endl;

    // For equality constraints, set lower and upper equal
    VectorNd l = VectorNd::Zero(6);
    VectorNd u = VectorNd::Zero(6);
    l.head(6) = eq_vec_0;
    cout << "o5" << endl;
    u.head(6) = eq_vec_0;
    cout << "o6" << endl;
    
    c_int m = A.rows();
    c_int n = A.cols();

    // Convert eigen vectors to standard vectors;
    vector<c_float> q_c;
    vector<c_float> l_c;
    vector<c_float> u_c;
    convertEigenToCfloat_(q,q_c);
    cout << "o7" << endl;
    convertEigenToCfloat_(l,l_c);
    convertEigenToCfloat_(u,u_c);
    cout << "o8" << endl;

    //Convert matrices into csc form
    vector<c_float> P_x;
    vector<c_int> P_p, P_i;
    convertEigenToCSC_(P, P_x, P_p, P_i);
    vector<c_float> A_x;
    vector<c_int> A_p, A_i;
    convertEigenToCSC_(A, A_x, A_p, A_i);

    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = QP_settings_.get();
    OSQPData      *data     = QP_data_.get();

    // Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(n, n, P_p.back(), P_x.data(), P_i.data(), P_p.data());
        data->q = q_c.data();
        data->A = csc_matrix(m, n, A_p.back(), A_x.data(), A_i.data(), A_p.data());
        data->l = l_c.data();
        data->u = u_c.data();
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
void PupperWBC::convertEigenToCfloat_(const VectorNd &q, vector<c_float> &q_c){
    // Convert Eigen vector to std::vector<c_float> for use in osqp
    q_c.clear();
    for (Eigen::Index i = 0; i < q.size(); i++){
        q_c.push_back(q(i));
    }
}