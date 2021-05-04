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

    // Robot dynamics matrices
    Jc_      = MatrixNd::Zero(4, NUM_JOINTS);
    massMat_ = MatrixNd::Zero(NUM_JOINTS,NUM_JOINTS);
    b_g_     = VectorNd::Zero(NUM_JOINTS);

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

    // Update the problem matrices
    CalcConstraintsJacobian(Pupper_, joint_angles_, pup_constraints_, Jc_, true);
    CompositeRigidBodyAlgorithm(Pupper_, joint_angles_, massMat_, false);
    NonlinearEffects(Pupper_, joint_angles_, joint_velocities_, b_g_);

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
    initConstraintSets_();

    // // Confirm masses have been read correctly (since verbose output of urdf reader is incorrect):
    // for(int i = 0; i < Pupper_.mBodies.size(); i++)
    //     cout << "Body " << Pupper_.GetBodyName(i) << " mass: "<< Pupper_.mBodies[i].mMass << endl;
    // // Confirm intertias have been read correctly:
    // for(int i = 0; i < Pupper_.mBodies.size(); i++)
    //     cout << "Body " << Pupper_.GetBodyName(i) << " inertia: \n"<< Pupper_.mBodies[i].mInertia.format(f) << endl;
}

std::array<float, 12> PupperWBC::calculateOutputTorque(){
    cout << "in" << endl;
    // Objective function terms
    MatrixNd P = MatrixNd::Zero(NUM_JOINTS + 4, NUM_JOINTS + 4);
    VectorNd q = VectorNd::Zero(NUM_JOINTS + 4);
    
    // Constraint terms
    MatrixNd A = MatrixNd::Zero(             6, NUM_JOINTS + 4);
    VectorNd lower_bounds = VectorNd::Zero(6);
    VectorNd upper_bounds = VectorNd::Zero(6);
    cout << "here" << endl;
    // Solve for q_ddot and reaction forces
    formQP(P, q, A, lower_bounds, upper_bounds);
    cout << "formed" << endl;
    VectorNd optimal_solution = solveQP(A.cols(), A.rows(), P, q.data(), A, lower_bounds.data(), upper_bounds.data());
    VectorNd q_ddot = optimal_solution.head(NUM_JOINTS);
    VectorNd F_r    = optimal_solution.tail(4);

    cout << "Solution: -----------------" << endl;
    for (int i = 0; i < optimal_solution.size(); i++){
        cout << optimal_solution[i] << endl;
    }
    cout << " --------------------------" << endl;

    // Solve for the command torques
    VectorNd tau = (massMat_*q_ddot + b_g_ - Jc_.transpose()*F_r).tail(ROBOT_NUM_JOINTS);

    cout << "Torques: " << endl;
    for (int i = 0; i < tau.size(); i++){
        cout << tau[i] << endl;  
    }

    array<float, 12> output;
    std::copy(tau.data(), tau.data() + tau.size(), output.data());
    return output;
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

        Jt = U * Jb.bottomRows(3);

        cout << "Position Only: " << endl;
        cout << Jt << endl;

    }else if (T->type == "body_ori"){
        MatrixNd Jb = getBodyJacobian_(T->body_id);

        // In general we care about all 4 parts of a quaternion, so 
        // there's no selection matrix for this one
        Jt = Jb.topRows(3);
        cout << "Orientation only: \n" << Jt << endl;

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
    return getTaskJacobian_(index);
}


void PupperWBC::formQP(MatrixNd &P, VectorNd &q, MatrixNd &A, VectorNd &l, VectorNd &u){
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
    


    // ---------------------------------------------------------------
    // ------------------------- OBJECTIVE ---------------------------
    // ---------------------------------------------------------------

    // Form task cost matrix and vector
    MatrixNd cost_t_mat = MatrixNd::Zero(NUM_JOINTS,NUM_JOINTS);
    VectorNd cost_t_vec = VectorNd::Zero(NUM_JOINTS);

    for (int i = 0; i < robot_tasks_.size(); i++ ){
        Task* T = robot_tasks_[i];
        
        MatrixNd j = getTaskJacobian_(i);
        VectorNd x_ddot_desired = VectorNd::Zero(j.rows());
        cost_t_mat += T->task_weight * j.transpose() * j; // nq x nq
        cost_t_vec += T->task_weight * j.transpose() * x_ddot_desired; // nq x 1
        cout << "Cost t mat: \n" << cost_t_mat.format(f) << endl;
    }

    // Add a cost to penalize high joint accelerations
    double lambda_t = 1.0;
    for (int i = 0; i < NUM_JOINTS; i++){
        cost_t_mat(i,i) += lambda_t;
    }

    // Form reaction force cost matrix and vector
    MatrixNd cost_rf_mat = MatrixNd::Identity(4,4);
    VectorNd cost_rf_vec = VectorNd::Zero(4);
    float lambda_rf = 1.0; // Reaction force penalty
    cost_rf_mat *= lambda_rf;
    
    // Form P matrix and q vector
    P.topLeftCorner(NUM_JOINTS,NUM_JOINTS) = cost_t_mat;
    P.bottomRightCorner(4,4)               = cost_rf_mat;
    q.head(NUM_JOINTS) = cost_t_vec;


    // -----------------------------------------------------------------
    // ------------------------- CONSTRAINTS ---------------------------
    // -----------------------------------------------------------------

    // Equality constraint of the form (first six rows only) 
    // A*q_ddot + b_g = Jc^T * Fr

    // Encorporate the non-linear effects in the dynamics equation
    VectorNd eq_vec_0 = -b_g_.head(6);

    // Constraints currently only floating-based dynamics 
    MatrixNd eq_mat_0(6,NUM_JOINTS+4);
    eq_mat_0 << massMat_.topRows(6), -Jc_.transpose().topRows(6);
    A = eq_mat_0;

    // For equality constraints, set lower and upper equal
    l = eq_vec_0;
    u = eq_vec_0;

    cout << "P Matrix: \n" << P.format(f) << endl << endl;
    cout << "q Matrix: \n" << q.format(f) << endl << endl;

    cout << "A Matrix: \n" << A.format(f) << endl << endl;
    cout << "l vector: \n" << l.format(f) << endl << endl;
    cout << "u vector: \n" << u.format(f) << endl << endl;
}



VectorNd PupperWBC::solveQP(int n, int m, MatrixNd &P, c_float  *q, MatrixNd &A, c_float  *lb, c_float  *ub){

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
        data->q = q;
        data->A = csc_matrix(m, n, A_p.back(), A_x.data(), A_i.data(), A_p.data());
        data->l = lb;
        data->u = ub;
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
    }

    // Return solution
    VectorNd q_ddot(NUM_JOINTS + 4);
    std::copy(work->solution->x, work->solution->x + q_ddot.size(), q_ddot.data());

    return q_ddot;
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
