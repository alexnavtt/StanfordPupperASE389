#include <chrono>
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

namespace {
    Eigen::IOFormat f(3);

    VectorNd quatDiff(Eigen::Quaternion<double> q1, Eigen::Quaternion<double> q2){
        // Not 100% sure about this order
        Eigen::Quaternion<double> error = q1 * q2.conjugate();
        // Get the 3d axis difference
        VectorNd error3d = VectorNd::Zero(3);
        error3d = error.vec() * error.w()/abs(error.w());
        return error3d;
    }

    double now(){
        // returns time in seconds since last epoch
        auto now = std::chrono::system_clock::now().time_since_epoch().count();
        return now/(double)1e9; // system_clock::now is in nanoseconds
    }
}


// Constructor
PupperWBC::PupperWBC(){
    // Correctly size the robot state vectors
    joint_angles_     = VectorNd::Zero(NUM_Q);
    joint_velocities_ = VectorNd::Zero(NUM_JOINTS);
    control_torques_  = VectorNd::Zero(NUM_JOINTS);
    robot_position_   = VectorNd::Zero(3);
    feet_in_contact_ = {true, true, true, true};

    // Create solver structs
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

    // Time for numerical derivative of jacobian
    t_prev = now();
}

// Update the controller with the current state of the robot
void PupperWBC::updateController(const VectorNd& joint_angles, 
                                 const VectorNd& joint_velocities,
                                 const Eigen::Vector3d& body_position,
                                 const Eigen::Quaterniond& body_quaternion,
                                 const array<bool, 4>& feet_in_contact){
    // Copy over the joint states
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++){
        joint_angles_[i+6]     = joint_angles[i];
        joint_velocities_[i+6] = joint_velocities[i];
    }

    // Record the body position vector
    robot_position_ = body_position;

    // Record the body orientation quaternion
    robot_orientation_.w() = body_quaternion.w();
    robot_orientation_.x() = body_quaternion.x();
    robot_orientation_.y() = body_quaternion.y();
    robot_orientation_.z() = body_quaternion.z();

    Pupper_.SetQuaternion(Pupper_.GetBodyId("bottom_PCB"), robot_orientation_, joint_angles_);

    // Copy which feet are in contact
    feet_in_contact_ = feet_in_contact;

    // Update the problem matrices
    massMat_.setZero(); // Required!
    Jc_.setZero(); // Just to be safe; not clear if this is required 
    b_g_.setZero(); // Just to be safe; not clear if this is required 
    updateContactJacobian_();
    CompositeRigidBodyAlgorithm(Pupper_, joint_angles_, massMat_, false);
    NonlinearEffects(Pupper_, joint_angles_, joint_velocities_, b_g_);

}

// Add a task to the IHWBC controller
void PupperWBC::addTask(string name, Task* T){
    task_indices_[name] = robot_tasks_.size();
    robot_tasks_.push_back(T);

    switch(T->type){
        case BODY_POS:
        T->active_targets.resize(3, false);
        if (Pupper_.GetBodyId(T->body_id.c_str()) == -1){
            string message = "Task name " + T->body_id + " did not match any known body";
            throw(std::runtime_error(message));
        }
        break;

        case BODY_ORI:
        if (Pupper_.GetBodyId(T->body_id.c_str()) == -1){
            string message = "Task name " + T->body_id + " did not match any known body";
            throw(std::runtime_error(message));
        }
        T->quat_measured = Eigen::Quaterniond::Identity();
        break;

        case JOINT_POS:
        assert(T->joint_target.size() == ROBOT_NUM_JOINTS);
        T->joint_measured = VectorNd::Zero(T->joint_target.size());
        T->active_targets.resize(T->joint_target.size(), false);
    }
}

// Update the measured state of the task focus
void PupperWBC::updateJointTask(std::string name, VectorNd state){
    Task* T = getTask(name);
    assert(T->type == JOINT_POS);
    T->joint_measured = state;
}

void PupperWBC::updateBodyPosTask(std::string name, Eigen::Vector3d state){
    Task* T = getTask(name);
    assert(T->type == BODY_POS);
    T->pos_measured = state;
}

void PupperWBC::updateBodyOriTask(std::string name, Eigen::Quaternion<double> state){
    Task* T = getTask(name);
    assert(T->type == BODY_ORI);
    T->quat_measured = state;
}

Task* PupperWBC::getTask(string name){
    return robot_tasks_[task_indices_[name]];
}

// Load the model
void PupperWBC::Load(std::string urdf_file_string){
    // Get the model information from the URDF
    RigidBodyDynamics::Addons::URDFReadFromString(urdf_file_string.c_str(), &Pupper_, true, false);

    // Summarize model characteristics
    printf("Loaded model with %d DOFs\n", Pupper_.dof_count);
    printf("*\tQ Count:   \t%d\n", Pupper_.q_size);
    printf("*\tQdot Count:\t%d\n", Pupper_.qdot_size);
    printf("*\tBody Count:\t%zd\n", Pupper_.mBodies.size());
    printf("*\tJoint Count:\t%zd\n", Pupper_.mJoints.size());
    printf("*\tBody Names:\n");
    for (int i = 0; i < Pupper_.mBodies.size(); i++)
        cout<< "*\t|----" << i << "\t=> " << Pupper_.GetBodyName(i) << endl;

    // Set the robot state
    Pupper_.SetQuaternion(Pupper_.GetBodyId("bottom_PCB"), robot_orientation_, joint_angles_);
    initConstraintSets_();
}


array<float, 12> PupperWBC::calculateOutputTorque(){
    // Objective function terms
    MatrixNd P = MatrixNd::Zero(NUM_JOINTS + 4, NUM_JOINTS + 4);
    VectorNd q = VectorNd::Zero(NUM_JOINTS + 4);
    
    // Constraint terms
    MatrixNd A = MatrixNd::Zero(             10, NUM_JOINTS + 4);
    VectorNd lower_bounds = VectorNd::Zero(10);
    VectorNd upper_bounds = VectorNd::Zero(10);

    // Solve for q_ddot and reaction forces
    formQP(P, q, A, lower_bounds, upper_bounds);
    VectorNd optimal_solution = solveQP(A.cols(), A.rows(), P, q.data(), A, lower_bounds.data(), upper_bounds.data());
    VectorNd q_ddot = optimal_solution.head(NUM_JOINTS);
    VectorNd F_r    = optimal_solution.tail(4);

    cout << "Solution: -----------------" << endl;
    for (int i = 0; i < optimal_solution.size(); i++){
        cout<< optimal_solution[i] << endl;
    }
    cout << " --------------------------" << endl;

    // Solve for the command torques
    VectorNd tau = (massMat_*q_ddot + b_g_ - Jc_.transpose()*F_r).tail(ROBOT_NUM_JOINTS);
    // cout << "Back left hip control torque: " << tau(0) << endl;
    // cout<< "Torques: " << endl;
    // for (int i = 0; i < tau.size(); i++){
    //     cout<< tau[i] << endl;  
    // }

    array<float, 12> output;
    std::copy(tau.data(), tau.data() + tau.size(), output.data());
    return output;
}

void PupperWBC::initConstraintSets_(){
    // Perform initialization of RBDL contact constraints
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
}

// Retrieve body Jacobian by ID
MatrixNd PupperWBC::getBodyJacobian_(string body_id) {
    // Create output
    MatrixNd J = MatrixNd::Zero(6, NUM_JOINTS);

    // Fill the Jacobian matrix
    CalcBodySpatialJacobian(Pupper_, joint_angles_, Pupper_.GetBodyId(body_id.c_str()), J, true);
    return J;
}

// Get the derivative of a task with respect to the joint angles (Jacobian)
MatrixNd PupperWBC::getTaskJacobian_(unsigned priority){
    Task* T = robot_tasks_[priority];
    MatrixNd Jt;

    switch(T->type){
    // Body Position Task
    case BODY_POS:
    {
        MatrixNd Jb = getBodyJacobian_(T->body_id);

        // Create selection matrix to zero out rows we don't care about
        MatrixNd U  = MatrixNd::Zero(3,3);
        for (int i = 0; i < 3; i++){
            U(i, i) = T->active_targets[i];
        }

        Jt = U * Jb.bottomRows(3);
        break;
    }

    // Body Orientation Task
    case BODY_ORI:
    {
        MatrixNd Jb = getBodyJacobian_(T->body_id);

        // In general we care about all 4 parts of a quaternion, so 
        // there's no selection matrix for this one
        Jt = Jb.topRows(3);
        break;
    }

    // Joint position task
    case JOINT_POS:
        Jt = MatrixNd::Zero(ROBOT_NUM_JOINTS, NUM_JOINTS);

        // For joint position Jacobians the value is either 1 or zero
        for (int i = 0; i < T->active_targets.size(); i++){
            Jt(i, i+6) = (T->active_targets[i] ? 1 : 0);
        }
        // cout<< "Joint position Jacobian: \n" << Jt.format(f) << endl;
        break;


    default:
        throw(std::runtime_error("Unrecognized WBC Task format"));
    }

    return Jt;
}

// Overloaded version of getTaskJacobian_
MatrixNd PupperWBC::getTaskJacobian_(std::string task_name){
    unsigned index = task_indices_.at(task_name);
    return getTaskJacobian_(index);
}

void PupperWBC::updateContactJacobian_(){
    CalcConstraintsJacobian(Pupper_, joint_angles_, pup_constraints_, Jc_, true);
    for (int i=0; i<4; i++){
        if (!feet_in_contact_[i]){
            //cout << "Foot " << i << " is floating." << endl;
            Jc_.row(i).setZero();
        }
    }
    
    //cout << "Jc_': \n" << Jc_.transpose().format(f) << endl;
}

void PupperWBC::formQP(MatrixNd &P, VectorNd &q, MatrixNd &A, VectorNd &l, VectorNd &u){
    // Optimization problem form:
    // min 1/2(x'Px) + q'x
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

    // Objective is of the form 1/2(x'Px) + q'x

    // Form task cost matrix and vector
    MatrixNd cost_t_mat = MatrixNd::Zero(NUM_JOINTS,NUM_JOINTS);
    VectorNd cost_t_vec = VectorNd::Zero(NUM_JOINTS);

    for (int i = 0; i < robot_tasks_.size(); i++ ){
        Task* T = robot_tasks_[i];
        
        MatrixNd j = getTaskJacobian_(i);

        /////////////////Calculate j_dot/////////////////
        MatrixNd j_dot;
        double delta_t = (now() - t_prev); // seconds
        if (T->j_prev_updated == true){
            j_dot = (j - T->j_prev)/delta_t;
            // cout << "j_dot calculated" << endl;
            // cout << "delta_t: " << delta_t << endl;
        }
        else{
            j_dot = MatrixNd::Zero(j.rows(),j.cols());
            T->j_prev_updated = true;
        }
        T->j_prev = j;
        MatrixNd j_dot_q_dot = j_dot * joint_velocities_;
        /////////////////////////////////////////////////

        VectorNd x_ddot_desired = VectorNd::Zero(j.rows());

        switch(T->type){
            case BODY_ORI:
                x_ddot_desired = T->Kp * quatDiff(T->quat_measured, T->quat_target);
                break;

            case BODY_POS:
                x_ddot_desired = T->Kp * (T->pos_measured - T->pos_target);
                break;

            case JOINT_POS:
                x_ddot_desired = T->Kp * (T->joint_measured - T->joint_target);
                break;
        }

        // cout << "j.transpose() size: (" << j.transpose().rows() << "x" << j.transpose().cols() << ")\n";
        // cout << "x_ddot_desired size: " << x_ddot_desired.size() << endl;

        cost_t_mat += 2 * T->task_weight * j.transpose() * j; // nq x nq
        cost_t_vec += 2 * T->task_weight * j.transpose() * (j_dot_q_dot + x_ddot_desired); // nq x 1 
        // Without j_dot_q_dot:
        //cost_t_vec += 2 * T->task_weight * j.transpose() * x_ddot_desired; // nq x 1

        //cout << "j_dot_q_dot_" << i << ": \n" << j_dot_q_dot << endl;
    }
    // For j_dot calculation
    t_prev = now();

    // Add a cost to penalize high joint accelerations
    double lambda_t = 0.1;
    for (int i = 0; i < NUM_JOINTS; i++){
        cost_t_mat(i,i) += lambda_t;
    }

    // Form reaction force cost matrix and vector
    // 1. Penalize large reaction forces with form: 
    //    lambda*||rf||^2
    // 2. Penalize error between desired and calculated:
    //    w*||rf_d-rf||^2

    VectorNd rf_desired(4,1);
    rf_desired << 4.5,4.5,4.5,4.5; 
    
    MatrixNd cost_rf_mat = MatrixNd::Identity(4,4);
    VectorNd cost_rf_vec = VectorNd::Zero(4);

    double lambda_rf = 0.1; // Reaction force penalty (minimize impacts)
    double w_rf = 1; // Reaction force tracking penalty (follow desired reaction force)

    cost_rf_mat *= lambda_rf;
    cost_rf_mat += MatrixNd::Identity(4,4) * 2 * w_rf;

    cost_rf_vec += -2 * rf_desired;

    // Form P matrix and q vector
    P.topLeftCorner(NUM_JOINTS,NUM_JOINTS) = cost_t_mat;
    P.bottomRightCorner(4,4)               = cost_rf_mat;
    q.head(NUM_JOINTS) = cost_t_vec;
    q.tail(4) = cost_rf_vec;
    
    assert(P.rows() == cost_t_mat.rows() + cost_rf_mat.rows());
    assert(q.rows() == cost_t_vec.rows() + cost_rf_vec.rows());

    // -----------------------------------------------------------------
    // ------------------------- CONSTRAINTS ---------------------------
    // -----------------------------------------------------------------

    // Equality constraint of the form (first six rows only) 
    // A*q_ddot + b_g = Jc^T * Fr (floating-based dynamics)

    // Incorporate the non-linear effects in the dynamics equation
    VectorNd eq_vec_0 = -b_g_.head(6);
    MatrixNd eq_mat_0(6,NUM_JOINTS+4);
    eq_mat_0 << massMat_.topRows(6), -Jc_.transpose().topRows(6);
    
    // Reaction force constraints of the form:
    // l <= Fr <= u      (l = u = 0 for feet not in contact)
    
    MatrixNd ineq_mat_0(4,NUM_JOINTS+4);
    VectorNd ineq_vec_l0 = VectorNd::Zero(4);
    VectorNd ineq_vec_u0 = (double)100.0 * VectorNd::Ones(4);

    for (int i=0; i<4; i++){
        if (!feet_in_contact_[i]){
            ineq_vec_u0[i] = 0; // force Fr = 0 for floating feet
        }
    }

    ineq_mat_0 << MatrixNd::Zero(4,NUM_JOINTS) , MatrixNd::Identity(4,4);
    
    A.topRows(6) = eq_mat_0;
    A.bottomRows(4) = ineq_mat_0;

    l.topRows(6) = eq_vec_0; // For equality constraints, set lower and upper equal
    u.topRows(6) = eq_vec_0;
    l.bottomRows(4) = ineq_vec_l0;
    u.bottomRows(4) = ineq_vec_u0;

    // cout<< "P Matrix: \n" << P.format(f) << endl << endl;
    // cout<< "q Matrix: \n" << q.format(f) << endl << endl;

    // cout<< "A Matrix: \n" << A.format(f) << endl << endl;
    // cout<< "l vector: \n" << l.format(f) << endl << endl;
    // cout<< "u vector: \n" << u.format(f) << endl << endl;
}


VectorNd PupperWBC::solveQP(int n, int m, MatrixNd &P, c_float  *q, MatrixNd &A, c_float  *lb, c_float  *ub){

    //Convert matrices into csc form
    vector<c_float> P_x, A_x;
    vector<c_int>   P_p, P_i, A_p, A_i;
    convertEigenToCSC_(P, P_x, P_p, P_i, true);
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
        settings->alpha = 1.0;       // Change alpha parameter
        settings->verbose = false;   // Prevent OSQP from printing after solving
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


void PupperWBC::convertEigenToCSC_(const MatrixNd &P, vector<c_float> &P_x, vector<c_int> &P_p, vector<c_int> &P_i, bool triup){
    // Convert Eigen types to CSC used in OSQP solver
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

            if (triup and i == c) break;
        }

        P_p.push_back(P_x.size());
    }
}
