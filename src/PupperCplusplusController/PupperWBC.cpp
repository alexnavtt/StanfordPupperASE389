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
    Jc_      = MatrixNd::Zero(12, NUM_JOINTS);
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

    // Record the body position vector NOTE: this does not update RBDL's translation joints
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
    MatrixNd P = MatrixNd::Zero(NUM_JOINTS + 12, NUM_JOINTS + 12);
    VectorNd q = VectorNd::Zero(NUM_JOINTS + 12);
    
    // Constraint terms
    MatrixNd A = MatrixNd::Zero(             18, NUM_JOINTS + 12);
    VectorNd lower_bounds = VectorNd::Zero(18);
    VectorNd upper_bounds = VectorNd::Zero(18);

    // Solve for q_ddot and reaction forces
    formQP(P, q, A, lower_bounds, upper_bounds);
    VectorNd optimal_solution = solveQP(A.cols(), A.rows(), P, q.data(), A, lower_bounds.data(), upper_bounds.data());
    
    VectorNd q_ddot = optimal_solution.head(NUM_JOINTS);
    VectorNd F_r    = optimal_solution.tail(12);

    // cout << "Solution: -----------------" << endl;
    // for (int i = 0; i < optimal_solution.size(); i++){
    //     cout<< optimal_solution[i] << endl;
    // }
    // cout << " --------------------------" << endl;

    // Solve for the command torques
    VectorNd tau = (massMat_*q_ddot + b_g_ - Jc_.transpose()*F_r).tail(ROBOT_NUM_JOINTS);
    // cout << "Back left hip control torque: " << tau(0) << endl;
    // cout<< "Torques: " << endl;
    // for (int i = 0; i < tau.size(); i++){
    //     cout<< tau[i] << endl;  
    // }

    // Troubleshooting (confirmed correct)
    //VectorNd sol = VectorNd::Zero(6);
    // sol = (massMat_*q_ddot + b_g_ - Jc_.transpose()*F_r).head(6); 
    // cout << "SOL: \n" << sol << endl;
    // cout << "Jc': \n" << Jc_.transpose().topRows(6).format(f) << endl;
    

    //---------------------------TEST OPTIMIZATION SOLUTION--------------------------//
    //-------------------------------------------------------------------------------//
    // Reason for test: OSQP solution for qddot does not match simulation (i.e. z acceleration positive while pupper falling)
    //                                                                     (    joint velocities not matching               )
    // The accelerations we get from RBDL forward dynamics should match what the solver gives.
    // They dont. 
    //                             
    // However, there's an error somewhere in the use of RBDL's forward dynamics. When initialziing joint angles 
    // to non-zero values (crouched position), RBDL gives incorrect joint velocities
    // 

    VectorNd QDDOT = VectorNd::Zero(18);
    VectorNd tau_gen = VectorNd::Zero(18); // generalized torques
    tau_gen.tail(12) = tau;
    
    // cout << "Joint angles for test: " << joint_angles_.transpose().format(f) << endl;
    // cout << "Joint velocities for test: " << joint_velocities_.transpose().format(f) << endl;
    // cout << "torques for test: " << tau_gen.transpose().format(f) << endl;
    
    // ForwardDynamicsConstraintsDirect(Pupper_,joint_angles_,joint_velocities_,tau_gen,pup_constraints_,QDDOT);
    cout << "Qdotdot OSQP: -------------------------" << endl;
    cout << optimal_solution.head(18).transpose().format(f) << endl;
    //cout << "Qdotdot RBDL: \n" << QDDOT.transpose().format(f) << endl;
    cout << " --------------------------------------" << endl;
    cout << "Reaction Forces OSQP: -----------------" << endl;
    cout << optimal_solution.tail(12).transpose().format(f) << endl;
    //cout << "Reaction forces RBDL: \n " << pup_constraints_.force.transpose().format(f) << endl;
    //cout << " --------------------------------------" << endl;

    // Check constraints:
    cout << "lower bounds: " << lower_bounds.transpose() << endl;
    cout << "A size: " << A.rows() << "x" << A.cols() << endl;
    cout << "x size: " << optimal_solution.rows() << endl;
    VectorNd Ax = A*optimal_solution;
    cout << "A*x = " << Ax.transpose().format(f) << endl; 

    //-------------------------------------------------------------------------------//
    //-------------------------------------------------------------------------------//

    array<float, 12> output;
    std::copy(tau.data(), tau.data() + tau.size(), output.data());
    return output;
}

void PupperWBC::initConstraintSets_(){
    // Perform initialization of RBDL contact constraints
    // Order is as follows (Rows of the contact Jacobian):
    // Back left foot
    //      X,Y,Z constraints (3 rows)
    // Back right foot
    //      X,Y,Z constraints (3 rows)
    // Front left foot
    //      X,Y,Z constraints (3 rows)
    // Front right foot 
    //      X,Y,Z constraints (3 rows)
    back_left_lower_link_id_ = Pupper_.GetBodyId("back_left_lower_link");
    back_right_lower_link_id_ = Pupper_.GetBodyId("back_right_lower_link");
    front_left_lower_link_id_ = Pupper_.GetBodyId("front_left_lower_link");
    front_right_lower_link_id_ = Pupper_.GetBodyId("front_right_lower_link");

    const Math::Vector3d body_contact_point_left(0.0, -.11, 0.0095);
    const Math::Vector3d body_contact_point_right(0.0, -.11, -0.0095);

    // Contact normal direction in global coordinates
    const Math::Vector3d world_x(1.0, 0.0, 0.0); 
    const Math::Vector3d world_y(0.0, 1.0, 0.0); 
    const Math::Vector3d world_z(0.0, 0.0, 1.0); 

    // Back left foot 
    pup_constraints_.AddContactConstraint(back_left_lower_link_id_, body_contact_point_left, world_x, "back_left_contact_x");
    pup_constraints_.AddContactConstraint(back_left_lower_link_id_, body_contact_point_left, world_y, "back_left_contact_y");
    pup_constraints_.AddContactConstraint(back_left_lower_link_id_, body_contact_point_left, world_z, "back_left_contact_z");
    // Back right foot 
    pup_constraints_.AddContactConstraint(back_right_lower_link_id_, body_contact_point_right, world_x, "back_right_contact_x");
    pup_constraints_.AddContactConstraint(back_right_lower_link_id_, body_contact_point_right, world_y, "back_right_contact_y");
    pup_constraints_.AddContactConstraint(back_right_lower_link_id_, body_contact_point_right, world_z, "back_right_contact_z");
    // Front left foot 
    pup_constraints_.AddContactConstraint(front_left_lower_link_id_, body_contact_point_left, world_x, "front_left_contact_x");
    pup_constraints_.AddContactConstraint(front_left_lower_link_id_, body_contact_point_left, world_y, "front_left_contact_y");
    pup_constraints_.AddContactConstraint(front_left_lower_link_id_, body_contact_point_left, world_z, "front_left_contact_z");
    // Front right foot 
    pup_constraints_.AddContactConstraint(front_right_lower_link_id_, body_contact_point_right, world_x, "front_right_contact_x");
    pup_constraints_.AddContactConstraint(front_right_lower_link_id_, body_contact_point_right, world_y, "front_right_contact_y");
    pup_constraints_.AddContactConstraint(front_right_lower_link_id_, body_contact_point_right, world_z, "front_right_contact_z");

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
            cout << "Foot " << i << " is floating." << endl;
            Jc_.row(i*3).setZero();
            Jc_.row(i*3 + 1).setZero();
            Jc_.row(i*3 + 2).setZero();
        }
    }
    
    //cout << "Jc_': \n" << Jc_.transpose().format(f) << endl;
}

void PupperWBC::formQP(MatrixNd &P, VectorNd &q, MatrixNd &A, VectorNd &l, VectorNd &u){
    // Optimization problem form:
    // min 1/2(x'Px) + q'x
    // st l <= Ax <= u

    // x is a concatenated vector of qdotdot (18x1) and Fr (12x1) 
    // P is a sparse block matrix 
    // P = [P_a,  0  ;       Sizes: [ 18x18  ,(18x12)  ;
    //       0 , P_b];               (12x18) , 12x12 ]
    //
    // A is sparse block matrix
    // A = [ M  , -Jc'  ;   Sizes: [  6x18  ,  6x12]
    //       0  , Fr_c ];            (12x18), 12x12]

    // TODO: Use update routines instead of FormQP every call
    //       
    //       Pass rf_desired somehow. Should this be a task?
    //       
    //       Set up tasks for foot locations. Can we know these in global coordinates? 



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

        //-----------------Calculate j_dot-----------------
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
        //--------------------------------------------------

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

        //----------------- WITH j_dot_q_dot ----------------
        cost_t_vec += 2 * T->task_weight * j.transpose() * (j_dot_q_dot + x_ddot_desired); // nq x 1 

        //----------------- WITHOUT j_dot_q_dot -------------
        // cost_t_vec += 2 * T->task_weight * j.transpose() * x_ddot_desired; // nq x 1
        //---------------------------------------------------

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
    // 2. Penalize error between desired and calculated (only for normal reaction force):
    //    w*||rf_d-rf||^2

    // TODO: Pass desired value and weights
    VectorNd rf_desired(4); // Desired normal reaction force (only 4)
    rf_desired << 4 * VectorNd::Ones(4); // Desired reaction force
    double lambda_rf_z = 0.01; // Normal reaction force penalty (minimize impacts)
    double lambda_rf_xy = 0.1; // Tangential reaction force penalty (minimize slipping)
    double w_rf = 10; // Normal reaction force tracking penalty (follow desired reaction force)

    MatrixNd cost_rf_mat = MatrixNd::Identity(12,12);
    VectorNd cost_rf_vec = VectorNd::Zero(12);

    // NOTE: my creation of cost_rf_mat is fragile. The order below is important.
    // Tangential reaction forces
    cost_rf_mat *= lambda_rf_xy; // First 

    // Normal reaction forces (penalize large values and tracking error)
    cost_rf_mat(2,2) = lambda_rf_z +  w_rf; // Second
    cost_rf_mat(5,5) = lambda_rf_z +  w_rf;
    cost_rf_mat(8,8) = lambda_rf_z +  w_rf;
    cost_rf_mat(11,11) = lambda_rf_z +  w_rf;

    cost_rf_vec(2) =  w_rf * -rf_desired(0); // Since rf_desired is 4x1 vector of just the desired normal rf
    cost_rf_vec(5) =  w_rf * -rf_desired(1);
    cost_rf_vec(8) =  w_rf * -rf_desired(2);
    cost_rf_vec(11) = w_rf * -rf_desired(3);

    // Form P matrix and q vector
    P.topLeftCorner(NUM_JOINTS,NUM_JOINTS) = cost_t_mat;
    P.bottomRightCorner(12,12)               = cost_rf_mat;
    q.head(NUM_JOINTS) = cost_t_vec;
    q.tail(12) = cost_rf_vec;
    
    assert(P.rows() == cost_t_mat.rows() + cost_rf_mat.rows());
    assert(q.rows() == cost_t_vec.rows() + cost_rf_vec.rows());

    // -----------------------------------------------------------------
    // ------------------------- CONSTRAINTS ---------------------------
    // -----------------------------------------------------------------

    // Equality constraint of the form (first six rows only) 
    // A*q_ddot + b_g = Jc^T * Fr (floating-based dynamics)

    // Incorporate the non-linear effects in the dynamics equation
    VectorNd eq_vec_0 = -b_g_.head(6);
    MatrixNd eq_mat_0(6,NUM_JOINTS+12);

    eq_mat_0 << massMat_.topRows(6), -Jc_.transpose().topRows(6);

    // Debug: (confirmed correct)
    // cout << "massMat: \n" << massMat_.topRows(6).format(f) << endl;
    // cout << "Jc_'.topRows(6): \n" << (MatrixNd::Zero(4,18)-Jc_).transpose().topRows(6).format(f) << endl;
    // cout << "combined: \n" << eq_mat_0.format(f) << endl;

    // Reaction force constraints of the form:
    // l <= Fr <= u      (l = u = 0 for feet not in contact)
    
    MatrixNd ineq_mat_0(12,NUM_JOINTS+12);
    VectorNd ineq_vec_l0 = VectorNd::Zero(12);
    VectorNd ineq_vec_u0 = (double)100.0 * VectorNd::Ones(12); // Max reaction force 

    for (int i=0; i<4; i++){
        if (!feet_in_contact_[i]){
            ineq_vec_u0.segment(i*3,3) = VectorNd::Zero(3); // force Fr = 0 for floating feet
        }
    }

    ineq_mat_0 << MatrixNd::Zero(12,NUM_JOINTS) , MatrixNd::Identity(12,12);

    A.topRows(6) = eq_mat_0;
    A.bottomRows(12) = ineq_mat_0;

    l.topRows(6) = eq_vec_0; // For equality constraints, set lower and upper equal
    u.topRows(6) = eq_vec_0;
    l.bottomRows(12) = ineq_vec_l0;
    u.bottomRows(12) = ineq_vec_u0;

    // cout << "P size: " << P.rows() << "x" << P.cols() << endl;
    // cout << "A size: " << A.rows() << "x" << A.cols() << endl;

    // cout<< "P Matrix: \n" << P.format(f) << endl << endl;
    // cout<< "q Matrix: \n" << q.format(f) << endl << endl;

    // cout<< "A Matrix: \n" << A.format(f) << endl << endl;
    // cout<< "l vector: \n" << l.format(f) << endl << endl;
    // cout<< "u vector: \n" << u.format(f) << endl << endl;

    // cout<< "A matrix bottom right: \n" << A.bottomRightCorner(12,12).format(f) << endl << endl;
    // cout<< "l vector bottom 12: \n" << l.tail(12).format(f) << endl << endl;
    // cout<< "u vector bottom 12: \n" << u.tail(12).format(f) << endl << endl;
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
        settings->verbose = true;   // Prevent OSQP from printing after solving
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
    VectorNd q_ddot(NUM_JOINTS + 12);
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
