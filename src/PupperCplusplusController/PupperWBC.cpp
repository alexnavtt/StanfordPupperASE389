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

#define PRINT_CLEAN(M) cout << #M << ":" << endl; printClean(M);

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

    inline void printClean(Eigen::MatrixXd &M){
        for (int i = 0; i < M.rows(); i++){
            for (int j = 0; j < M.cols(); j++){
                if (abs(M(i,j)) < 1e-4) M(i,j) = 0;
            }
        }

        cout << M << endl;
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

    // Update robot height
    // Note: contacts, joint angles, and orientation must be updated before this
    robot_height_ = calcPupperHeight_();

    // Update the problem matrices
    massMat_.setZero(); // Required!
    Jc_.setZero(); // Just to be safe; not clear if this is required 
    b_g_.setZero(); // Just to be safe; not clear if this is required 
    updateContactJacobian_();
    CompositeRigidBodyAlgorithm(Pupper_, joint_angles_, massMat_, false);
    NonlinearEffects(Pupper_, joint_angles_, joint_velocities_, b_g_);

    // cout << "mass matrix: " << endl;
    // PRINT_CLEAN(massMat_);
    // cout << "b + g: \n" << b_g_.transpose().format(f) << endl;

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
    if (not task_indices_.count(name)) return;
    Task* T = getTask(name);
    assert(T->type == JOINT_POS);
    T->joint_measured = state;
}

void PupperWBC::updateBodyPosTask(std::string name, Eigen::Vector3d state){
    if (not task_indices_.count(name)) return;
    Task* T = getTask(name);
    assert(T->type == BODY_POS);
    T->pos_measured = state;
}

void PupperWBC::updateBodyOriTask(std::string name, Eigen::Quaternion<double> state){
    if (not task_indices_.count(name)) return;
    Task* T = getTask(name);
    assert(T->type == BODY_ORI);
    T->quat_measured = state;
}

Task* PupperWBC::getTask(string name){
    return robot_tasks_[task_indices_[name]];
}

VectorNd PupperWBC::getRelativeBodyLocation(std::string body_name, VectorNd offset){
    int id = Pupper_.GetBodyId(body_name.c_str());
    VectorNd pos = CalcBodyToBaseCoordinates(Pupper_, joint_angles_, id, offset, false);
    cout << "Position of " << body_name << " is: " << endl << pos << endl;
    return pos;
}

// Load the model
void PupperWBC::Load(std::string urdf_file_string){
    // Get the model information from the URDF
    RigidBodyDynamics::Addons::URDFReadFromString(urdf_file_string.c_str(), &Pupper_, true, true);

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

void PupperWBC::Load(Model& m){
    Pupper_ = m;

    Pupper_.SetQuaternion(Pupper_.GetBodyId("bottom_PCB"), robot_orientation_, joint_angles_);
    initConstraintSets_();
}


array<float, 12> PupperWBC::calculateOutputTorque(){
    // Objective function terms
    MatrixNd P = MatrixNd::Zero(NUM_JOINTS + 12, NUM_JOINTS + 12);
    VectorNd q = VectorNd::Zero(NUM_JOINTS + 12);
    
    // Constraint terms
    MatrixNd A = MatrixNd::Zero(             26, NUM_JOINTS + 12); // 6 for floating, 16 for cone, 4 for normal reaction
    VectorNd lower_bounds = VectorNd::Zero(26);
    VectorNd upper_bounds = VectorNd::Zero(26);

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
    // Currently broken because RBDL's forward dynamics is returning nonsense. 
    // Reason for test: OSQP solution for qddot does not match simulation (i.e. z acceleration positive while pupper falling)
    //                                                                     (    joint velocities not matching               )
    // The accelerations we get from RBDL forward dynamics should match what the solver gives.
    // They dont. 
    //                             
    // However, there's an error somewhere in the use of RBDL's forward dynamics. When initialziing joint angles 
    // to non-zero values (crouched position), RBDL gives incorrect joint velocities
    // 


    VectorNd Fr = optimal_solution.tail(12); // reaction forces
    // VectorNd tau_gen = VectorNd::Zero(18); // generalized torques
    // tau_gen.tail(12) = tau;
    
    // cout << "Joint angles for test: " << joint_angles_.transpose().format(f) << endl;
    // cout << "Joint velocities for test: " << joint_velocities_.transpose().format(f) << endl;
    // cout << "torques for test: " << tau_gen.transpose().format(f) << endl;

    // VectorNd QDDOT = VectorNd::Zero(18);
    // ForwardDynamicsConstraintsDirect(Pupper_,joint_angles_,joint_velocities_,tau_gen,pup_constraints_,QDDOT);
    // cout << "Qdotdot OSQP: -------------------------" << endl;
    // cout << optimal_solution.head(18).transpose().format(f) << endl;
    //cout << "Qdotdot RBDL: \n" << QDDOT.transpose().format(f) << endl;
    // cout << " --------------------------------------" << endl;
    // cout << "Reaction Forces OSQP: -----------------" << endl;
    // cout << optimal_solution.tail(12).transpose().format(f) << endl;
    //cout << "Reaction forces RBDL: \n " << pup_constraints_.force.transpose().format(f) << endl;
    //cout << " --------------------------------------" << endl;

    // //-------------------------------------------------------------------------------//
    // //-------------------------------------------------------------------------------//
    // Check constraints:
    double mu = 1; 

    VectorNd Ax = (A*optimal_solution);
    // First leg x
    cout << "Cone Constraint Test --------------------------------" << endl;
    cout << "Fr_x_1: " << abs(Fr(0)) << " <= " << mu*Fr(2) << endl;
    cout << "Fr_y_1: " << abs(Fr(1)) << " <= " << mu*Fr(2) << endl;

    cout << "----------------------------------------------------" << endl;
    //cout << "A size: " << A.rows() << "x" << A.cols() << endl;
    //cout << "x size: " << optimal_solution.rows() << endl;
    //cout << "A*x = " << Ax.transpose().format(f) << endl; 

    // //-------------------------------------------------------------------------------//
    // //-------------------------------------------------------------------------------//

    array<float, 12> output;
    std::copy(tau.data(), tau.data() + tau.size(), output.data());
    return output;
}

void PupperWBC::initConstraintSets_(){
    // Perform initialization of RBDL contact constraints
    // Contacts are defined in the global frame
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

    // Contact normal direction in global coordinates
    const Math::Vector3d world_x(1.0, 0.0, 0.0); 
    const Math::Vector3d world_y(0.0, 1.0, 0.0); 
    const Math::Vector3d world_z(0.0, 0.0, 1.0); 

    // Back left foot 
    pup_constraints_.AddContactConstraint(back_left_lower_link_id_, body_contact_point_left_, world_x, "back_left_contact_x");
    pup_constraints_.AddContactConstraint(back_left_lower_link_id_, body_contact_point_left_, world_y, "back_left_contact_y");
    pup_constraints_.AddContactConstraint(back_left_lower_link_id_, body_contact_point_left_, world_z, "back_left_contact_z");
    // Back right foot 
    pup_constraints_.AddContactConstraint(back_right_lower_link_id_, body_contact_point_right_, world_x, "back_right_contact_x");
    pup_constraints_.AddContactConstraint(back_right_lower_link_id_, body_contact_point_right_, world_y, "back_right_contact_y");
    pup_constraints_.AddContactConstraint(back_right_lower_link_id_, body_contact_point_right_, world_z, "back_right_contact_z");
    // Front left foot 
    pup_constraints_.AddContactConstraint(front_left_lower_link_id_, body_contact_point_left_, world_x, "front_left_contact_x");
    pup_constraints_.AddContactConstraint(front_left_lower_link_id_, body_contact_point_left_, world_y, "front_left_contact_y");
    pup_constraints_.AddContactConstraint(front_left_lower_link_id_, body_contact_point_left_, world_z, "front_left_contact_z");
    // Front right foot 
    pup_constraints_.AddContactConstraint(front_right_lower_link_id_, body_contact_point_right_, world_x, "front_right_contact_x");
    pup_constraints_.AddContactConstraint(front_right_lower_link_id_, body_contact_point_right_, world_y, "front_right_contact_y");
    pup_constraints_.AddContactConstraint(front_right_lower_link_id_, body_contact_point_right_, world_z, "front_right_contact_z");

    pup_constraints_.Bind(Pupper_);
}

// Retrieve body Jacobian by ID
MatrixNd PupperWBC::getBodyJacobian_(string body_id, const Eigen::Vector3d& offset) {
    // Create output
    MatrixNd J = MatrixNd::Zero(6, NUM_JOINTS);

    // Fill the Jacobian matrix
    // cout << body_id << " " << endl;
    // CalcBodySpatialJacobian(Pupper_, joint_angles_, Pupper_.GetBodyId(body_id.c_str()), J, true);
    // PRINT_CLEAN(J);
    CalcPointJacobian6D(Pupper_, joint_angles_, Pupper_.GetBodyId(body_id.c_str()), offset, J, true);
    // PRINT_CLEAN(J);
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
        MatrixNd Jb = getBodyJacobian_(T->body_id, T->offset);

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
        MatrixNd Jb = getBodyJacobian_(T->body_id, T->offset);

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

void PupperWBC::updateContactJacobian_(bool update_kinematics){
    // Gives the velocity of the feet in the global frame.
    // update_kinematics defaults to true.
    // Note: Update_kinematics must be true if an RBDL function has not been called previously and the joint angles have changed.

    CalcConstraintsJacobian(Pupper_, joint_angles_, pup_constraints_, Jc_, update_kinematics);
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
    // P is a sparse block matrix, P_b is a diagonal matrix
    // P = [P_a,  0  ;       Sizes: [ 18x18  ,(18x12)  ;
    //       0 , P_b];               (12x18) , 12x12 ]
    //
    // A is sparse block matrix
    // A = [ M  , -Jc'  ;   Sizes: [  6x18  ,  6x12]
    //       0  , A_fr ];            (20x18), 20x12]

    // TODO: Use update routines instead of FormQP every call
    //       
    //       Set weights in main function
    //       
    //       Set up tasks for foot locations. Can we know these in global coordinates? 

    // Parameters
    double lambda_t = 0.0001; // Penalizes high joint accelerations
    VectorNd rf_desired = 9.0 * VectorNd::Ones(12);// Desired reaction forces 
    double lambda_rf_z = 0; // Normal reaction force penalty (minimize impacts)
    double lambda_rf_xy = 100; // Tangential reaction force penalty (minimize slipping)
    double w_rf = 0; // Normal reaction force tracking penalty (follow desired reaction force)
    double mu = 1; // Coefficient of friction 
    double rf_z_max = 100; // Max normal reaction force

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
        // MatrixNd j_dot;
        // double delta_t = (now() - t_prev); // seconds
        // if (T->j_prev_updated == true){
        //     j_dot = (j - T->j_prev)/delta_t;
        //     // cout << "j_dot calculated" << endl;
        //     // cout << "delta_t: " << delta_t << endl;
        // }
        // else{
        //     j_dot = MatrixNd::Zero(j.rows(),j.cols());
        //     T->j_prev_updated = true;
        // }
        // T->j_prev = j;
        // MatrixNd j_dot_q_dot = j_dot * joint_velocities_;
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
        //cost_t_vec += 2 * T->task_weight * j.transpose() * (j_dot_q_dot + x_ddot_desired); // nq x 1 

        //----------------- WITHOUT j_dot_q_dot -------------
        cost_t_vec += 2 * T->task_weight * j.transpose() * x_ddot_desired; // nq x 1
        //---------------------------------------------------

        //cout << "j_dot_q_dot_" << i << ": \n" << j_dot_q_dot << endl;
    }
    // For j_dot calculation
    t_prev = now();

    // Add a cost to penalize high joint accelerations
    for (int i = 6; i < NUM_JOINTS; i++){
        cost_t_mat(i,i) += lambda_t;
    }

    // Form reaction force cost matrix and vector
    // 1. Penalize large reaction forces with form: 
    //    lambda_rf_z*||rf_z||^2
    // 2. Penalize error between desired and calculated (only for normal reaction force for now):
    //    w*||rf_d-rf||^2

    //MatrixNd cost_rf_mat = MatrixNd::Identity(12,12);
    VectorNd cost_rf_vec = VectorNd::Zero(12);
    MatrixNd cost_rf_mat = MatrixNd::Zero(12,12);
    VectorNd diag_terms = VectorNd::Zero(12); 

    // Penalize large lateral and normal reaction forces (lambda_rf), and penalize tracking error (w_rf)
    // These elements go into the diagonal of the cost matrix
    diag_terms << lambda_rf_xy + w_rf, lambda_rf_xy + w_rf, lambda_rf_z +  w_rf, 
                  lambda_rf_xy + w_rf, lambda_rf_xy + w_rf, lambda_rf_z +  w_rf,
                  lambda_rf_xy + w_rf, lambda_rf_xy + w_rf, lambda_rf_z +  w_rf,
                  lambda_rf_xy + w_rf, lambda_rf_xy + w_rf, lambda_rf_z +  w_rf;

    cost_rf_mat = diag_terms.asDiagonal();

    // Penalizes tracking error
    cost_rf_vec =  w_rf * -rf_desired; // rf_desired is 12x1 vector

    // cout << "assert statement to cause failure here" << endl;
    // assert(0);
    
    // Form P matrix and q vector
    P.topLeftCorner(NUM_JOINTS,NUM_JOINTS) = cost_t_mat;
    P.bottomRightCorner(12,12)             = cost_rf_mat;
    q.head(NUM_JOINTS) = cost_t_vec;
    q.tail(12) = cost_rf_vec;
    
    assert(P.rows() == cost_t_mat.rows() + cost_rf_mat.rows());
    assert(q.rows() == cost_t_vec.rows() + cost_rf_vec.rows());
    assert(P.topRightCorner(18,12) == MatrixNd::Zero(18,12));
    assert(P.bottomLeftCorner(12,18) == MatrixNd::Zero(12,18));

    // -----------------------------------------------------------------
    // ------------------------- CONSTRAINTS ---------------------------
    // -----------------------------------------------------------------

    // Equality constraint of the form (first six rows only) 
    // A*q_ddot + b_g = Jc^T * Fr (floating-based dynamics)

    // Incorporate the non-linear effects in the dynamics equation
    VectorNd eq_vec_0 = -b_g_.head(6);
    MatrixNd eq_mat_0(6,NUM_JOINTS+12);

    eq_mat_0 << massMat_.topRows(6), -Jc_.transpose().topRows(6);

    // Reaction force constraints:

    // For lateral reaction forces (Contact wrench cone constraint):
    // -Fr_z*mu <= Fr_x <= Fr_z*mu   which is equivalent to    -inf <= Fr_x - Fr_z*mu < 0  AND  0 <= Fr_x + Fr_z*mu <= inf
    // -Fr_z*mu <= Fr_y <= Fr_z*mu   which is equivalent to                      (same as above)

    // For normal reaction forces:
    // 0 <= Fr_z <= rf_z_max     (l = u = 0 for feet commanded to swing)

    MatrixNd ineq_mat_0(20,NUM_JOINTS+12); // Block matrix to store inequality matrix 20x30
    MatrixNd A_fr = MatrixNd::Zero(20,12); // Inequality matrix for reaction forces (3 for Fr_z, 12 for Fr_x/Fr_z)
    VectorNd ineq_vec_l0 = VectorNd::Zero(20); // Min reaction force (zero for normal reaction forces)
    VectorNd ineq_vec_u0 = VectorNd::Zero(20); // Max reaction force 

    // Matrix for reaction force constraints. // Indexing ({x,y}, constraint_j, leg_k)
    // Leg 1 
    A_fr(0,0) = 1; // x_1_1
    A_fr(0,2) = -mu;
    A_fr(1,0) = 1; // x_2_1
    A_fr(1,2) = mu;
    A_fr(2,1) = 1; // y_1_1
    A_fr(2,2) = -mu;
    A_fr(3,1) = 1; // y_2_1
    A_fr(3,2) = mu;

    // Leg 2
    A_fr(4,3) = 1;  // x_1_2
    A_fr(4,5) = -mu;
    A_fr(5,3) = 1;  // x_2_2
    A_fr(5,5) = mu;
    A_fr(6,4) = 1;  // y_1_2
    A_fr(6,5) = -mu;
    A_fr(7,4) = 1;  // y_2_2
    A_fr(7,5) = mu;

    // Leg 3
    A_fr(8,6) = 1;  // x_1_3
    A_fr(8,8) = -mu;
    A_fr(9,6) = 1;  // x_2_3
    A_fr(9,8) = mu;
    A_fr(10,7) = 1;  // y_1_3
    A_fr(10,8) = -mu;
    A_fr(11,7) = 1;  // y_2_3
    A_fr(11,8) = mu;

    // Leg 4
    A_fr(12,9) = 1;  // x_1_4
    A_fr(12,11) = -mu;
    A_fr(13,9) = 1;  // x_2_4
    A_fr(13,11) = mu;
    A_fr(14,10) = 1;  // y_1_4
    A_fr(14,11) = -mu;
    A_fr(15,10) = 1;  // y_2_4
    A_fr(15,11) = mu;
    
    // Constraints on Fr_z
    A_fr(16,2) = 1;
    A_fr(17,5) = 1;
    A_fr(18,8) = 1;
    A_fr(19,11) = 1;

    // Min for cone constraints
    ineq_vec_l0(0) = -OSQP_INFTY;
    ineq_vec_l0(1) = 0;
    ineq_vec_l0(2) = -OSQP_INFTY;
    ineq_vec_l0(3) = 0;
    ineq_vec_l0(4) = -OSQP_INFTY;
    ineq_vec_l0(5) = 0;
    ineq_vec_l0(6) = -OSQP_INFTY;
    ineq_vec_l0(7) = 0;
    ineq_vec_l0(8) = -OSQP_INFTY;
    ineq_vec_l0(9) = 0;
    ineq_vec_l0(10) = -OSQP_INFTY;
    ineq_vec_l0(11) = 0;
    ineq_vec_l0(12) = -OSQP_INFTY;
    ineq_vec_l0(13) = 0;
    ineq_vec_l0(14) = -OSQP_INFTY;
    ineq_vec_l0(15) = 0;

    // Max for cone constraints
    ineq_vec_u0(0)  = 0;
    ineq_vec_u0(1)  = OSQP_INFTY;
    ineq_vec_u0(2)  = 0;
    ineq_vec_u0(3)  = OSQP_INFTY;
    ineq_vec_u0(4)  = 0;
    ineq_vec_u0(5)  = OSQP_INFTY;
    ineq_vec_u0(6)  = 0;
    ineq_vec_u0(7)  = OSQP_INFTY;
    ineq_vec_u0(8)  = 0;
    ineq_vec_u0(9)  = OSQP_INFTY;
    ineq_vec_u0(10) = 0;
    ineq_vec_u0(11) = OSQP_INFTY;
    ineq_vec_u0(12) = 0;
    ineq_vec_u0(13) = OSQP_INFTY;
    ineq_vec_u0(14) = 0;
    ineq_vec_u0(15) = OSQP_INFTY;

    // Max normal reaction force
    ineq_vec_u0(16) = rf_z_max;
    ineq_vec_u0(17) = rf_z_max;
    ineq_vec_u0(18) = rf_z_max;
    ineq_vec_u0(19) = rf_z_max;

    // THIS IS CURRENTLY NOT CORRECT
    // force Fr = 0 in x,y and z for swinging feet
    for (int i=0; i<4; i++){
        if (!feet_in_contact_[i]){
            //ineq_vec_u0.segment(i*3,3) = VectorNd::Zero(3); 
        }
    }

    ineq_mat_0 << MatrixNd::Zero(20,NUM_JOINTS) , A_fr;

    A.topRows(6) = eq_mat_0;
    A.bottomRows(20) = ineq_mat_0;

    l.topRows(6) = eq_vec_0; // For equality constraints, set lower and upper equal
    u.topRows(6) = eq_vec_0;
    l.bottomRows(20) = ineq_vec_l0;
    u.bottomRows(20) = ineq_vec_u0;

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

    // cout << "massMat: \n" << massMat_.topRows(6).format(f) << endl;
    // cout << "Jc_'.topRows(6): \n" << (MatrixNd::Zero(4,18)-Jc_).transpose().topRows(6).format(f) << endl;
    // cout << "combined: \n" << eq_mat_0.format(f) << endl;
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

double PupperWBC::calcPupperHeight_(){
    // Calculates the height of the pupper using contacts, joint angles, and orientation 
    // Note: feet_in_contact_, joint_angles_ and orientation should be updated before this. 
    // The orientation is implicitly used in the CalcBodyToBaseCoordinates.

    Vector3d r_bl = Vector3d::Zero(3);
    Vector3d r_br = Vector3d::Zero(3);
    Vector3d r_fl = Vector3d::Zero(3);
    Vector3d r_fr = Vector3d::Zero(3);

    double num_contacts = 0;
    double sum_z = 0;
    double height; // height from floor to COM base
    if (feet_in_contact_[0]){
        r_bl = CalcBodyToBaseCoordinates(Pupper_, joint_angles_, Pupper_.GetBodyId("back_left_lower_link"), body_contact_point_left_, true);
        num_contacts += 1;
        sum_z += -r_bl(2);
    }
    if (feet_in_contact_[1]){
        r_br = CalcBodyToBaseCoordinates(Pupper_, joint_angles_, Pupper_.GetBodyId("back_right_lower_link"), body_contact_point_right_, true);
        num_contacts += 1;
        sum_z += -r_br(2);
    }
    if (feet_in_contact_[2]){
        r_fl = CalcBodyToBaseCoordinates(Pupper_, joint_angles_, Pupper_.GetBodyId("front_left_lower_link"), body_contact_point_left_, true);
        num_contacts += 1;
        sum_z += -r_fl(2);
    }
    if (feet_in_contact_[3]){
        r_fr = CalcBodyToBaseCoordinates(Pupper_, joint_angles_, Pupper_.GetBodyId("front_right_lower_link"), body_contact_point_right_, true);
        num_contacts += 1;
        sum_z += -r_fr(2);
    }

    height = sum_z/num_contacts;
    
    // Since the base frame is aligned with the prismatic floating joints (which are aligned with world frame), the z axis should be the height. 
    // However, I'm keeping the code below just in case.
    // Below, we rotate the vectors to the world frame and extract the z component. 
    // Retrieve Orientation of pupper base
    Eigen::Matrix3d Rsb = robot_orientation_.toMatrix(); // Rotation matrix from world to PCB orientation 

    Eigen::Vector3d s_bl = Rsb * r_bl;
    Eigen::Vector3d s_br = Rsb * r_br;
    Eigen::Vector3d s_fl = Rsb * r_fl;
    Eigen::Vector3d s_fr = Rsb * r_fr;

    double s_height = -(s_bl(2) + s_br(2) + s_fl(2) + s_fr(2) ) / num_contacts;
    // cout << "------------height calculation ---------------" << endl;
    // cout << "Back left contact point in base coord: \n" << r_bl.transpose().format(f) << endl;
    // cout << "Back right contact point in base coord: \n" << r_br.transpose().format(f) << endl;
    // cout << "Front left contact point in base coord: \n" << r_fl.transpose().format(f) << endl;
    // cout << "Front right contact point in base coord: \n" << r_fr.transpose().format(f) << endl;
    // cout << "robot joint angles: " << joint_angles_.format(f) << endl;
    // cout << "robot_orientation: " << endl << robot_orientation_ << endl;
    // cout << "Rsb: \n" << Rsb.format(f) << endl;
    // cout << "bl rotated: \n" << s_bl.transpose().format(f) << endl;
    // cout << "br rotated: \n" << s_br.transpose().format(f) << endl;
    // cout << "fl rotated: \n" << s_fl.transpose().format(f) << endl;
    // cout << "fr rotated: \n" << s_fr.transpose().format(f) << endl;

    // cout << "height assuming base frame is not aligned with world (this should be wrong): " << s_height << endl;
    // cout << "height (this should be right): " << height << endl;
    // cout << "---------------------------- ----------------" << endl;

    return height;
}