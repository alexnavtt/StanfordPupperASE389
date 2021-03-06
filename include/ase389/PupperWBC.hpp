#ifndef PUPPER_CONTROLLER_HH_
#define PUPPER_CONTROLLER_HH_

#include <vector>
#include <array>
#include <string>
#include <cstring>
#include <unordered_map>

#include "osqp/osqp.h"
#include "rbdl/rbdl.h"
#include "eigen3/Eigen/Dense"
#include "ase389/WBCTask.hpp"

#define ROBOT_NUM_JOINTS 12
#define NUM_JOINTS 18   // 12 motors plus 6 floating base joints
#define NUM_Q 19        // 12 motors plus 3 xyz plus 4 quaternion, refer to: https://rbdl.github.io/df/dbe/joint_description.html

// To make the code a little more readable
typedef RigidBodyDynamics::Math::MatrixNd Matrix;
typedef RigidBodyDynamics::Math::VectorNd VectorNd;
typedef RigidBodyDynamics::Math::Quaternion Quat;

class PupperWBC{
public:
    // Constructor
    PupperWBC();

    // Take in the current robot data
    void updateController(const VectorNd& joint_angles, 
                          const VectorNd& joint_velocities,
                          const Eigen::Vector3d& body_position,
                          const Eigen::Quaterniond& body_quaternion,
                          const std::array<bool, 4>& feet_in_contact);

    // Add a task to the robot's task list
    void addTask(std::string name, Task* task);
    Task* getTask(std::string name);

    // Update a task with its current measurement
    void updateJointTask(  std::string name, VectorNd state);
    void updateBodyPosTask(std::string name, Eigen::Vector3d state);
    void updateBodyOriTask(std::string name, Eigen::Quaternion<double> state);

    // Get the position of a point on a body relative to the root body
    VectorNd getRelativeBodyLocation(std::string body_name, VectorNd offset = VectorNd::Zero(3));

    // Getter for the robot joint angles
    VectorNd getJointPositions();

    // Calculate height from contacts
    double calcPupperHeight();

    // Load the Pupper model from a URDF
    void Load(std::string filename);
    void Load(RigidBodyDynamics::Model& model);

    // Get the torque command fulfilling the current tasks
    std::array<float, 12> calculateOutputTorque();

private:
    // The Pupper model for RBDL
    RigidBodyDynamics::Model Pupper_;
    // Pupper constraints struct for RBDL
    RigidBodyDynamics::ConstraintSet pup_constraints_;

    // Retrieve the body COM Jacobian
    Matrix getBodyJacobian_(std::string body_id, const Eigen::Vector3d &offset = Eigen::Vector3d::Zero());

    // Retrieve the task Jacobian for specific task
    Matrix getTaskJacobian_(std::string task_name);
    Matrix getTaskJacobian_(unsigned priority);

    // Initialize RBDL constraints 
    void initConstraintSets_();

    // Retrieve the contact Jacobian for the active contacts
    void updateContactJacobian_(bool update_kinematics = true);

    // Store the robot state
    VectorNd joint_angles_;        // joint angles in radians
    VectorNd joint_velocities_;    // joint velocities in rad/s
    VectorNd robot_position_;      // robot center of mass position in meters QUESTION: Is this actually the COM? 
    double robot_height_;        // distance in m from floor to robot base (bottom PCB) in world coordinates
    VectorNd robot_velocity_;      // robot center of mass velocity in m/s
    Quat   robot_orientation_;   // robot orientation from IMU (Quaternion)
    Matrix Jc_;                  // contact Jacobian
    Matrix massMat_;             // mass matrix
    VectorNd b_g_;                 // coriolis plus gravity
    std::array<bool, 4> feet_in_contact_;                   // Feet in contact (boolean array: [back left, back right, front left, front right])

    // Control torques in Nm
    VectorNd control_torques_;

    // Body ids of lower links 
    uint back_left_lower_link_id_;
    uint back_right_lower_link_id_;
    uint front_left_lower_link_id_;
    uint front_right_lower_link_id_;

    // Map of robot tasks organized by name
    std::vector<Task*> robot_tasks_;
    std::unordered_map<std::string, unsigned> task_indices_;

    // Location of center of foot in lower link coordinates (center of hemisphere extended as a sphere)
    const RigidBodyDynamics::Math::Vector3d body_contact_point_left_ =  RigidBodyDynamics::Math::Vector3d(0.0, -.11, 0.0095);
    const RigidBodyDynamics::Math::Vector3d body_contact_point_right_ = RigidBodyDynamics::Math::Vector3d(0.0, -.11, -0.0095);

    // OSQP Solver
    std::unique_ptr<OSQPSettings> QP_settings_;
    std::unique_ptr<OSQPData> QP_data_;
    void convertEigenToCSC_(const Matrix &P, std::vector<c_float> &P_x, std::vector<c_int> &P_p, std::vector<c_int> &P_i, bool triup = false);
    void formQP(Matrix &P, VectorNd &q, Matrix &A, VectorNd &l, VectorNd &u);
    VectorNd solveQP(int n, int m, Matrix  &P, c_float *q, Matrix  &A, c_float *lb, c_float *ub);

    // Used for numerical derivative of task jacobians (j_dot)
    VectorNd taskDerivative_(const Task *T);
    double t_prev_;
};

#endif
