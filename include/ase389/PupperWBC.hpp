#ifndef PUPPER_CONTROLLER_HH_
#define PUPPER_CONTROLLER_HH_

#include <vector>
#include <array>
#include <string>
#include <cstring>
#include <unordered_map>

#include "osqp.h"
#include "rbdl/rbdl.h"
#include "eigen3/Eigen/Dense"
#include "ase389/WBCTask.hpp"

#define ROBOT_NUM_JOINTS 12
#define NUM_JOINTS 18   // 12 motors plus 6 floating base joints
#define NUM_Q 19        // 12 motors plus 3 xyz plus 4 quaternion, refer to: https://rbdl.github.io/df/dbe/joint_description.html

class PupperWBC{
public:
    // Constructor
    PupperWBC();

    // Take in the current robot data
    void updateController(const std::array<float, ROBOT_NUM_JOINTS>& joint_angles, 
                          const std::array<float, ROBOT_NUM_JOINTS>& joint_velocities,
                          const Eigen::Quaternion<float>& body_quaternion,
                          const std::array<bool, 4>& feet_in_contact);

    // Add a task to the robot's task list
    void addTask(unsigned priority, std::string name, Task* task);

    // Load the Pupper model from a URDF
    void Load(std::string filename);

    // Get the torque command fulfilling the current tasks
    std::array<float, 12> calculateOutputTorque();

// private:
    // The Pupper model for RBDL
    RigidBodyDynamics::Model Pupper_;
    // Pupper constraints struct for RBDL
    RigidBodyDynamics::ConstraintSet pup_constraints_;

    // Retrieve the body COM Jacobian
    RigidBodyDynamics::Math::MatrixNd getBodyJacobian_(std::string body_id);

    // Retrieve the task Jacobian for specific task
    RigidBodyDynamics::Math::MatrixNd getTaskJacobian_(std::string task_name);
    RigidBodyDynamics::Math::MatrixNd getTaskJacobian_(unsigned priority);

    // Initialize RBDL constraints 
    void initConstraintSets_();

    // Retrieve the contact Jacobian for the active contacts
    RigidBodyDynamics::Math::MatrixNd getContactJacobian_();
    
    // Retrieve the null space for a specific task
    RigidBodyDynamics::Math::MatrixNd getTaskNullSpace_(std::string task_name);
    RigidBodyDynamics::Math::MatrixNd getTaskNullSpace_(unsigned priority);

    // Joint angles in radians
    RigidBodyDynamics::Math::VectorNd joint_angles_;

    // Joint velocities in rad/s
    RigidBodyDynamics::Math::VectorNd joint_velocities_;

    // Control torques in Nm
    RigidBodyDynamics::Math::VectorNd control_torques_;

    // Robot orientation from IMU (Quaternion)
    RigidBodyDynamics::Math::Quaternion robot_orientation_;

    // Feet in contact (boolean array: [back left, back right, front left, front right])
    std::array<bool, 4> feet_in_contact_;

    // Body ids of lower links 
    uint back_left_lower_link_id_;
    uint back_right_lower_link_id_;
    uint front_left_lower_link_id_;
    uint front_right_lower_link_id_;

    // Map of robot tasks organized by name
    std::vector<Task*> robot_tasks_;
    std::unordered_map<std::string, unsigned> task_indices_;

    // Joint selection matrix (takes the form [0_6, I_12]^T)
    RigidBodyDynamics::Math::MatrixNd U_;

    // OSQP Solver
    std::unique_ptr<OSQPSettings> QP_settings_;
    std::unique_ptr<OSQPData> QP_data_;
    void convertEigenToCSC_(const RigidBodyDynamics::Math::MatrixNd &P, std::vector<c_float> &P_x, std::vector<c_int> &P_p, std::vector<c_int> &P_i);
    void convertEigenToCfloat_(const RigidBodyDynamics::Math::VectorNd &q, std::vector<c_float> &q_c);
    void testQPSolver();
    void formQP();
};

#endif
