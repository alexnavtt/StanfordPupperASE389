#include <iostream>
#include "ase389/PupperWBC.hpp"
#include "rbdl/addons/urdfreader/urdfreader.h"
#include "ase389/PupperUrdfString.hpp"

using std::vector;
using std::array;
using std::string;
using std::cout;
using std::endl;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// Constructor
PupperWBC::PupperWBC(){
    joint_angles_     = VectorNd::Zero(NUM_JOINTS);
    joint_velocities_ = VectorNd::Zero(NUM_JOINTS);
    joint_torques_    = VectorNd::Zero(NUM_JOINTS);
    
}

void PupperWBC::updateController(const array<float, NUM_JOINTS>& joint_angles, 
                                 const array<float, NUM_JOINTS>& joint_velocities,
                                 const array<float, NUM_JOINTS>& joint_torques)
{
    for (int i = 0; i < NUM_JOINTS; i++){
        joint_angles_[i]     = joint_angles[i];
        joint_velocities_[i] = joint_velocities[i];
        joint_torques_[i]    = joint_torques[i];
    }
}

void PupperWBC::addTask(unsigned priority, string name, Task T){
    if (robot_tasks_.size() <= priority){
        robot_tasks_.resize(priority + 1);
    }
    robot_tasks_[priority] = T;
    task_indices_[name] = priority;
}

void PupperWBC::Load(string filename){
    // Load the model
    // RigidBodyDynamics::Addons::URDFReadFromFile(filename.c_str(), &Pupper_, true, true);
    RigidBodyDynamics::Addons::URDFReadFromString(pupper_urdf_string, &Pupper_, true, true);
    // Summarize model characteristics
    printf("Loaded model with %d DOFs\n", Pupper_.dof_count);
    printf("*\tQ Count:   \t%d\n", Pupper_.q_size);
    printf("*\tQdot Count:\t%d\n", Pupper_.qdot_size);
    printf("*\tBody Count:\t%zd\n", Pupper_.mBodies.size());
    printf("*\tJoint Count:\t%zd\n", Pupper_.mJoints.size());
    printf("*\tBody Names:\n");
    for (int i = 0; i < Pupper_.mBodies.size(); i++)
        cout << "*\t|----" << i << "\t=> " << Pupper_.GetBodyName(i) << endl;

    //Test
    array<float, 4> feet_in_contact = {1,1,1,1};
    setContacts(feet_in_contact);

    // Test
    getBodyJacobian_();
}

// Set rbdl model contact constraints
void PupperWBC::setContacts(const array<float, 4> feet_in_contact){
    //TODO
}

// Retrieve body Jacobian
MatrixNd PupperWBC::getBodyJacobian_(){
    // Create output
    static MatrixNd J = Eigen::MatrixXd::Zero(6, Pupper_.qdot_size);
    printf("\n\nBody Jacobian Test:\n");

    // Fill the Jacobian matrix
    const char* body_name = "bottom_PCB";
    CalcBodySpatialJacobian(Pupper_, joint_angles_, Pupper_.GetBodyId(body_name), J, true);
    cout << "Jacobian for \"" << body_name << "\" is: \n" << J.transpose() << endl;

    return J;
}
