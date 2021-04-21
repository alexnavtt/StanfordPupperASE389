#include "pupper.hpp"

#include <iostream>

using std::cout;
using std::endl;
using std::vector;

static double angleDiff(double angle1, double angle2){
    return fmod(angle1 - angle2 + M_PI, 2*M_PI) - M_PI;
}

namespace gazebo
{

// Load the model
void PupperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Safety check
    if (_model->GetJointCount() != 12)
    {
        cout << "Invalid joint count, Pupper plugin not loaded" << endl;
        return;
    }

    // Store the model pointer
    model_ = _model;

    // Get the individual leg joints
    front_left_joints_[0]  = model_->GetJoint("front_left_shoulder_1_joint");
    front_left_joints_[1]  = model_->GetJoint("front_left_shoulder_2_joint");
    front_left_joints_[2]  = model_->GetJoint("front_left_elbow_joint");

    front_right_joints_[0] = model_->GetJoint("front_right_shoulder_1_joint");
    front_right_joints_[1] = model_->GetJoint("front_right_shoulder_2_joint");
    front_right_joints_[2] = model_->GetJoint("front_right_elbow_joint");

    back_right_joints_[0]  = model_->GetJoint("back_right_shoulder_1_joint");
    back_right_joints_[1]  = model_->GetJoint("back_right_shoulder_2_joint");
    back_right_joints_[2]  = model_->GetJoint("back_right_elbow_joint");

    back_left_joints_[0]   = model_->GetJoint("back_left_shoulder_1_joint");
    back_left_joints_[1]   = model_->GetJoint("back_left_shoulder_2_joint");
    back_left_joints_[2]   = model_->GetJoint("back_left_elbow_joint");

    // Also collect all joints into a single array
    for (int i = 0; i < 3; i++){
        all_joints_[i + 3*FRONT_LEFT_LEG]  = front_left_joints_[i];
        all_joints_[i + 3*FRONT_RIGHT_LEG] = front_right_joints_[i];
        all_joints_[i + 3*BACK_RIGHT_LEG]  = back_right_joints_[i];
        all_joints_[i + 3*BACK_LEFT_LEG]   = back_left_joints_[i];
    }

    for (int i = 0; i < 12; i++){
        control_torques_[i] = 0;
    }

    //Connect plugin to Gazebo world instance
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&PupperPlugin::onUpdate, this));

    // Set up update rate variables
    update_interval_  = common::Time(0, common::Time::SecToNano(0.02));  // In the form common::Time(sec, nanoSec): 50Hz
    last_update_time_ = common::Time::GetWallTime();
}


// Control the joints on one leg
void PupperPlugin::controlJoints(enum PupperLegs leg, vector<float> torques){
    switch(leg){
    case FRONT_LEFT_LEG:
        front_left_joints_[0]->SetForce(0, torques[0]);
        front_left_joints_[1]->SetForce(0, torques[1]);
        front_left_joints_[2]->SetForce(0, torques[2]);
        break;

    case FRONT_RIGHT_LEG:
        front_right_joints_[0]->SetForce(0, torques[0]);
        front_right_joints_[1]->SetForce(0, torques[1]);
        front_right_joints_[2]->SetForce(0, torques[2]);
        break;

    case BACK_RIGHT_LEG:
        back_right_joints_[0]->SetForce(0, torques[0]);
        back_right_joints_[1]->SetForce(0, torques[1]);
        back_right_joints_[2]->SetForce(0, torques[2]);
        break;

    case BACK_LEFT_LEG:
        back_left_joints_[0]->SetForce(0, torques[0]);
        back_left_joints_[1]->SetForce(0, torques[1]);
        back_left_joints_[2]->SetForce(0, torques[2]);
        break;
    }
}


// Control all joints on the robot 
void PupperPlugin::controlAllJoints(vector<float> torques){
    for (int i = 0; i < 12; i++){
        all_joints_[i]->SetForce(0, torques[i]);
    }
} 


// Set joint positions through Gazebo (useful for debugging)
void PupperPlugin::setJointPositions(vector<float> angles){
    for (int i = 0; i < 12; i++){
        double error = angleDiff(angles[i], all_joints_[i]->Position(0));
        control_torques_[i] = 10*error;
    }
}


// Get torque measurements from the joints
vector<float> PupperPlugin::getJointFeedback(){
    vector<float> joint_torques(12);

    for (uint8_t i = 0; i < 12; i++){
        physics::JointWrench joint_wrench = all_joints_[i]->GetForceTorque(0);
        joint_torques[i] = (float)joint_wrench.body1Torque.Z();
    }

    return joint_torques;        
}

// Update the simulation with the current control torques
void PupperPlugin::applyTorques_(){
    for (int i = 0; i < 12; i++){
        all_joints_[i]->SetForce(0, control_torques_[i]);
    }
}

// Called on every simulation time step
void PupperPlugin::onUpdate(){
    static vector<float> test_angles = {-0.1,  M_PI_4,  M_PI_2, 
                                         0.1, -M_PI_4, -M_PI_2,
                                         0.1, -M_PI_4, -M_PI_2,
                                        -0.1,  M_PI_4,  M_PI_2};

    //Manage publisher update rate
    if (common::Time::GetWallTime() - last_update_time_ > update_interval_)
    {
        getJointFeedback();
        setJointPositions(test_angles);
        last_update_time_ = common::Time::GetWallTime();
    }

    applyTorques_();
}

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(PupperPlugin)

}   // end namespace gazebo
