#ifndef _PUPPER_PLUGIN_HH_
#define _PUPPER_PLUGIN_HH_

#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/advertise_options.h"

namespace gazebo{

//A plugin to control a the Stanford Pupper V3 robot
class PupperPlugin : public ModelPlugin
{
public:
    //Constructor
    PupperPlugin(){}

    //Destructor
    virtual ~PupperPlugin(){}

    // Load function - Called on model creation and is used for setup and initialization
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // TODO: Called on every timestep of the simulation (control code goes here)
    void onUpdate();

    // TODO: Retrieve whatever data from these joints as we would actually get from the motors
    std::vector<float> getJointFeedback();

    // TODO: Apply command to the joints as we would actually do on the robot
    void controlJoints(std::vector<float> torques);

private:
    physics::ModelPtr model;                    // Pointer to the model in Gazebo
    physics::JointPtr front_left_joints[3];     // Array of joints on the front left leg
    physics::JointPtr front_right_joints[3];    // Array of joints on the front right leg
    physics::JointPtr back_left_joints[3];      // Array of joints on the back left leg
    physics::JointPtr back_right_joints[3];     // Array of joints on the back right leg
    event::ConnectionPtr updateConnection;      // Event connection between the Gazebo simulation and this plugin
};


} // end namespace gazebo


#endif