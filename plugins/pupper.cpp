#include "pupper.hpp"

#include <iostream>

using std::cout;
using std::endl;

namespace gazebo
{
    void PupperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Safety check
        if (_model->GetJointCount() != 12)
        {
            cout << "Invalid joint count, Scrat plugin not loaded" << endl;
            return;
        }

        // Store the model pointer
        model = _model;

        // Get the individual leg joints
        front_left_joints[0] = model->GetJoint("front_left_shoulder_1_joint");
        front_left_joints[1] = model->GetJoint("front_left_shoulder_2_joint");
        front_left_joints[2] = model->GetJoint("front_left_elbow_joint");

        front_right_joints[0] = model->GetJoint("front_right_shoulder_1_joint");
        front_right_joints[1] = model->GetJoint("front_right_shoulder_2_joint");
        front_right_joints[2] = model->GetJoint("front_right_elbow_joint");

        back_left_joints[0] = model->GetJoint("back_left_shoulder_1_joint");
        back_left_joints[1] = model->GetJoint("back_left_shoulder_2_joint");
        back_left_joints[2] = model->GetJoint("back_left_elbow_joint");

        back_right_joints[0] = model->GetJoint("back_right_shoulder_1_joint");
        back_right_joints[1] = model->GetJoint("back_right_shoulder_2_joint");
        back_right_joints[2] = model->GetJoint("back_right_elbow_joint");

        //Connect plugin to Gazebo world instance
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&PupperPlugin::onUpdate, this));
    }



    void PupperPlugin::onUpdate(){

    }

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(PupperPlugin)
}
