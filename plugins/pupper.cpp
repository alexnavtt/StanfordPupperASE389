#ifndef _PUPPER_PLUGIN_HH_
#define _PUPPER_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/advertise_options.h"

#include <iostream>

using std::cout;
using std::endl;

namespace gazebo
{
	//A plugin to control a Scrat's wheels
	class PupperPlugin : public ModelPlugin
	{
    public:
		//Constructor
		PupperPlugin()
		{

        }

		//Destructor
		virtual ~PupperPlugin()
		{

		}

		////////////////////////////////////////////////
		//----------------LOAD FUNCTION---------------//
		////////////////////////////////////////////////

		virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{

        }
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(PupperPlugin)
}

#endif