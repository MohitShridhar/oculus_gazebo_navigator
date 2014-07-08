/*
A plugin to control the virtual Oculus-Rift camera in Gazebo 3.0
Copyright (C) 2014  David Lee, Mohit Shridhar

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <gazebo/rendering/Scene.hh>
#include <boost/shared_ptr.hpp>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/rendering/OculusCamera.hh>
#include <gazebo/rendering/RenderEngine.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/WindowManager.hh>
#include <gazebo/rendering/OculusCamera.hh>
#include <gazebo/transport/transport.hh>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


namespace gazebo
{   
  
  typedef const boost::shared_ptr<const msgs::Quaternion> QuaternionPtr;

  class OculusGazeboNavigator : public ModelPlugin
  {

    public: OculusGazeboNavigator()
    {
      std::string name = "oculus_gazebo_navigator";
      int argc = 0;
      ros::init(argc, NULL, name);

    }

    public: ~OculusGazeboNavigator()
    {
      delete this->rosNode;
      transport::fini();
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)
    {
    	establishLinks(_parent);
    	setupHMDSubscription();
    }

    private: void setupHMDSubscription()
    {
    	transport::NodePtr node(new transport::Node());
  		node->Init();

  		this->hmdSub = node->Subscribe("~/oculusHMD", &OculusGazeboNavigator::GzHMDCallback, this);
    }

    private: void GzHMDCallback(QuaternionPtr &msg)
    {
    	headOrientation = msgs::Convert(*msg);
    }

    private: void establishLinks(physics::ModelPtr _parent)
    {
    	this->model = _parent;
    	this->bodyLink = this->model->GetLink("body");

    	this->rosNode = new ros::NodeHandle("/camera_controller");
    	this->sub_twist = this->rosNode->subscribe<geometry_msgs::Twist>("/camera_controller/twist", 1, &OculusGazeboNavigator::ROSCallbackTwist, this);

      	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&OculusGazeboNavigator::OnUpdate, this));
    }

    public: void OnUpdate()
    {
    	ros::spinOnce();
    	UpdateVel();
    	Stabilize();
    }

    private: void UpdateVel()
    {	
    	math::Vector3 currLinearVel = this->bodyLink->GetRelativeLinearVel();
    	// math::Vector3 currAngularVel = this->bodyLink->GetRelativeAngularVel();

    	// math::Quaternion headOrientation = this->oculusCamera->GetWorldRotation();
    	math::Vector3 newLinearVel = headOrientation.RotateVector(cmd_linear_vel);

    	this->bodyLink->SetLinearVel(math::Vector3(newLinearVel.x, newLinearVel.y, currLinearVel.z));
    	// this->bodyLink->SetAngularVel(math::Vector3(0, 0, cmd_angular_vel.z));
    }

    private: void Stabilize()
    {
    	math::Pose currPose = this->model->GetWorldPose();
    	math::Vector3 currPosition = currPose.pos;

    	math::Pose stabilizedPose;
    	stabilizedPose.pos = currPosition;

    	stabilizedPose.rot.x = 0;
    	stabilizedPose.rot.y = 0;

    	stabilizedPose.rot.z = currPose.rot.z;

    	this->model->SetWorldPose(stabilizedPose);    
    }

    private: void ROSCallbackTwist(const geometry_msgs::Twist::ConstPtr& msg)
    {
    	// Movements constricted to z plane:
    	cmd_linear_vel.x = msg->linear.x;
    	cmd_linear_vel.y = msg->linear.y;

    	cmd_angular_vel.z = msg->angular.z;
    }

    private: physics::ModelPtr model;
    private: physics::LinkPtr bodyLink;
    private: rendering::OculusCameraPtr oculusCamera;

    private: ros::NodeHandle* rosNode;
    private: ros::Subscriber sub_twist;

    private: transport::SubscriberPtr hmdSub;

    private: math::Vector3 cmd_linear_vel, cmd_angular_vel;
    private: math::Quaternion headOrientation;

   	private: event::ConnectionPtr updateConnection;

  };

  GZ_REGISTER_MODEL_PLUGIN(OculusGazeboNavigator)
}