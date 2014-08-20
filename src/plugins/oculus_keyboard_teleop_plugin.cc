// Copyright (c) 2014 Mohit Shridhar, David Lee

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


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

      	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&OculusGazeboNavigator::OnUpdate, this));
    }

    public: void OnUpdate()
    {
    	ros::spinOnce();
    	updateVels();
    	Stabilize();
    }

    private: void updateVels()
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
    private: transport::SubscriberPtr hmdSub;

    private: math::Vector3 cmd_linear_vel, cmd_angular_vel;
    private: math::Quaternion headOrientation;

   	private: event::ConnectionPtr updateConnection;

  };

  GZ_REGISTER_MODEL_PLUGIN(OculusGazeboNavigator)
}