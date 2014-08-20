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

#include <sensor_msgs/Joy.h>
#include <actionlib_msgs/GoalID.h>

#include <gazebo/rendering/OculusCamera.hh>
#include <gazebo/rendering/RenderEngine.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/WindowManager.hh>
#include <gazebo/rendering/OculusCamera.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportIface.hh>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

// Camera Default Settings:
#define DEFAULT_WALKING_SPEED_X 2.0 // in m/s
#define DEFAULT_WALKING_SPEED_Y 1.5

#define DEFAULT_RUNNING_SPEED_X 4.0
#define DEFAULT_RUNNING_SPEED_Y 3.0

#define DEFAULT_VERTICAL_SPEED 1.0
#define DEFAULT_LIMIT_UPPER_POS 41.0 // in meters
#define DEFAULT_LIMIT_LOWER_POS 2.52

// Bot-Control Default Settings:
#define DEFAULT_BOT_MIRROR_MODE_FIXED_Z_POS 0.79

#define DEFAULT_BOT_LINEAR_SPEED 0.4 
#define DEFAULT_BOT_ANGULAR_SPEED 0.785 // rad/s

#define DEFAULT_BOT_FAST_LINEAR_SPEED 1.1
#define DEFAULT_BOT_FAST_ANGULAR_SPEED  1.60 

#define DEFAULT_BOT_MODEL_NAME "robot0"
#define DEFAULT_BOT_CMD_VEL_TOPIC "robot0/cmd_vel"

// PS3 Controller Buttons:
#define SELECT 0

#define LEFT_JOY 1
#define RIGHT_JOY 2
#define START 3

#define UP 4
#define RIGHT 5
#define DOWN 6
#define LEFT 7

#define LEFT_REAR_TAP 8
#define RIGHT_REAR_TAP 9
#define LEFT_FORWARD_TAP 10
#define RIGHT_FORWARD_TAP 11

#define TRIANGLE 12
#define CIRCLE 13
#define CROSS 14
#define SQUARE 15

#define NUM_CONTROLLER_BTNS 16

// Assigned Button functions:
#define XRAY_BTN CROSS
#define COLLISION_BTN SQUARE
#define GRAVITY_BTN TRIANGLE

#define BOT_CONTROL_BTN SELECT
#define BOT_RESET_BTN START
#define BOT_CALIBRATE_BTN START

#define HOVER_UP_BTN RIGHT_FORWARD_TAP
#define HOVER_DOWN_BTN LEFT_FORWARD_TAP
#define HOVER_STEADY_BTN RIGHT_REAR_TAP

// Programmable service function:
#define SERVICE_UP_BTN UP
#define SERVICE_DOWN_BTN DOWN
#define SERVICE_LEFT_BTN LEFT
#define SERVICE_RIGHT_BTN RIGHT

#define REQUEST_UP_BTN "up_btn"
#define REQUEST_DOWN_BTN "down_btn"
#define REQUEST_RIGHT_BTN "right_btn"
#define REQUEST_LEFT_BTN "left_btn"

// Parameter namespaces:
#define PARAM_NAMESPACE_CAMERA "/oculus_gazebo_navigator/ps3_teleop/camera/"
#define PARAM_NAMESPACE_ROBOT "/oculus_gazebo_navigator/ps3_teleop/robot/"

// Other constants:
#define ROS_RATE 10.0

namespace gazebo
{
	class OculusGazeboNavigator : public ModelPlugin {

	public:
		OculusGazeboNavigator();
		~OculusGazeboNavigator();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr);
		
	private:
		void initVars();
		void setuphmd_orientation_sub();
		void establishLinks(physics::ModelPtr _parent);
		void OnUpdate();

		void loadParams();
		void loadCameraSettings();
		void loadRobotSettings();
		void checkIfBotExists();

		void GzHMDCallback(const boost::shared_ptr<const msgs::Quaternion> &msg);
	    void updateBtnStates(const sensor_msgs::Joy::ConstPtr& msg);
	    void updateStickStates(const sensor_msgs::Joy::ConstPtr& msg);
	    void ps3_controller_cb(const sensor_msgs::Joy::ConstPtr& msg);
	    void origin_offset_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	 	void updateToggleStates();

		void updateVels();
		void updateBotVel();
		void updateBotVelIsolated();
		void updateBotVelMirrored();
		void updateCameraVel();

		void calibrateBot();
		math::Pose transformTo3DOFPose(tf::StampedTransform trans);
		math::Quaternion tfToGzQuat(tf::Quaternion quatTf);
		bool lookupTfListener();

		void resetBot();
		void toggleBotIsoControlMode();
		void toggleBotTfListenerControlMode();
		void toggleGravityMode();
		void toggleNavStackControlMode();
		float computeHoverVelocity(float currVerticalVel);
		void toggleCollisionMode();	
		void toggleXrayMode();
		bool wasActivated(uint btnRef);

		void checkModeChanges();
		void checkWorldPropChanges();
		void checkBotControlChanges();
		void checkServiceRequests();

	    math::Vector3 constrainVerticalMovement(math::Vector3 currLinearVel);
	    bool isStrayVerticalVel(math::Vector3 currLinearVel);
	    math::Vector3 computeVelocities(math::Vector3 currLinearVel);
	    void stabilizeCamera();

	private:
	    physics::ModelPtr model, botPtr;
	    physics::LinkPtr bodyLink;

	    ros::NodeHandle* rosNode;
	    transport::NodePtr gazeboNode;
	    ros::Subscriber joystick_sub, origin_offset_sub;
	    ros::Publisher bot_cmd_vel_pub, cancel_nav_goal_pub;

	    tf::TransformListener tfListener;
	    tf::StampedTransform transform;

	    transport::SubscriberPtr hmd_orientation_sub;
	    transport::PublisherPtr camera_pose_pub;

	    math::Vector3 cmd_linear_vel, cmd_hovering_vel;
	    geometry_msgs::Twist bot_cmd_vel;
	    math::Pose botOffsetPose;
	    math::Quaternion headOrientation, botOffsetQuat;

	    event::ConnectionPtr updateConnection;

	    // PS3 Dual-shock controller buttons and joysticks:
	    float stickRightX, stickRightY, stickLeftX, stickLeftY;
	    bool currBtn[16], prevBtn[16];

	    // World States:
	    bool isGravityEnabled, isCollisionEnabled, isXrayVisionEnabled, isBotIsoControlEnabled, isBotTfListenerControlEnabled, isAutoNavEnabled;

	    // Camera & Robot Parameters:
	 	double max_walking_speed_x, max_walking_speed_y, max_running_speed_x, max_running_speed_y, max_vertical_speed, upper_position_limit, lower_position_limit;  
	 	double bot_normal_linear_speed, bot_normal_angular_speed, bot_fast_linear_speed, bot_fast_angular_speed, mirror_mode_fixed_z_pos;
	 	std::string bot_model_name, bot_cmd_vel_topic;
	 	bool isBotAvailable;
	};

	GZ_REGISTER_MODEL_PLUGIN(OculusGazeboNavigator)
}
