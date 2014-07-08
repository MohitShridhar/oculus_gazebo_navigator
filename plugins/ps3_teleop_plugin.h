#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <gazebo/rendering/Scene.hh>
#include <boost/shared_ptr.hpp>
#include <gazebo/msgs/msgs.hh>

#include <sensor_msgs/Joy.h>

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

// FPV Constants
#define WALKING_SPEED_X 2.0 // in m/s
#define WALKING_SPEED_Y 1.5

#define RUNNING_SPEED_X 4.0
#define RUNNING_SPEED_Y 3.0

#define VERTICAL_SPEED 1.0
#define CEILING_HEIGHT 40.0 // in meters
#define FLOOR_HEIGHT 2.52

// Bot-Control Constants
#define BOT_MODEL_NAME "youbot"
#define BOT_CMD_TOPIC_NAME "/youbot/cmd_vel"
#define NORMAL_LINEAR_SPEED 0.4 // in m/s
#define NORMAL_ANGULAR_SPEED 0.785 // in rad/s

#define FAST_LINEAR_SPEED 1.1
#define FAST_ANGULAR_SPEED  1.60 

// Buttons:
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

// Assigned functions:
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

#define ROS_RATE 10.0

namespace gazebo
{

	typedef const boost::shared_ptr<const msgs::Quaternion> QuaternionPtr;

	class OculusGazeboNavigator : public ModelPlugin {

	public: OculusGazeboNavigator();
	public: ~OculusGazeboNavigator();

	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr);
	public: void OnUpdate();

	private: void initVars();
	private: void setupHMDSubscription();
	private: void establishLinks(physics::ModelPtr _parent);

	private: void GzHMDCallback(QuaternionPtr &msg);
    private: void updateBtnStates(const sensor_msgs::Joy::ConstPtr& msg);
    private: void ROSCallbackJoy(const sensor_msgs::Joy::ConstPtr& msg);
 	private: void updateToggleStates();

	private: void updateVel();
	private: void updateBotVel();
	private: void updateBotVelIsolated();
	private: void updateBotVelListener();
	private: void updateFpvVel();

	private: void calibrateBot();
	private: math::Pose transformTo3DOFPoseRotated(tf::StampedTransform trans);
	private: math::Quaternion tfToGzQuat(tf::Quaternion quatTf);
	private: bool lookupTfListener();

	private: void resetBot();
	private: void toggleBotIsoControlMode();
	private: void toggleBotTfListenerControlMode();
	private: void toggleGravityMode();
	private: float computeHoverVelocity(float currVerticalVel);
	private: void toggleCollisionMode();	
	private: void toggleXrayMode();
	private: bool wasActivated(uint btnRef);

    private: math::Vector3 constrainVerticalMovement(math::Vector3 currLinearVel);
    private: bool isStrayVerticalVel(math::Vector3 currLinearVel);
    private: math::Vector3 computeVelocities(math::Vector3 currLinearVel);
    private: void stabilize();

    // Global Vars:

    private: physics::ModelPtr model;
    private: physics::LinkPtr bodyLink;

    private: ros::NodeHandle* rosNode;
    private: transport::NodePtr gazeboNode;
    private: ros::Subscriber sub_twist;
    private: ros::Publisher pub_cmd_vel;

    private: tf::TransformListener tfListener;
    private: tf::StampedTransform transform;

    private: transport::SubscriberPtr hmdSub;
    private: transport::PublisherPtr pubFpvPose;

    private: math::Vector3 cmd_linear_vel, cmd_hovering_vel;
    private: geometry_msgs::Twist bot_cmd_vel;
    private: math::Pose botOffsetPose;
    private: math::Quaternion headOrientation;

    private: event::ConnectionPtr updateConnection;

    // PS3 Dual-shock controller buttons and joysticks:
    private: float stickRightX, stickRightY, stickLeftX, stickLeftY;
    private: bool currBtn[16], prevBtn[16];

    // World States:
    private: bool isGravityEnabled, isCollisionEnabled, isXrayVisionEnabled, isBotIsoControlEnabled, isBotTfListenerControlEnabled;

	};

	GZ_REGISTER_MODEL_PLUGIN(OculusGazeboNavigator)
}
