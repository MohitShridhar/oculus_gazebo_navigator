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
        initVars();
    }

    private: void initVars()
    {
        this->isGravityEnabled = true;
        this->isCollisionEnabled = true;
        this->isXrayVisionEnabled = false;
        this->isBotIsoControlEnabled = false;
        this->isBotTfListenerControlEnabled = false;

        this->botOffsetPose = this->model->GetWorld()->GetModel(BOT_MODEL_NAME)->GetWorldPose();

        ros::Rate rate(ROS_RATE);
    }

    private: void setupHMDSubscription()
    {
        this->gazeboNode = transport::NodePtr(new transport::Node());
        this->gazeboNode->Init();

  		this->hmdSub = this->gazeboNode->Subscribe("~/oculusHMD", &OculusGazeboNavigator::GzHMDCallback, this);
        this->pubFpvPose = this->gazeboNode->Advertise<msgs::Pose>("~/oculus/fpvPose");

    }

    private: void GzHMDCallback(QuaternionPtr &msg)
    {
    	headOrientation = msgs::Convert(*msg);
    }

    private: void establishLinks(physics::ModelPtr _parent)
    {
    	this->model = _parent;
    	this->bodyLink = this->model->GetLink("body");

    	this->rosNode = new ros::NodeHandle("/joy0");
    	this->sub_twist = this->rosNode->subscribe<sensor_msgs::Joy>("/joy0", 50, &OculusGazeboNavigator::ROSCallbackJoy, this);
        this->pub_cmd_vel = this->rosNode->advertise<geometry_msgs::Twist>(BOT_CMD_TOPIC_NAME, 50);

      	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&OculusGazeboNavigator::OnUpdate, this));
    }

    public: void OnUpdate()
    {
    	ros::spinOnce();
    	updateVel();
        stabilize();
    }

    private: void updateVel()
    {
        //Bot control: TF Listener:
        if (this->isBotTfListenerControlEnabled) {
            updateFpvVel();
            updateBotVelListener();
            return;
        }

        //Bot control: Joystick:
        if (!this->isBotIsoControlEnabled) {
            updateFpvVel();
            updateBotVel();
        } else {
            updateBotVelIsolated();
        }
    }

    // Dual controls: FPV & Bot-control (Left joystick -> FPV | Right joystick -> Bot-control)
    private: void updateBotVel()
    {
        if (!currBtn[RIGHT_REAR_TAP]) { // linear controls

            if (fabs(stickRightY) > fabs(stickRightX)) {
                bot_cmd_vel.linear.x = stickRightY * (currBtn[LEFT_REAR_TAP] ? FAST_LINEAR_SPEED : NORMAL_LINEAR_SPEED);
                bot_cmd_vel.linear.y = 0.0;            
            } else {
                bot_cmd_vel.linear.x = 0.0;
                bot_cmd_vel.linear.y = -stickRightX * (currBtn[LEFT_REAR_TAP] ? FAST_LINEAR_SPEED : NORMAL_LINEAR_SPEED);
            }

            bot_cmd_vel.angular.z = 0.0;

        } else if (currBtn[RIGHT_REAR_TAP]) { // angular control

            bot_cmd_vel.linear.x = 0.0;
            bot_cmd_vel.linear.y = 0.0;

            bot_cmd_vel.angular.z = -stickRightX * (currBtn[LEFT_REAR_TAP] ? FAST_ANGULAR_SPEED : NORMAL_ANGULAR_SPEED);
        }

        bot_cmd_vel.angular.x = 0.0;
        bot_cmd_vel.angular.y = 0.0;

        pub_cmd_vel.publish(bot_cmd_vel);
    }

    // Isolated Mode: Bot-control (Left joystick -> x,y axis | Right joystick -> yaw)
    private: void updateBotVelIsolated()
    {
        // Mimic "Mecanum wheels" effect:
        if (fabs(stickLeftY) > fabs(stickLeftX)) {
            bot_cmd_vel.linear.x = stickLeftY * (currBtn[LEFT_REAR_TAP] ? FAST_LINEAR_SPEED : NORMAL_LINEAR_SPEED);
            bot_cmd_vel.linear.y = 0.0;
        } else {
            bot_cmd_vel.linear.x = 0.0;
            bot_cmd_vel.linear.y = -stickLeftX * (currBtn[LEFT_REAR_TAP] ? FAST_LINEAR_SPEED : NORMAL_LINEAR_SPEED);
        }

        bot_cmd_vel.angular.x = 0.0;
        bot_cmd_vel.angular.y = 0.0;
        bot_cmd_vel.angular.z = -stickRightX * (currBtn[LEFT_REAR_TAP] ? FAST_ANGULAR_SPEED : NORMAL_ANGULAR_SPEED);

        pub_cmd_vel.publish(bot_cmd_vel);
    }

    // TF Listener Mode: Bot World-Pose controlled directly via a TF Listener
    private: void updateBotVelListener()
    {
        bool wasLookUpSuccessful = lookupTfListener();

        if (!wasLookUpSuccessful) {
            return;
        } 

        math::Pose newBotPose = transformTo3DOFPose(transform);

        newBotPose.pos.x -= botOffsetPose.pos.x;
        newBotPose.pos.y -= botOffsetPose.pos.y;
        newBotPose.rot.z -= botOffsetPose.rot.z;

        this->model->GetWorld()->GetModel(BOT_MODEL_NAME)->SetWorldPose(newBotPose);
    }

    private: void calibrateBot()
    {
        bool wasLookUpSuccessful = lookupTfListener();

        if (!wasLookUpSuccessful) {
            printf("Oculus Navigator: CALIBRATION FAILED - Could not get TF data\n");
            return;
        }

        math::Pose realBotPose = transformTo3DOFPose(transform);
        math::Pose virtualBotPose = this->model->GetWorld()->GetModel(BOT_MODEL_NAME)->GetWorldPose();

        botOffsetPose.pos.x = realBotPose.pos.x - virtualBotPose.pos.x;
        botOffsetPose.pos.y = realBotPose.pos.y - virtualBotPose.pos.y;
        botOffsetPose.rot.z = realBotPose.rot.z - virtualBotPose.rot.z;

        printf("Oculus Navigator: New bot-offset pose – x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f\n-----------------\n", botOffsetPose.pos.x, botOffsetPose.pos.y, botOffsetPose.pos.z, botOffsetPose.rot.x, botOffsetPose.rot.y, botOffsetPose.rot.z);            
    }   

    private: math::Pose transformTo3DOFPose(tf::StampedTransform trans)
    {
        tf::Quaternion quatTf;
        double roll, pitch, yaw;

        tf::Matrix3x3(trans.getRotation()).getRPY(roll, pitch, yaw);

        math::Pose newPose;

        newPose.pos.x = trans.getOrigin().x();
        newPose.pos.y = trans.getOrigin().y();
        newPose.pos.z = botOffsetPose.pos.z;

        newPose.rot.z = yaw;

        return newPose;        
    }

    private: bool lookupTfListener()
    {
        try {
            tfListener.lookupTransform("map", "base_link", ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("Active TF Listener failed to get Pose data: %s\n", ex.what());
            toggleBotTfListenerControlMode();
            return false;
        }

        return true;
    }

    private: void updateToggleStates()
    {
        // Toggle TF-Listener mode & Joystick bot-control:

        if (wasActivated(BOT_CONTROL_BTN) && !currBtn[LEFT_REAR_TAP]) {
            toggleBotTfListenerControlMode();
            printf("Oculus Navigator: Bot TF Listener control - %s\n", (isBotTfListenerControlEnabled ? "enabled" : "disabled"));
        }

        // Calibrate Real-world pos & Gazebo bot origin through manual alignment:

        if (wasActivated(BOT_CALIBRATE_BTN) && currBtn[LEFT_REAR_TAP] && currBtn[RIGHT_REAR_TAP]) {
            calibrateBot();
        }

        // World properties control:

        if (wasActivated(GRAVITY_BTN)) {
            toggleGravityMode();
            printf("Oculus Navigator: Gravity %s\n", (isGravityEnabled ? "enabled" : "disabled"));
        }

        if (wasActivated(COLLISION_BTN)) {
            toggleCollisionMode();
            printf("Oculus Navigator: Collision-mode %s\n", (isCollisionEnabled ? "enabled" : "disabled"));
        }

        if (wasActivated(XRAY_BTN)) {
            toggleXrayMode();
            printf("Oculus Navigator: X-Ray mode %s\n", (isXrayVisionEnabled ? "enabled" : "disabled"));
        }

        // Bot-control:

        if (wasActivated(BOT_CONTROL_BTN) && currBtn[LEFT_REAR_TAP]) {
            toggleBotIsoControlMode();
            printf("Oculus Navigator: Bot-Control mode %s, Fpv Navigator %s\n", (isBotIsoControlEnabled ? "enabled" : "disabled"), (isBotIsoControlEnabled ? "disabled" : "enabled"));
        }

        if (wasActivated(BOT_RESET_BTN) && !currBtn[LEFT_REAR_TAP] && !currBtn[RIGHT_REAR_TAP]) {
            resetBot();
            printf("Oculus Navigator: '%s' reset to offset pose\n", BOT_MODEL_NAME);
        }

        // Programmable services based on pose context:

        if (wasActivated(SERVICE_UP_BTN) || wasActivated(SERVICE_DOWN_BTN) || wasActivated(SERVICE_RIGHT_BTN) || wasActivated(SERVICE_LEFT_BTN)) {
            this->pubFpvPose->Publish(msgs::Convert(this->model->GetWorldPose()));
        } else {
            return; // no service was required
        }

        if (wasActivated(SERVICE_UP_BTN)) {
            transport::requestNoReply(this->gazeboNode->GetTopicNamespace(), REQUEST_UP_BTN, "all");
        }

        if (wasActivated(SERVICE_DOWN_BTN)) {
            transport::requestNoReply(this->gazeboNode->GetTopicNamespace(), REQUEST_DOWN_BTN, "all");
        }

        if (wasActivated(SERVICE_RIGHT_BTN)) {
            transport::requestNoReply(this->gazeboNode->GetTopicNamespace(), REQUEST_RIGHT_BTN, "all");
        }

        if (wasActivated(SERVICE_LEFT_BTN)) {
            transport::requestNoReply(this->gazeboNode->GetTopicNamespace(), REQUEST_LEFT_BTN, "all");
        }

    }

    private: void resetBot()
    {
        this->model->GetWorld()->GetModel(BOT_MODEL_NAME)->SetWorldPose(botOffsetPose);
    }

    private: void toggleBotIsoControlMode()
    {
        isBotIsoControlEnabled = !isBotIsoControlEnabled;
    }

    private: void toggleBotTfListenerControlMode()
    {
        isBotTfListenerControlEnabled = !isBotTfListenerControlEnabled;
    }

    private: void toggleGravityMode()
    {
        isGravityEnabled = !isGravityEnabled;
        this->model->SetGravityMode(isGravityEnabled);
    }

    private: float computeHoverVelocity(float currVerticalVel)
    {
        if (!isGravityEnabled) {
            if (currBtn[HOVER_DOWN_BTN]) {
                currVerticalVel = -VERTICAL_SPEED;
            } else if (currBtn[HOVER_UP_BTN]) {
                currVerticalVel = VERTICAL_SPEED;
            } else if (currBtn[HOVER_STEADY_BTN]) {
                currVerticalVel = 0;
            }
        }

        return currVerticalVel;
    }

    private: void toggleCollisionMode()
    {
        isCollisionEnabled = !isCollisionEnabled;

        if (isCollisionEnabled) {
            this->bodyLink->SetCollideMode("all");
        } else {
            this->bodyLink->SetCollideMode("none");
        }

    }

    private: void toggleXrayMode()
    {
        isXrayVisionEnabled = !isXrayVisionEnabled;

        if (isXrayVisionEnabled) {
            transport::requestNoReply(this->gazeboNode->GetTopicNamespace(), "set_transparent", "all");
        } else {
            transport::requestNoReply(this->gazeboNode->GetTopicNamespace(), "set_opaque", "all");
        }
    }

    private: bool wasActivated(uint btnRef)
    {
        // Register 'state changes' only
        if (prevBtn[btnRef] != currBtn[btnRef] && currBtn[btnRef] == true) {
            return true;
        }

        return false;
    }

    private: void updateFpvVel()
    {	
    	math::Vector3 currLinearVel = this->bodyLink->GetRelativeLinearVel();
        math::Vector3 newLinearVel = computeVelocities(currLinearVel);

        currLinearVel = constrainVerticalMovement(currLinearVel);
        currLinearVel.z = computeHoverVelocity(currLinearVel.z);

    	this->bodyLink->SetLinearVel(math::Vector3(newLinearVel.x, newLinearVel.y, currLinearVel.z));
    }

    private: math::Vector3 constrainVerticalMovement(math::Vector3 currLinearVel)
    { 

        if (isStrayVerticalVel(currLinearVel)) {
            currLinearVel.z = 0.0;
        }

        return currLinearVel;
    }

    private: bool isStrayVerticalVel(math::Vector3 currLinearVel)
    {
        return (currLinearVel.z > 0.0 && (fabs(currLinearVel.x) > (currBtn[LEFT_REAR_TAP] ? RUNNING_SPEED_X/2 : WALKING_SPEED_X/2) || fabs(currLinearVel.y) > (currBtn[LEFT_REAR_TAP] ? RUNNING_SPEED_Y/2 : WALKING_SPEED_Y/2))) && isGravityEnabled;
    }

    private: math::Vector3 computeVelocities(math::Vector3 currLinearVel)
    {
        // Gazebo's coordinate system is flipped:
        cmd_linear_vel.x = stickLeftY * (currBtn[LEFT_REAR_TAP] ? RUNNING_SPEED_X : WALKING_SPEED_X); 
        cmd_linear_vel.y = -stickLeftX * (currBtn[LEFT_REAR_TAP] ? RUNNING_SPEED_Y : WALKING_SPEED_Y);
        cmd_linear_vel.z = 0;

        // Rotate velocity vector according to the head orientation:
        math::Vector3 newLinearVel = headOrientation.RotateVector(cmd_linear_vel);
        newLinearVel.z = 0;

        return newLinearVel;
    }

    private: void stabilize()
    {
    	math::Pose currPose = this->model->GetWorldPose();
    	math::Vector3 currPosition = currPose.pos;

    	math::Pose stabilizedPose;
    	stabilizedPose.pos = currPosition;

        if (!isGravityEnabled && currPosition.z > CEILING_HEIGHT) {
            stabilizedPose.pos.z = CEILING_HEIGHT;
        } else if (!isCollisionEnabled && currPosition.z < FLOOR_HEIGHT) {
            stabilizedPose.pos.z = FLOOR_HEIGHT;
        }

    	stabilizedPose.rot.x = 0;
    	stabilizedPose.rot.y = 0;

    	stabilizedPose.rot.z = currPose.rot.z;

    	this->model->SetWorldPose(stabilizedPose);    
    }  

    private: void updateBtnStates(const sensor_msgs::Joy::ConstPtr& msg)
    {

        prevBtn[SELECT] = currBtn[SELECT];
        currBtn[SELECT] = msg->buttons[SELECT];

        prevBtn[LEFT_JOY] = currBtn[LEFT_JOY];
        currBtn[LEFT_JOY] = msg->buttons[LEFT_JOY];

        prevBtn[RIGHT_JOY] = currBtn[RIGHT_JOY];
        currBtn[RIGHT_JOY] = msg->buttons[RIGHT_JOY];

        prevBtn[START] = currBtn[START];
        currBtn[START] = msg->buttons[START];

        prevBtn[UP] = currBtn[UP];
        currBtn[UP] = msg->buttons[UP];

        prevBtn[RIGHT] = currBtn[RIGHT];
        currBtn[RIGHT] = msg->buttons[RIGHT];
        
        prevBtn[DOWN] = currBtn[DOWN];
        currBtn[DOWN] = msg->buttons[DOWN];

        prevBtn[LEFT] = currBtn[LEFT];
        currBtn[LEFT] = msg->buttons[LEFT];

        prevBtn[LEFT_REAR_TAP] = currBtn[LEFT_REAR_TAP];
        currBtn[LEFT_REAR_TAP] = msg->buttons[LEFT_REAR_TAP];

        prevBtn[RIGHT_REAR_TAP] = currBtn[RIGHT_REAR_TAP];
        currBtn[RIGHT_REAR_TAP] = msg->buttons[RIGHT_REAR_TAP];

        prevBtn[LEFT_FORWARD_TAP] = currBtn[LEFT_FORWARD_TAP];
        currBtn[LEFT_FORWARD_TAP] = msg->buttons[LEFT_FORWARD_TAP];

        prevBtn[RIGHT_FORWARD_TAP] = currBtn[RIGHT_FORWARD_TAP];
        currBtn[RIGHT_FORWARD_TAP] = msg->buttons[RIGHT_FORWARD_TAP];

        prevBtn[TRIANGLE] = currBtn[TRIANGLE];
        currBtn[TRIANGLE] = msg->buttons[TRIANGLE];

        prevBtn[CIRCLE] = currBtn[CIRCLE];
        currBtn[CIRCLE] = msg->buttons[CIRCLE];

        prevBtn[CROSS] = currBtn[CROSS];
        currBtn[CROSS] = msg->buttons[CROSS];

        prevBtn[SQUARE] = currBtn[SQUARE];
        currBtn[SQUARE] = msg->buttons[SQUARE];

    }     

    private: void ROSCallbackJoy(const sensor_msgs::Joy::ConstPtr& msg)
    {
        // Get joystick states:
        stickLeftX = msg->axes[0];
        stickLeftY = msg->axes[1];
        stickRightX = msg->axes[2];
        stickRightY = msg->axes[3];

        updateBtnStates(msg);
        updateToggleStates();
    }

    private: physics::ModelPtr model;
    private: physics::LinkPtr bodyLink;

    // private: rendering::OculusCameraPtr oculusCamera;

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