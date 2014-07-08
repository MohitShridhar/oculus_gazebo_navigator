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

#include "oculus_ps3_teleop_plugin.h"

namespace gazebo
{   

    OculusGazeboNavigator::OculusGazeboNavigator()
    {
      std::string name = "oculus_gazebo_navigator";
      int argc = 0;
      ros::init(argc, NULL, name);

    }

    OculusGazeboNavigator::~OculusGazeboNavigator()
    {
      delete this->rosNode;
      transport::fini();
    }

    void OculusGazeboNavigator::Load(physics::ModelPtr _parent, sdf::ElementPtr)
    {
        establishLinks(_parent);
        setupHMDSubscription();
        initVars();
    }

    void OculusGazeboNavigator::initVars()
    {
        this->isGravityEnabled = true;
        this->isCollisionEnabled = true;
        this->isXrayVisionEnabled = false;
        this->isBotIsoControlEnabled = false;
        this->isBotTfListenerControlEnabled = true;

        this->botOffsetPose = this->model->GetWorld()->GetModel(BOT_MODEL_NAME)->GetWorldPose();

        ros::Rate rate(ROS_RATE);
    }

    void OculusGazeboNavigator::setupHMDSubscription()
    {
        this->gazeboNode = transport::NodePtr(new transport::Node());
        this->gazeboNode->Init();

        this->hmdSub = this->gazeboNode->Subscribe("~/oculusHMD", &OculusGazeboNavigator::GzHMDCallback, this);
        this->pubFpvPose = this->gazeboNode->Advertise<msgs::Pose>("~/oculus/fpvPose");

    }

    void OculusGazeboNavigator::GzHMDCallback(QuaternionPtr &msg)
    {
        headOrientation = msgs::Convert(*msg);
    }

    void OculusGazeboNavigator::establishLinks(physics::ModelPtr _parent)
    {
        this->model = _parent;
        this->bodyLink = this->model->GetLink("body");

        this->rosNode = new ros::NodeHandle("/joy0");
        this->sub_twist = this->rosNode->subscribe<sensor_msgs::Joy>("/joy0", 50, &OculusGazeboNavigator::ROSCallbackJoy, this);
        this->pub_cmd_vel = this->rosNode->advertise<geometry_msgs::Twist>(BOT_CMD_TOPIC_NAME, 50);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&OculusGazeboNavigator::OnUpdate, this));
    }

    void OculusGazeboNavigator::OnUpdate()
    {
        ros::spinOnce();
        updateVel();
        stabilize();
    }

    void OculusGazeboNavigator::updateVel()
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
    void OculusGazeboNavigator::updateBotVel()
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
    void OculusGazeboNavigator::updateBotVelIsolated()
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
    void OculusGazeboNavigator::updateBotVelListener()
    {
        bool wasLookUpSuccessful = lookupTfListener();

        if (!wasLookUpSuccessful) {
            return;
        } 

        math::Pose newBotPose = transformTo3DOFPoseRotated(transform);

        newBotPose.pos.x -= botOffsetPose.pos.x;
        newBotPose.pos.y -= botOffsetPose.pos.y;
        newBotPose.rot.z -= botOffsetPose.rot.z;

        this->model->GetWorld()->GetModel(BOT_MODEL_NAME)->SetWorldPose(newBotPose);
    }

    void OculusGazeboNavigator::calibrateBot()
    {
        bool wasLookUpSuccessful = lookupTfListener();

        if (!wasLookUpSuccessful) {
            printf("Oculus Navigator: CALIBRATION FAILED - Could not get TF data\n");
            return;
        }

        math::Pose realBotPose = transformTo3DOFPoseRotated(transform);
        math::Pose virtualBotPose = this->model->GetWorld()->GetModel(BOT_MODEL_NAME)->GetWorldPose();

        botOffsetPose.pos.x = realBotPose.pos.x - virtualBotPose.pos.x;
        botOffsetPose.pos.y = realBotPose.pos.y - virtualBotPose.pos.y;
        botOffsetPose.rot.z = realBotPose.rot.z - virtualBotPose.rot.z;

        printf("Oculus Navigator: New bot-offset pose – x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f\n-----------------\n", botOffsetPose.pos.x, botOffsetPose.pos.y, botOffsetPose.pos.z, botOffsetPose.rot.x, botOffsetPose.rot.y, botOffsetPose.rot.z);            
    }   

    math::Pose OculusGazeboNavigator::transformTo3DOFPoseRotated(tf::StampedTransform trans)
    {
        tf::Quaternion quatTf = trans.getRotation();
        math::Quaternion quatGz = tfToGzQuat(quatTf);

        double roll, pitch, yaw;
        tf::Matrix3x3(quatTf).getRPY(roll, pitch, yaw);

        math::Pose newPose;

        newPose.pos.x = trans.getOrigin().x();
        newPose.pos.y = trans.getOrigin().y();
        newPose.rot.z = yaw;

        // Rotate translation vector according to yaw:
        // newPose.pos = quatGz.RotateVector(newPose.pos);
        newPose.pos.z = botOffsetPose.pos.z;

        return newPose;        
    }

    math::Quaternion OculusGazeboNavigator::tfToGzQuat(tf::Quaternion quatTf)
    {
        math::Quaternion quatGz;

        quatGz.x = quatTf.x();
        quatGz.y = quatTf.y();
        quatGz.z = quatTf.z();
        quatGz.w = quatTf.w();

        return quatGz;
    }

    bool OculusGazeboNavigator::lookupTfListener()
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

    void OculusGazeboNavigator::updateToggleStates()
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

    void OculusGazeboNavigator::resetBot()
    {
        this->model->GetWorld()->GetModel(BOT_MODEL_NAME)->SetWorldPose(botOffsetPose);
    }

    void OculusGazeboNavigator::toggleBotIsoControlMode()
    {
        isBotIsoControlEnabled = !isBotIsoControlEnabled;
    }

    void OculusGazeboNavigator::toggleBotTfListenerControlMode()
    {
        isBotTfListenerControlEnabled = !isBotTfListenerControlEnabled;
    }

    void OculusGazeboNavigator::toggleGravityMode()
    {
        isGravityEnabled = !isGravityEnabled;
        this->model->SetGravityMode(isGravityEnabled);
    }

    float OculusGazeboNavigator::computeHoverVelocity(float currVerticalVel)
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

    void OculusGazeboNavigator::toggleCollisionMode()
    {
        isCollisionEnabled = !isCollisionEnabled;

        if (isCollisionEnabled) {
            this->bodyLink->SetCollideMode("all");
        } else {
            this->bodyLink->SetCollideMode("none");
        }

    }

    void OculusGazeboNavigator::toggleXrayMode()
    {
        isXrayVisionEnabled = !isXrayVisionEnabled;

        if (isXrayVisionEnabled) {
            transport::requestNoReply(this->gazeboNode->GetTopicNamespace(), "set_transparent", "all");
        } else {
            transport::requestNoReply(this->gazeboNode->GetTopicNamespace(), "set_opaque", "all");
        }
    }

    bool OculusGazeboNavigator::wasActivated(uint btnRef)
    {
        // Register 'state changes' only
        if (prevBtn[btnRef] != currBtn[btnRef] && currBtn[btnRef] == true) {
            return true;
        }

        return false;
    }

    void OculusGazeboNavigator::updateFpvVel()
    {   
        math::Vector3 currLinearVel = this->bodyLink->GetRelativeLinearVel();
        math::Vector3 newLinearVel = computeVelocities(currLinearVel);

        currLinearVel = constrainVerticalMovement(currLinearVel);
        currLinearVel.z = computeHoverVelocity(currLinearVel.z);

        this->bodyLink->SetLinearVel(math::Vector3(newLinearVel.x, newLinearVel.y, currLinearVel.z));
    }

    math::Vector3 OculusGazeboNavigator::constrainVerticalMovement(math::Vector3 currLinearVel)
    { 

        if (isStrayVerticalVel(currLinearVel)) {
            currLinearVel.z = 0.0;
        }

        return currLinearVel;
    }

    bool OculusGazeboNavigator::isStrayVerticalVel(math::Vector3 currLinearVel)
    {
        return (currLinearVel.z > 0.0 && (fabs(currLinearVel.x) > (currBtn[LEFT_REAR_TAP] ? RUNNING_SPEED_X/2 : WALKING_SPEED_X/2) || fabs(currLinearVel.y) > (currBtn[LEFT_REAR_TAP] ? RUNNING_SPEED_Y/2 : WALKING_SPEED_Y/2))) && isGravityEnabled;
    }

    math::Vector3 OculusGazeboNavigator::computeVelocities(math::Vector3 currLinearVel)
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

    void OculusGazeboNavigator::stabilize()
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

    void OculusGazeboNavigator::updateBtnStates(const sensor_msgs::Joy::ConstPtr& msg)
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

    void OculusGazeboNavigator::ROSCallbackJoy(const sensor_msgs::Joy::ConstPtr& msg)
    {
        // Get joystick states:
        stickLeftX = msg->axes[0];
        stickLeftY = msg->axes[1];
        stickRightX = msg->axes[2];
        stickRightY = msg->axes[3];

        updateBtnStates(msg);
        updateToggleStates();
    }

}