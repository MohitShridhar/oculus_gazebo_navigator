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

    void OculusGazeboNavigator::parseParams()
    {
        this->rosNode->getParam("navigator_bot_name", botName);
        this->topicCmdVel = "/" + botName + "/cmd_vel";
    }

    void OculusGazeboNavigator::initVars()
    {
        this->isGravityEnabled = true;
        this->isCollisionEnabled = true;
        this->isXrayVisionEnabled = false;
        this->isBotIsoControlEnabled = false;
        this->isBotTfListenerControlEnabled = false;
        this->isNavStackControlEnabled = true;

        this->botOffsetPose = this->model->GetWorld()->GetModel(botName)->GetWorldPose();

        ros::Rate rate(ROS_RATE);
    }

    void OculusGazeboNavigator::setupHMDSubscription()
    {
        this->gazeboNode = transport::NodePtr(new transport::Node());
        this->gazeboNode->Init();

        this->hmdSub = this->gazeboNode->Subscribe("~/oculusHMD", &OculusGazeboNavigator::GzHMDCallback, this);
        this->pubFpvPose = this->gazeboNode->Advertise<msgs::Pose>("~/oculus/fpvPose");

    }

    void OculusGazeboNavigator::GzHMDCallback(const boost::shared_ptr<const msgs::Quaternion> &msg)
    {
        headOrientation = msgs::Convert(*msg);
    }

    void OculusGazeboNavigator::establishLinks(physics::ModelPtr _parent)
    {
        this->model = _parent;
        this->bodyLink = this->model->GetLink("body");

        this->rosNode = new ros::NodeHandle("");
        parseParams();

        this->sub_twist = this->rosNode->subscribe<sensor_msgs::Joy>("/joy0", 50, &OculusGazeboNavigator::ROSCallbackJoy, this);
        this->sub_origin_offset = this->rosNode->subscribe<geometry_msgs::PoseWithCovarianceStamped>("/gazebo_offset", 2, &OculusGazeboNavigator::CallbackOriginOffset, this);
        this->pub_cmd_vel = this->rosNode->advertise<geometry_msgs::Twist>(this->topicCmdVel, 50);
        
        this->cancel_goal_pub_ = this->rosNode->advertise<actionlib_msgs::GoalID>("/" + botName + "/move_base/cancel", 1);

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
        // Navigation stack - Bot control:
        if (this->isNavStackControlEnabled) {
            updateFpvVel();
            return;
        }

        //Bot control - TF Listener:
        if (this->isBotTfListenerControlEnabled) {
            updateFpvVel();
            updateBotVelListener();
            return;
        }

        //Bot control - Joystick:
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

    // TF Listener Mode: Bot World-Pose controlled directly via TF
    void OculusGazeboNavigator::updateBotVelListener()
    {
        bool wasLookUpSuccessful = lookupTfListener();

        if (!wasLookUpSuccessful) {
            ROS_ERROR("Failed to receive data from TF Listener\n");
            return;
        } 

        math::Pose tfRawPose = transformTo3DOFPose(transform);
        math::Pose gzRotatedPose = tfRawPose.RotatePositionAboutOrigin(this->botOffsetQuat);

        gzRotatedPose.pos.x -= this->botOffsetPose.pos.x;
        gzRotatedPose.pos.y -= this->botOffsetPose.pos.y;
        gzRotatedPose.pos.z = BOT_FIXED_Z_POS;
        
        gzRotatedPose.rot = math::Quaternion(0, 0, tfRawPose.rot.GetYaw() + botOffsetQuat.GetYaw());

        this->model->SetWorldPose(gzRotatedPose);

    }

    void OculusGazeboNavigator::calibrateBot()
    {
        this->botOffsetQuat = tfToGzQuat(transform.getRotation()).GetInverse();
        math::Pose tfOriginPose = transformTo3DOFPose(transform);

        this->botOffsetPose = tfOriginPose.RotatePositionAboutOrigin(this->botOffsetQuat);
        this->botOffsetPose.rot = tfOriginPose.rot;         
    }

    math::Pose OculusGazeboNavigator::transformTo3DOFPose(tf::StampedTransform trans)
    {
        tf::Quaternion quatTf = trans.getRotation();
        math::Quaternion quatGz = tfToGzQuat(quatTf);

        double roll, pitch, yaw;
        tf::Matrix3x3(quatTf).getRPY(roll, pitch, yaw);

        math::Pose newPose;

        newPose.pos.x = trans.getOrigin().x();
        newPose.pos.y = trans.getOrigin().y();
        newPose.rot = math::Quaternion(0.0, 0.0, yaw);

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

        // Toggle Navigation-Stack control mode:
        if (wasActivated(BOT_CONTROL_BTN) && !currBtn[LEFT_REAR_TAP] && !currBtn[RIGHT_REAR_TAP]) {
            toggleNavStackControlMode();
            printf("Oculus Navigator: Navigation Stack control = %s\n", (isNavStackControlEnabled ? "enabled" : "disabled"));
        }

        // Toggle TF-Listener mode & Joystick bot-control:

        if (wasActivated(BOT_CONTROL_BTN) && currBtn[RIGHT_REAR_TAP] && !currBtn[LEFT_REAR_TAP]) {
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

        if (wasActivated(BOT_CONTROL_BTN) && currBtn[LEFT_REAR_TAP] && !currBtn[RIGHT_REAR_TAP]) {
            toggleBotIsoControlMode();
            printf("Oculus Navigator: Bot-Control mode %s, Fpv Navigator %s\n", (isBotIsoControlEnabled ? "enabled" : "disabled"), (isBotIsoControlEnabled ? "disabled" : "enabled"));
        }

        if (wasActivated(BOT_RESET_BTN) && !currBtn[LEFT_REAR_TAP] && !currBtn[RIGHT_REAR_TAP]) {
            resetBot();
            printf("Oculus Navigator: '%s' reset to offset pose\n", botName.c_str());
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
        this->model->GetWorld()->GetModel(botName)->SetWorldPose(botOffsetPose);
    }

    void OculusGazeboNavigator::toggleBotIsoControlMode()
    {
        isBotIsoControlEnabled = !isBotIsoControlEnabled;
        isBotTfListenerControlEnabled = false;
        isNavStackControlEnabled = false;
    }

    void OculusGazeboNavigator::toggleNavStackControlMode()
    {
        isNavStackControlEnabled = !isNavStackControlEnabled;
        isBotIsoControlEnabled = false;
        isBotTfListenerControlEnabled = false;

        if (!isNavStackControlEnabled) {
            actionlib_msgs::GoalID emptyReq;
            this->cancel_goal_pub_.publish(emptyReq);
        }
    }

    void OculusGazeboNavigator::toggleBotTfListenerControlMode()
    {
        isBotTfListenerControlEnabled = !isBotTfListenerControlEnabled;
        isNavStackControlEnabled = false;
        isBotIsoControlEnabled = false;
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
            transport::requestNoReply(this->gazeboNode->GetTopicNamespace(), "set_transparent", "world");
        } else {
            transport::requestNoReply(this->gazeboNode->GetTopicNamespace(), "set_opaque", "world");
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

        stabilizedPose.rot.x = 0.0;
        stabilizedPose.rot.y = 0.0;

        // stabilizedPose.rot.z = currPose.rot.z;
        // stabilizedPose.rot.w = currPose.rot.w; 
        stabilizedPose.rot.z = 0.0;

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

    void OculusGazeboNavigator::CallbackOriginOffset(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        geometry_msgs::Pose originPose = msg->pose.pose;

        transform.setOrigin(tf::Vector3(originPose.position.x, originPose.position.y, originPose.position.z));
        transform.setRotation(tf::Quaternion(originPose.orientation.x, originPose.orientation.y, originPose.orientation.z, originPose.orientation.w));

        calibrateBot();
        printf("Initial TF Origin pose - POS: %f %f %f\n ROT: %f %f %f %f\n", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
    }

}