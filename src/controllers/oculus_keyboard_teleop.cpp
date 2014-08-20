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

#include <unistd.h> 
#include <termios.h>

#include <stdio.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#define DEFAULT_WALKING_SPEED 5  // in m/s
#define DEFAULT_RUNNING_SPEED 8
#define DEFAULT_ROTATING_SPEED 1.57

#define DEFAULT_SAMPLING_DURATION 40000

class CameraController

{
private:

  ros::NodeHandle rosNode;
  ros::Publisher cmd_vel_pub_, cmd_request_pub_;

  geometry_msgs::Twist base_cmd;
  std_msgs::String base_request;

  double walking_speed, running_speed, rotating_speed;
  int sampling_duration;

  std::clock_t start;
  double duration;

public:

  CameraController(ros::NodeHandle &nh)
  {
    rosNode = nh;

    rosNode = ros::NodeHandle("");
    loadParams();

    cmd_vel_pub_ = rosNode.advertise<geometry_msgs::Twist>("/camera_controller/twist", 10);
    cmd_request_pub_ = rosNode.advertise<std_msgs::String>("/camera_controller/request", 10);
  }

  void loadParams()
  {
    rosNode.param<double>("/oculus_gazebo_navigator/keyboard_teleop/camera/rotating_speed", rotating_speed, DEFAULT_ROTATING_SPEED);
    rosNode.param<double>("/oculus_gazebo_navigator/keyboard_teleop/camera/running_speed", running_speed, DEFAULT_RUNNING_SPEED);
    rosNode.param<double>("/oculus_gazebo_navigator/keyboard_teleop/camera/walking_speed", walking_speed, DEFAULT_WALKING_SPEED);

    rosNode.param<int>("/oculus_gazebo_navigator/keyboard_teleop/others/sampling_duration", sampling_duration, DEFAULT_SAMPLING_DURATION);
  }

  void printControls()
  {
    printf("Instructions:\n'w' forward, 's' back, 'a' left, 'd' right\n'z' rotate leftwards, 'c' rotate rightwards\nTo run, hold 'shift' first and then press the corresponding buttons\n'Q' to quit (i.e. Shift + 'q')\n\n");
  }

  void startInterface()
  {
    printControls();
    activate();   
  }

  void activate()
  { 
    char cmd = 0;

    do
    {

      if (kbhit()) {

        cmd = getch();
        std::cin.sync();

        switch(cmd)
        {
            case 'W':
              base_cmd.linear.x = running_speed;
              printf(" FAST FORWARD\n");
              break;
            case 'w':
              printf(" FORWARD\n");
              base_cmd.linear.x = walking_speed;
              break;
            case 'A':
              base_cmd.linear.y = running_speed;
              printf(" FAST LEFT\n");
              break;
            case 'a':
              printf(" LEFT\n");
              base_cmd.linear.y = walking_speed;
              break;
            case 'S':
              printf(" FAST BACKWARD\n");
              base_cmd.linear.x = -running_speed;
              break;
            case 's':
              printf(" BACK\n");
              base_cmd.linear.x = -walking_speed;
              break;
            case 'D':
              printf(" FAST RIGHT\n");
              base_cmd.linear.y = -running_speed;
              break;
            case 'd':
              printf(" RIGHT\n");
              base_cmd.linear.y = -walking_speed;
              break;
        }

      } else {
        stopMotion();
      }

      usleep(sampling_duration);

      cmd_vel_pub_.publish(base_cmd);
      cmd_request_pub_.publish(base_request);


    }while (cmd != 'Q' && rosNode.ok());
  }


  void stopMotion()
  {
    base_cmd.linear.x = 0;
    base_cmd.linear.y = 0;
    base_cmd.linear.z = 0;

    base_cmd.angular.x = 0;
    base_cmd.angular.y = 0;
    base_cmd.angular.z = 0;    
  }

  /* Input scanning functions: */

  int getch(void)
  {
    int ch;

    struct termios oldt;
    struct termios newt;

    tcgetattr(STDIN_FILENO, &oldt);
    
    newt = oldt; 
    newt.c_lflag &= ~(ICANON | ECHO);
    
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); 
    
    ch = getchar(); 
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch; 
  }

  int kbhit(void)
  {
    struct termios oldt, newt;
    int ch;
    int oldf;
   
    tcgetattr(STDIN_FILENO, &oldt);

    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
   
    ch = getchar();
   
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
   
    if(ch != EOF)
    {
      ungetc(ch, stdin);
      return 1;
    }
   
    return 0;
  }

};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "oculus_gazebo_navigator_keyboard_control");
  ros::NodeHandle nh;

  CameraController controller(nh);
  controller.startInterface();
}