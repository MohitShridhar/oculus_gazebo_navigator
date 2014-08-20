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

#define WALKING_SPEED 5  // in m/s
#define RUNNING_SPEED 8
#define ROTATING_SPEED 1.57

#define SAMPLING_DURATION 40000

class CameraController

{
private:

  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_, cmd_request_pub_;

  geometry_msgs::Twist base_cmd;
  std_msgs::String base_request;

  std::clock_t start;
  double duration;

public:

  CameraController(ros::NodeHandle &nh)
  {
    nh_ = nh;
    this->cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/camera_controller/twist", 10);
    this->cmd_request_pub_ = nh_.advertise<std_msgs::String>("/camera_controller/request", 10);
  }

  void printControls()
  {
    printf("Instructions:\n'w' forward, 's' back, 'a' left, 'd' right\n'z' rotate leftwards, 'c' rotate rightwards\nTo run, hold 'shift' first and then press the corresponding buttons\n'Q' to quit (i.e. Shift + 'q')\n\n");
  }

  void driveKeyboard()
  {
    printControls();
    activateInterface();   
  }

  void activateInterface()
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
              base_cmd.linear.x = RUNNING_SPEED;
              printf(" FAST FORWARD\n");
              break;
            case 'w':
              printf(" FORWARD\n");
              base_cmd.linear.x = WALKING_SPEED;
              break;
            case 'A':
              base_cmd.linear.y = RUNNING_SPEED;
              printf(" FAST LEFT\n");
              break;
            case 'a':
              printf(" LEFT\n");
              base_cmd.linear.y = WALKING_SPEED;
              break;
            case 'S':
              printf(" FAST BACKWARD\n");
              base_cmd.linear.x = -RUNNING_SPEED;
              break;
            case 's':
              printf(" BACK\n");
              base_cmd.linear.x = -WALKING_SPEED;
              break;
            case 'D':
              printf(" FAST RIGHT\n");
              base_cmd.linear.y = -RUNNING_SPEED;
              break;
            case 'd':
              printf(" RIGHT\n");
              base_cmd.linear.y = -WALKING_SPEED;
              break;
        }

      } else {
        stopMotion();
      }

      usleep(SAMPLING_DURATION);

      cmd_vel_pub_.publish(base_cmd);
      cmd_request_pub_.publish(base_request);


    }while (cmd != 'Q' && nh_.ok());
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

  ros::init(argc, argv, "camera_driver");
  ros::NodeHandle nh;

  CameraController driver(nh);
  driver.driveKeyboard();
}