#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h" 
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "gazebo_msgs/ModelState.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sstream>
#include <unistd.h>

char c = 0;

static struct termios old, new2;

/* Initialize new terminal i/o settings */
void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  new2 = old; /* make new settings same as old settings */
  new2.c_lflag &= ~ICANON; /* disable buffered i/o */
  new2.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new2); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void) 
{
  return getch_(1);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/pioneer2dx/cmd_vel", 10);

	gazebo_msgs::ModelState modelState;
	
	geometry_msgs::Pose pose;
	geometry_msgs::Twist twist;
  
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  gazebo_msgs::ModelState modelstate;
  modelstate.model_name ="pioneer2dx";
	modelstate.pose = pose;
	modelstate.twist = twist;
	setmodelstate.request.model_state=modelstate;         

	if (client.call(setmodelstate))
	{
		ROS_INFO("Connected!!");
	}
	else
	{
		ROS_ERROR("Failed to call service for setmodelstate ");
		return 1;
	}
		
	bool dirty = false;
	int skala = 1;
	int ang = 0;
	int lin = 0;

  

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use WASD to move the robot (Q - exit, F - stop)");

  while(1)
  {
		ang = 0;
		lin = 0;
		c=getch();
    switch(c)
    {
      case 'a':
      case 'A':
				ROS_DEBUG("LEFT");        
				ang = -1;
        dirty = true;
        break;
      case 'd':
      case 'D':
        ROS_DEBUG("RIGHT");
        ang = 1;
        dirty = true;
        break;
      case 'w':
      case 'W':
        ROS_DEBUG("UP");
        lin = -1;
        dirty = true;
        break;
      case 's':
      case 'S':
        ROS_DEBUG("DOWN");
        lin = 1;
        dirty = true;
        break;
			case 'f':
      case 'F':
				ROS_DEBUG("STOP");
				dirty = true;
				break;
      case 'r':
      case 'R':
        ROS_DEBUG("RESET");
        dirty = true;
				if (client.call(setmodelstate))
				{
					ROS_INFO("Reset");
				}
				else
				{
					ROS_ERROR("Failed to call service for setmodelstate ");
					return 1;
				}
        break;
			case 't':
			case 'T':
				while(true)
				{
					lin = 0;
					ang = 1;
					
					twist.linear.x = skala * lin;
					twist.angular.z = skala * ang * 5;
					chatter_pub.publish(twist);
					usleep(1000000);

					lin = 1;
					ang = 0;

		      twist.linear.x = skala * lin;
					twist.angular.z = skala * ang * 5;
					chatter_pub.publish(twist);	
					usleep(1000000);	
				}
			case 'q':
      case 'Q':
				ROS_DEBUG("EXIT");	
        return 0;
    }

    if(dirty ==true)
    {
			twist.linear.x = skala * lin;
			twist.angular.z = skala * ang * 5;
			chatter_pub.publish(twist);			

      dirty=false;
    }
  }

  return 0;
}
