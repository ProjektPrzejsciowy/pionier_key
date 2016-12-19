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
#include <string>
#include <iostream>
#include "../inc/keyboard.h"
#include <SDL/SDL.h>
using namespace std;

char c = 0;
static struct termios old, new2;
void initTermios(int echo);
void resetTermios(void);
char getch_(int echo);
char getch(void);
char getche(void);

namespace patch
{
    template <typename T> std::string to_string(const T& str)
    {
        std::ostringstream stm;
        stm << str;
        return stm.str();
    }
}

int main(int argc, char **argv)
{
	string robotName = "pioneer2dx";
	//string robotName = "pioneer2dx_with_sensors";
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/" + robotName + "/cmd_vel", 10);

  gazebo_msgs::GetModelState getmodelstate;
	getmodelstate.request.model_name = robotName;
	ros::ServiceClient gms_c = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	//getmodelstate.model_name="pioneer2dx";
	

  //gms_c.call("pioneer2dx", "world");
	
	/*if (gms_c.call(getmodelstate))
	{
		puts("get_model_state  service call  failed");
	}
	if (gms_c)
	{
		puts("get_model_state not found");
	}*/
	gazebo_msgs::ModelState modelState;
	
	geometry_msgs::Pose pose;
	geometry_msgs::Twist twist;

	geometry_msgs::Pose poseCURR;
	geometry_msgs::Twist twistCURR;
  
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = robotName;
	modelstate.pose = poseCURR;
	modelstate.twist = twistCURR;
	setmodelstate.request.model_state=modelstate;         

	/*if (client.call(setmodelstate))
	{
		ROS_INFO("Connected!!");
	}
	else
	{
		ROS_ERROR("Failed to call service for setmodelstate ");
		return 1;
	}*/
		
	bool dirty = false;
	double linSpeed = 1;
	double angSpeed = 2;
	int ang = 0;
	int lin = 0;

  
	puts("Wywrotowiec wita");
  //puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use ARROWS to move the robot");
	puts(" (Q - exit, R - reset do [0, 0], G - remember current state, T - random movement");
	puts(" To modify speed hold for 0.5s (W or S) or (E or D) ofr lin/ang inc/decrease");

	cKeyboard kb;
	if (kb.active == false)
	{
		puts("Run this node as an administrator - try 'sudo su', 'sudo rosrun' doesn't work')");
		return 0;
	}
  initTermios(false);
	while (true)
	{
		ang = 0;
		lin = 0;
		if (kb.getKeyState(KEY_UP) && kb.getKeyState(KEY_LEFT))
		{
			ROS_INFO("UP + LEFT");
			ang = -1;
			lin = 1;
		}
		else if (kb.getKeyState(KEY_UP) && kb.getKeyState(KEY_RIGHT))
		{
			ROS_INFO("UP + RIGHT");
			ang = 1;
			lin = 1;
		}
		else if (kb.getKeyState(KEY_DOWN) && kb.getKeyState(KEY_LEFT))
		{
			ROS_INFO("DOWN + LEFT");
			ang = -1;
			lin = -1;
		}
		else if (kb.getKeyState(KEY_DOWN) && kb.getKeyState(KEY_RIGHT))
		{
			ROS_INFO("DOWN + RIGHT");
			ang = 1;
			lin = -1;
		}
		else if (kb.getKeyState(KEY_UP))
		{
			ROS_INFO("UP");
			lin = 1;
		}
		else if (kb.getKeyState(KEY_DOWN))
		{
			ROS_INFO("DOWN");
			lin = -1;
		}
		else if (kb.getKeyState(KEY_LEFT))
		{
			ROS_INFO("LEFT");
			ang = -1;
		}
		else if (kb.getKeyState(KEY_RIGHT))
		{
			ROS_INFO("RIGHT");
			ang = 1;
		}
		else if (kb.getKeyState(KEY_Q))
		{
			ROS_INFO("SAY GOODBYE!");
			//usleep(200000);
  		resetTermios();
			//usleep(200000);
			return 0;
		}
		else if (kb.getKeyState(KEY_W))
		{
			ROS_INFO("W - hold to increase linear speed");
			usleep(500000);
			if (kb.getKeyState(KEY_W))
			{
				linSpeed += 0.2;
				cout << "Changed linear speed to " << linSpeed << " [nieznanych jednostek :<]" << endl;
			}
		}
		else if (kb.getKeyState(KEY_S))
		{
			ROS_INFO("S - hold to decrease linear speed");
			usleep(500000);
			if (kb.getKeyState(KEY_S))
			{
				linSpeed -= 0.2;
				cout << "Changed linear speed to " << linSpeed << " [nieznanych jednostek :<]" << endl;
			}
		}
		else if (kb.getKeyState(KEY_E))
		{
			ROS_INFO("E - hold to increase angular speed");
			usleep(500000);
			if (kb.getKeyState(KEY_E))
			{
				angSpeed += 0.2;
				cout << "Changed angular speed to " << angSpeed << " [nieznanych jednostek :<]" << endl;
			}
		}
		else if (kb.getKeyState(KEY_D))
		{
			ROS_INFO("D - hold to decrease angular speed");
			usleep(500000);
			if (kb.getKeyState(KEY_D))
			{
				angSpeed -= 0.2;
				cout << "Changed angular speed to " << angSpeed << " [nieznanych jednostek :<]" << endl;
			}
		}
		else if (kb.getKeyState(KEY_R))
		{
			ROS_INFO("R");
			try 
			{
				client.call(setmodelstate);
			}
			catch (...)
			{
				ROS_INFO("Failed to call service for set_model_state");
			}
		}
		else if (kb.getKeyState(KEY_T))
		{
			ROS_INFO("T - brak funkcjonalnosci");
		}
		else if (kb.getKeyState(KEY_G))
		{
			ROS_INFO("G - brak fukcjonalnosci");
		}
		twist.linear.x = linSpeed * lin;
		twist.angular.z = angSpeed * ang;
		chatter_pub.publish(twist);			
		usleep(100000); // 0.1s
	}
  resetTermios();
	usleep(500000);
	return 0;
  
	// Stara obsuga klawiatury, tak zostawiam dla wlasnego uzytku - 
	/*while (true)
  {
		puts("R");
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
			case 'g':
		  case 'G':
				ROS_DEBUG("GET_STATE");
				if (!gms_c.call(getmodelstate))
				{
					ROS_INFO("GET_STATE");

					poseCURR = getmodelstate.response.pose;
					twistCURR = getmodelstate.response.twist;
				}
				else
				{
					ROS_ERROR("Failed to call service for getstatemodel");
					return 1;
				}
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
					
					twist.linear.x = speed * lin;
					twist.angular.z = speed * ang * 5;
					chatter_pub.publish(twist);
					usleep(1000000);

					lin = 1;
					ang = 0;

		      twist.linear.x = speed * lin;
					twist.angular.z = speed * ang * 5;
					chatter_pub.publish(twist);	
					usleep(1000000);	

					lin = 0;
					ang = -1;
					
					twist.linear.x = speed * lin;
					twist.angular.z = speed * ang * 5;
					chatter_pub.publish(twist);
					usleep(500000);

					lin = 1;
					ang = 0;

		      twist.linear.x = speed * lin;
					twist.angular.z = speed * ang * 5;
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
			twist.linear.x = speed * lin;
			twist.angular.z = speed * ang * 5;
			chatter_pub.publish(twist);			

      dirty=false;
    }
  }*/

  return 0;
}

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
