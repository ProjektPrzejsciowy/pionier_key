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
	//string robotName = "pioneer_1/RosAria";
	string robotName = "pioneer2dx_with_sensors";
	ros::init(argc, argv, "randomCircle");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/" + robotName + "/cmd_vel", 10);

	geometry_msgs::Pose pose;
	geometry_msgs::Twist twist;
	
	puts("randomCircle");
	//puts("Reading from keyboard");
	puts("---------------------------");
	puts("Press 's' to start, press 'q' to exit [and stop]");
    c = getch();
    if (c == 's') 
    {
        puts("Robot powinien jezdzic wlasnie...");
    }
    else 
    {
        puts("Daj mu szanse nastepnym razem :(");
        // resetTermios();
        return 0;
    }	
	twist.linear.x = 0.2;
	twist.angular.z = 2;
	chatter_pub.publish(twist);
	while (getch() != 'q') 
    {
        puts("Aby wyjsc musisz nacisnac Q, nie ma tu drogi na skroty mlody adepcie.");
    }
	twist.linear.x = 0;
	twist.angular.z = 0;
	chatter_pub.publish(twist);
    // resetTermios();
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
