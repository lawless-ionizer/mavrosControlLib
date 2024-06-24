#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/duration.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/trajectory.h>
#include <mavros_msgs/PositionTarget.h>


ros::Publisher setPosition;
ros::Publisher setVelocity;
ros::Publisher setAccel;
ros::Publisher waypointTraj;
ros::Publisher bezierTraj;
ros::Subscriber currState;
ros::ServiceClient mavMode;
ros::ServiceClient mavArming;
ros::ServiceClient mavFrame;
ros::ServiceClient takeoff;
ros::ServiceClient land;

typedef struct waypoint
{
    float x;
    float y;
    float z;
    float psi;
};

typedef struct bezier
{
    waypoint pos;
    float32 time;
};

void setMode(string mode = "GUIDED")

void setFrame(string frame = "LOCAL_NED")

void arm()

void cmdTakeoff(float alti)

void cmdLand()

bool checkPosTgt(float linTol, float orientTol)

void gotoPos(waypoint Target)

bool checkVelTgt(float velTol)

void attainVel(float vx, float vy, float vz)

bool checkAccelTgt(float AccelTol)

void attainAccel(float ax, float ay, float az)

void createWaypointTraj(std::vector<waypoint> setWaypoints)

void createBezierTraj(st::vector<bezier> setBezier)


int initControler(ros::NodeHandle controller)
{
    std::string ros_namespace;
	if (!controlnode.hasParam("namespace"))
	{

		ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}
}