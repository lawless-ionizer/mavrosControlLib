#include <iostream>
#include <math.h>
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
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/trajectory.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>


ros::Publisher setPosition;
ros::Publisher setVelocity;
ros::Publisher setAccel;
ros::Publisher waypointTraj;
ros::Publisher bezierTraj;
ros::Subscriber currState;
ros::Subscriber currPose;
ros::Subscriber AltInfo;
ros::ServiceClient mavMode;
ros::ServiceClient mavArming;
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

waypoint transformPosetoWay(geometry_msgs::Pose PoseVal)
{

}

bool setMode(std::string mode = "GUIDED")
{
	mavros_msgs::SetMode srvMode;

    srvMode.request.base_mode = 0;
    srvMode.request.custom_mode = mode.c_str();

    if(mavMode.call(srvMode))
    {
      ROS_INFO("Requested mode is set.");
	  return true;
    }
    else
    {
      ROS_ERROR("Failed to set requested mode.");
      return false;
    }
}

bool arm()
{
    ROS_INFO("Arming drone");

	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;

	while (!currState.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		currState.call(arm_request);
	}

	if(arm_request.response.success)
	{
		ROS_INFO("Arming Successful");	
		return true;
	}
    else
    {
		ROS_INFO("Arming failed with %d", arm_request.response.success);
		return false;
	}
}

void cmdLand()

bool checkPosTgt(float linTol = 0.5, float orientTol = 0.1)

void gotoPos(waypoint Target)

void cmdTakeoff(float alti)
{
    mavros_msgs::CommandTOLLocal takeoffPoint;

    takeoffPoint.request.Position.z = alti;

    while(!takeoff_request.response.success && ros::ok())
    {
        ros::Duration(.1).sleep();
        takeoff.call(takeoff_request);
    }

    if(abs(AltInfo - alti) < 0.5)
}

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