#include <iostream>
#include <math.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/duration.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/trajectory.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/PositionTarget.h>


ros::Publisher setPosition;
geometry_msgs::PoseStamped tgtPose;
ros::Publisher setVelocity;
geometry_msgs::TwistStamped tgtVel;
ros::Publisher setAccel;
geometry_msgs::AccelWithCovarianceStamped tgtAccel;
ros::Publisher setTraj;
//ros::Publisher bezierTraj;

ros::Subscriber currState;
mavros_msgs::State presState;
ros::Subscriber currPose;
geometry_msgs::PoseStamped presPose;
ros::Subscriber currVel;
geometry_msgs::TwistStamped preVel;
ros::Subscriber currAccel;
geometry_msgs::AccelWithCovarianceStamped presAccel;

ros::ServiceClient mavMode;
ros::ServiceClient mavArmingmavArming;
ros::ServiceClient takeoff;
ros::ServiceClient land;

float initAlti;



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

void StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    presState = *msg;
}

void PoseCallback(const geomtery_msgs::PoseStamped::ConstPtr& msg)
{
    presPose = *msg;
}

void VelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    presVel = *msg;
}

void AccelCallback(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& msg)
{
    presAccel = *msg;
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

bool checkPosTgt(float linTol = 0.5, float orientTol = 0.1)

void changeHeading(float yaw)
{
    float pitch = 0;
    float roll = 0;

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);

    float qw = cy * cr * cp + sy * sr * sp;
    float qx = cy * sr * cp - sy * cr * sp;
    float qy = cy * cr * sp + sy * sr * cp;
    float qz = sy * cr * cp - cy * sr * sp;

    tgtPose.pose.orientation.w = qw;
    tgtPose.pose.orientation.x = qx;
    tgtPose.pose.orientation.y = qy;
    tgtPose.pose.orientation.z = qz;

    setPosition.publish(tgtPose);
}

void gotoPos(waypoint Target)
{
    changeHeading(Target.psi);

    tgtPose.pose.position.x = Target.x;
    tgtPose.pose.position.y = Target.y;
    tgtPose.pose.position.z = Target.z;
    
    setPosition.publish(tgtPose);

    while(!checkPosTgt)
        ros::Duration(.1).sleep();
}

void cmdTakeoff(float alti)
{
    mavros_msgs::CommandTOL takeoffPoint;

    takeoffPoint.request.Altitude = alti;

    while(!takeoff_request.response.success && ros::ok())
    {
        ros::Duration(.1).sleep();
        takeoff.call(takeoff_request);
    }

    if(abs(AltInfo - alti) < 0.5)
        ROS_INFO("Success!");
    else
        ROS_INFO("Did not reach target.");
}

void cmdLand()
{
    cmdTakeoff(float initAlti);
}

bool checkVelTgt(float velTol)

void attainVel(float vx, float vy, float vz)

bool checkAccelTgt(float AccelTol)

void attainAccel(float ax, float ay, float az)

void createTraj(std::vector<waypoint> setWaypoints)

//void createBezierTraj(st::vector<bezier> setBezier)


int initControler(ros::NodeHandle controller)
{
    std::string ros_namespace;
	if(!controlnode.hasParam("namespace"))
		ROS_INFO("Using default namespace");
    else
    {
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("Using namespace %s", ros_namespace.c_str());
	}

    setPosition = controller.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/setpoint_position/local"),c_str(), 100);
    setVelocity = controller.advertise<geometry_msgs::TwistStamped>((ros_namespace + "/mavros/setpoint_velocity/cmd_vel").c_str(), 100);
    setAccel = controller.advertise<geometry_msgs::Vector3Stamped>((ros_namespace + "/mavros/setpoint_accel/accel").c_str(), 100);
    setTraj = controller.advertise<mavros_msgs::Trajectory>((ros_namespace + "/mavros/trajectory/generated".c_str()), 100);

    currState = controller.subscribe<mavros_msgs::State>((ros_namespace + "/mavros/state").c_str(), 100, StateCallback);
    currPose = controller.subscribe<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/local_position/pose").c_str(), 100, PoseCallback);
    currVel = controller.subscribe<geometry_msgs::TwistStamped>((ros_namespace + "geometry_msgs/TwistStamped").c_str(), 100, VelCallback);
    currAccel = controller.subscribe<geometry_msgs::AccelWithCovarianceStamped>((ros_namespace + "/mavros/local_position/accel").c_str(), 100, AccelCallback);

    mavMode = controller.serviceClient<mavros_msgs::SetMode>((ros_namespace + "/mavros/set_mode").c_str());
    mavArming = controller.serviceClient<mavros_msgs::CommandBool>((ros_namespace + "/mavros/cmd/arming").c_str());
    takeoff = controller.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/takeoff").c_str());
    land = controller.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/land").c_str());
}