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
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Trajectory.h>


ros::Publisher posePub;
geometry_msgs::PoseStamped tgtPose;
ros::Publisher velPub;
geometry_msgs::TwistStamped tgtVel;
ros::Publisher accelPub;
geometry_msgs::Vector3Stamped tgtAccel;
ros::Publisher trajPub;
mavros_msgs::Trajectory tgtTraj;

ros::Subscriber stateSubs;
mavros_msgs::State currState;
ros::Subscriber poseSubs;
geometry_msgs::PoseStamped currPose;
ros::Subscriber velSubs;
geometry_msgs::TwistStamped currVel;
ros::Subscriber accelSubs;
geometry_msgs::AccelWithCovarianceStamped currAccel;

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

typedef struct vec
{
    float x;
    float y;
    float z;
};

typedef struct trajectory
{
    int type;
    
    vec pos;
    vec vel;
    vec acc;

    float yaw;
    float yaw_rate;
};

void StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    currState = *msg;
}

void PoseCallback(const geomtery_msgs::PoseStamped::ConstPtr& msg)
{
    currPose = *msg;
}

void VelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    currVel = *msg;
}

void AccelCallback(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& msg)
{
    currAccel = *msg;
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

	while (!stateSubs.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		stateSubs.call(arm_request);
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
{
    bool deltaX = (abs(tgtPose.pose.position.x - currPose.pose.position.x) < linTol);
    bool deltaY = (abs(tgtPose.pose.position.y - currPose.pose.position.y) < linTol);
    bool deltaZ = (abs(tgtPose.pose.position.z - currPose.pose.position.z) < linTol);
    bool deltaOW = (abs(tgtPose.pose.orientation.w - currPose.pose.orientation.w) < orientTol);
    bool deltaOX = (abs(tgtPose.pose.orientation.x - currPose.pose.orientation.x) < orientTol);
    bool deltaOY = (abs(tgtPose.pose.orientation.y - currPose.pose.orientation.y) < orientTol);
    bool deltaOZ = (abs(tgtPose.pose.orientation.z - currPose.pose.orientation.z) < orientTol);

    return (deltaX && deltaY && deltaZ && deltaOW && deltaOX && deltaOY && deltaOZ);
}

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

    posePub.publish(tgtPose);
}

void gotoPos(waypoint Target)
{
    changeHeading(Target.psi);

    tgtPose.pose.position.x = Target.x;
    tgtPose.pose.position.y = Target.y;
    tgtPose.pose.position.z = Target.z;
    
    posePub.publish(tgtPose);

    while(!checkPosTgt)
        ros::Duration(.1).sleep();
}

void cmdTakeoff(float alti)
{
    initAlti = currPose.pose.position.z;

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
        ROS_INFO("Didsizeof(desTraj) / sizeof(mavros_msgs::Trajectory) not reach target.");
}

void cmdLand()
{
    cmdTakeoff(float initAlti);
}

bool checkVelTgt(float velTol = 0.01)
{
    bool delVx = (abs(tgtVel.twist.linear.x - currVel.twist.linear.x) < velTol);
    bool delVy = (abs(tgtVel.twist.linear.y - currVel.twist.linear.y) < velTol);
    bool delVz = (abs(tgtVel.twist.linear.z - currVel.twist.linear.z) < velTol);

    return (delVx && delVy && delVz);
}

void attainVel(float vx, float vy, float vz)
{
    tgtVel.twist.linear.x = vx;
    tgtVel.twist.linear.y = vy;
    tgtVel.twist.linear.z = vz;

    velPub.publish(tgtVel);

	while(!checkVelTgt)
        ros::Duration(.1).sleep();
}

bool checkAccelTgt(float accelTol = 0.01)
{
    bool delAx = (abs(tgtAccel.vector.x - currAccel.accel.accel.linear.x) < accelTol);
    bool delAy = (abs(tgtAccel.vector.y - currAccel.accel.accel.linear.y) < accelTol);
    bool delAz = (abs(tgtAccel.vector.z - currAccel.accel.accel.linear.z) < accelTol);

    return (delAx && delAy && delAz);
}

void attainAccel(float ax, float ay, float az)
{
    tgtAccel.vector.x = ax;
    tgtAccel.vector.y = ay;
    tgtAccel.vector.z = az;

    accelPub.publish(tgtAccel);

	while(!checkAccelTgt)
        ros::Duration(.1).sleep();
}

void trajAllocate(trajectory desTraj[], int a, int b)
{
    if(b - a == 1)
    {
        tgtTraj.type = desTraj[a].type;

        tgtTraj.point_1.position.x = desTraj[a].pos.x;
        tgtTraj.point_1.position.y = desTraj[a].pos.y;
        tgtTraj.point_1.position.z = desTraj[a].pos.z;

        tgtTraj.point_1.velocity.x = desTraj[a].vel.x;
        tgtTraj.point_1.velocity.y = desTraj[a].vel.y;
        tgtTraj.point_1.velocity.z = desTraj[a].vel.z;

        tgtTraj.point_1.acceleration_or_force.x = desTraj[a].acc.x;
        tgtTraj.point_1.acceleration_or_force.y = desTraj[a].acc.y;
        tgtTraj.point_1.acceleration_or_force.z = desTraj[a].acc.z;

        tgtTraj.point_1.yaw = desTraj[a].yaw;
        tgtTraj.point_1.yaw_rate = desTraj[a].yaw_rate;


        tgtTraj.point_2.position.x = desTraj[a+1].pos.x;
        tgtTraj.point_2.position.y = desTraj[a+1].pos.y;
        tgtTraj.point_2.position.z = desTraj[a+1].pos.z;

        tgtTraj.point_2.velocity.x = desTraj[a+1].vel.x;
        tgtTraj.point_2.velocity.y = desTraj[a+1].vel.y;
        tgtTraj.point_2.velocity.z = desTraj[a+1].vel.z;

        tgtTraj.point_2.acceleration_or_force.x = desTraj[a+1].acc.x;
        tgtTraj.point_2.acceleration_or_force.y = desTraj[a+1].acc.y;
        tgtTraj.point_2.acceleration_or_force.z = desTraj[a+1].acc.z;

        tgtTraj.point_2.yaw = desTraj[a+1].yaw;
        tgtTraj.point_2.yaw_rate = desTraj[a+1].yaw_rate;
    }

    if(b - a == 2)
    {
        tgtTraj.type = desTraj[a].type;

        tgtTraj.point_1.position.x = desTraj[a].pos.x;
        tgtTraj.point_1.position.y = desTraj[a].pos.y;
        tgtTraj.point_1.position.z = desTraj[a].pos.z;

        tgtTraj.point_1.velocity.x = desTraj[a].vel.x;
        tgtTraj.point_1.velocity.y = desTraj[a].vel.y;
        tgtTraj.point_1.velocity.z = desTraj[a].vel.z;

        tgtTraj.point_1.acceleration_or_force.x = desTraj[a].acc.x;
        tgtTraj.point_1.acceleration_or_force.y = desTraj[a].acc.y;
        tgtTraj.point_1.acceleration_or_force.z = desTraj[a].acc.z;

        tgtTraj.point_1.yaw = desTraj[a].yaw;
        tgtTraj.point_1.yaw_rate = desTraj[a].yaw_rate;


        tgtTraj.point_2.position.x = desTraj[a+1].pos.x;
        tgtTraj.point_2.position.y = desTraj[a+1].pos.y;
        tgtTraj.point_2.position.z = desTraj[a+1].pos.z;

        tgtTraj.point_2.velocity.x = desTraj[a+1].vel.x;
        tgtTraj.point_2.velocity.y = desTraj[a+1].vel.y;
        tgtTraj.point_2.velocity.z = desTraj[a+1].vel.z;

        tgtTraj.point_2.acceleration_or_force.x = desTraj[a+1].acc.x;
        tgtTraj.point_2.acceleration_or_force.y = desTraj[a+1].acc.y;
        tgtTraj.point_2.acceleration_or_force.z = desTraj[a+1].acc.z;

        tgtTraj.point_2.yaw = desTraj[a+1].yaw;
        tgtTraj.point_2.yaw_rate = desTraj[a+1].yaw_rate;


        tgtTraj.point_3.position.x = desTraj[a+2].pos.x;
        tgtTraj.point_3.position.y = desTraj[a+2].pos.y;
        tgtTraj.point_3.position.z = desTraj[a+2].pos.z;

        tgtTraj.point_3.velocity.x = desTraj[a+2].vel.x;
        tgtTraj.point_3.velocity.y = desTraj[a+2].vel.y;
        tgtTraj.point_3.velocity.z = desTraj[a+2].vel.z;

        tgtTraj.point_3.acceleration_or_force.x = desTraj[a+2].acc.x;
        tgtTraj.point_3.acceleration_or_force.y = desTraj[a+2].acc.y;
        tgtTraj.point_3.acceleration_or_force.z = desTraj[a+2].acc.z;

        tgtTraj.point_3.yaw = desTraj[a+2].yaw;
        tgtTraj.point_3.yaw_rate = desTraj[a+2].yaw_rate;
    }

    if(b - a == 3)
    {
        tgtTraj.type = desTraj[a].type;

        tgtTraj.point_1.position.x = desTraj[a].pos.x;
        tgtTraj.point_1.position.y = desTraj[a].pos.y;
        tgtTraj.point_1.position.z = desTraj[a].pos.z;

        tgtTraj.point_1.velocity.x = desTraj[a].vel.x;
        tgtTraj.point_1.velocity.y = desTraj[a].vel.y;
        tgtTraj.point_1.velocity.z = desTraj[a].vel.z;

        tgtTraj.point_1.acceleration_or_force.x = desTraj[a].acc.x;
        tgtTraj.point_1.acceleration_or_force.y = desTraj[a].acc.y;
        tgtTraj.point_1.acceleration_or_force.z = desTraj[a].acc.z;

        tgtTraj.point_1.yaw = desTraj[a].yaw;
        tgtTraj.point_1.yaw_rate = desTraj[a].yaw_rate;


        tgtTraj.point_2.position.x = desTraj[a+1].pos.x;
        tgtTraj.point_2.position.y = desTraj[a+1].pos.y;
        tgtTraj.point_2.position.z = desTraj[a+1].pos.z;

        tgtTraj.point_2.velocity.x = desTraj[a+1].vel.x;
        tgtTraj.point_2.velocity.y = desTraj[a+1].vel.y;
        tgtTraj.point_2.velocity.z = desTraj[a+1].vel.z;

        tgtTraj.point_2.acceleration_or_force.x = desTraj[a+1].acc.x;
        tgtTraj.point_2.acceleration_or_force.y = desTraj[a+1].acc.y;
        tgtTraj.point_2.acceleration_or_force.z = desTraj[a+1].acc.z;

        tgtTraj.point_2.yaw = desTraj[a+1].yaw;
        tgtTraj.point_2.yaw_rate = desTraj[a+1].yaw_rate;


        tgtTraj.point_3.position.x = desTraj[a+2].pos.x;
        tgtTraj.point_3.position.y = desTraj[a+2].pos.y;
        tgtTraj.point_3.position.z = desTraj[a+2].pos.z;

        tgtTraj.point_3.velocity.x = desTraj[a+2].vel.x;
        tgtTraj.point_3.velocity.y = desTraj[a+2].vel.y;
        tgtTraj.point_3.velocity.z = desTraj[a+2].vel.z;

        tgtTraj.point_3.acceleration_or_force.x = desTraj[a+2].acc.x;
        tgtTraj.point_3.acceleration_or_force.y = desTraj[a+2].acc.y;
        tgtTraj.point_3.acceleration_or_force.z = desTraj[a+2].acc.z;

        tgtTraj.point_3.yaw = desTraj[a+2].yaw;
        tgtTraj.point_3.yaw_rate = desTraj[a+2].yaw_rate;


        tgtTraj.point_4.position.x = desTraj[a+3].pos.x;
        tgtTraj.point_4.position.y = desTraj[a+3].pos.y;
        tgtTraj.point_4.position.z = desTraj[a+3].pos.z;

        tgtTraj.point_4.velocity.x = desTraj[a+3].vel.x;
        tgtTraj.point_4.velocity.y = desTraj[a+3].vel.y;
        tgtTraj.point_4.velocity.z = desTraj[a+3].vel.z;

        tgtTraj.point_4.acceleration_or_force.x = desTraj[a+3].acc.x;
        tgtTraj.point_4.acceleration_or_force.y = desTraj[a+3].acc.y;
        tgtTraj.point_4.acceleration_or_force.z = desTraj[a+3].acc.z;

        tgtTraj.point_4.yaw = desTraj[a+3].yaw;
        tgtTraj.point_4.yaw_rate = desTraj[a+3].yaw_rate;
    }

    if(b - a == 4)
    {
        tgtTraj.type = desTraj[a].type;

        tgtTraj.point_1.position.x = desTraj[a].pos.x;
        tgtTraj.point_1.position.y = desTraj[a].pos.y;
        tgtTraj.point_1.position.z = desTraj[a].pos.z;

        tgtTraj.point_1.velocity.x = desTraj[a].vel.x;
        tgtTraj.point_1.velocity.y = desTraj[a].vel.y;
        tgtTraj.point_1.velocity.z = desTraj[a].vel.z;

        tgtTraj.point_1.acceleration_or_force.x = desTraj[a].acc.x;
        tgtTraj.point_1.acceleration_or_force.y = desTraj[a].acc.y;
        tgtTraj.point_1.acceleration_or_force.z = desTraj[a].acc.z;

        tgtTraj.point_1.yaw = desTraj[a].yaw;
        tgtTraj.point_1.yaw_rate = desTraj[a].yaw_rate;


        tgtTraj.point_2.position.x = desTraj[a+1].pos.x;
        tgtTraj.point_2.position.y = desTraj[a+1].pos.y;
        tgtTraj.point_2.position.z = desTraj[a+1].pos.z;

        tgtTraj.point_2.velocity.x = desTraj[a+1].vel.x;
        tgtTraj.point_2.velocity.y = desTraj[a+1].vel.y;
        tgtTraj.point_2.velocity.z = desTraj[a+1].vel.z;

        tgtTraj.point_2.acceleration_or_force.x = desTraj[a+1].acc.x;
        tgtTraj.point_2.acceleration_or_force.y = desTraj[a+1].acc.y;
        tgtTraj.point_2.acceleration_or_force.z = desTraj[a+1].acc.z;

        tgtTraj.point_2.yaw = desTraj[a+1].yaw;
        tgtTraj.point_2.yaw_rate = desTraj[a+1].yaw_rate;


        tgtTraj.point_3.position.x = desTraj[a+2].pos.x;
        tgtTraj.point_3.position.y = desTraj[a+2].pos.y;
        tgtTraj.point_3.position.z = desTraj[a+2].pos.z;

        tgtTraj.point_3.velocity.x = desTraj[a+2].vel.x;
        tgtTraj.point_3.velocity.y = desTraj[a+2].vel.y;
        tgtTraj.point_3.velocity.z = desTraj[a+2].vel.z;

        tgtTraj.point_3.acceleration_or_force.x = desTraj[a+2].acc.x;
        tgtTraj.point_3.acceleration_or_force.y = desTraj[a+2].acc.y;
        tgtTraj.point_3.acceleration_or_force.z = desTraj[a+2].acc.z;

        tgtTraj.point_3.yaw = desTraj[a+2].yaw;
        tgtTraj.point_3.yaw_rate = desTraj[a+2].yaw_rate;


        tgtTraj.point_4.position.x = desTraj[a+3].pos.x;
        tgtTraj.point_4.position.y = desTraj[a+3].pos.y;
        tgtTraj.point_4.position.z = desTraj[a+3].pos.z;

        tgtTraj.point_4.velocity.x = desTraj[a+3].vel.x;
        tgtTraj.point_4.velocity.y = desTraj[a+3].vel.y;
        tgtTraj.point_4.velocity.z = desTraj[a+3].vel.z;

        tgtTraj.point_4.acceleration_or_force.x = desTraj[a+3].acc.x;
        tgtTraj.point_4.acceleration_or_force.y = desTraj[a+3].acc.y;
        tgtTraj.point_4.acceleration_or_force.z = desTraj[a+3].acc.z;

        tgtTraj.point_4.yaw = desTraj[a+3].yaw;
        tgtTraj.point_4.yaw_rate = desTraj[a+3].yaw_rate;


        tgtTraj.point_5.position.x = desTraj[a+4].pos.x;
        tgtTraj.point_5.position.y = desTraj[a+4].pos.y;
        tgtTraj.point_5.position.z = desTraj[a+4].pos.z;

        tgtTraj.point_5.velocity.x = desTraj[a+4].vel.x;
        tgtTraj.point_5.velocity.y = desTraj[a+4].vel.y;
        tgtTraj.point_5.velocity.z = desTraj[a+4].vel.z;

        tgtTraj.point_5.acceleration_or_force.x = desTraj[a+4].acc.x;
        tgtTraj.point_5.acceleration_or_force.y = desTraj[a+4].acc.y;
        tgtTraj.point_5.acceleration_or_force.z = desTraj[a+4].acc.z;

        tgtTraj.point_5.yaw = desTraj[a+4].yaw;
        tgtTraj.point_5.yaw_rate = desTraj[a+4].yaw_rate;
    }
}

void createTraj(trajectory desTraj[])
{
    int n = sizeof(desTraj) / sizeof(mavros_msgs::Trajectory);
    
    for(int i = 0; i < n; i++)
    {
        if(n - i < 5)
        {
            trajAllocate(desTraj, i, n-1);
            trajPub.publish(tgtTraj);
            break;
        }
        else
        {
            trajAllocate(desTraj, i, i+4);
            trajPub.publish(tgtTraj);
        }
    }
}

void initControler(ros::NodeHandle controller)
{
    std::string ros_namespace;
	if(!controlnode.hasParam("namespace"))
		ROS_INFO("Using default namespace");
    else
    {
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("Using namespace %s", ros_namespace.c_str());
	}

    posePub = controller.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/setpoint_position/local"),c_str(), 100);
    velPub = controller.advertise<geometry_msgs::TwistStamped>((ros_namespace + "/mavros/setpoint_velocity/cmd_vel").c_str(), 100);
    accelPub = controller.advertise<geometry_msgs::Vector3Stamped>((ros_namespace + "/mavros/setpoint_accel/accel").c_str(), 100);
    trajPub = controller.advertise<mavros_msgs::Trajectory>((ros_namespace + "/mavros/trajectory/generated".c_str()), 100);

    stateSubs = controller.subscribe<mavros_msgs::State>((ros_namespace + "/mavros/state").c_str(), 100, StateCallback);
    poseSubs = controller.subscribe<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/local_position/pose").c_str(), 100, PoseCallback);
    velSubs = controller.subscribe<geometry_msgs::TwistStamped>((ros_namespace + "geometry_msgs/TwistStamped").c_str(), 100, VelCallback);
    accelSubs = controller.subscribe<geometry_msgs::AccelWithCovarianceStamped>((ros_namespace + "/mavros/local_position/accel").c_str(), 100, AccelCallback);

    mavMode = controller.serviceClient<mavros_msgs::SetMode>((ros_namespace + "/mavros/set_mode").c_str());
    mavArming = controller.serviceClient<mavros_msgs::CommandBool>((ros_namespace + "/mavros/cmd/arming").c_str());
    takeoff = controller.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/takeoff").c_str());
    land = controller.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/land").c_str());

    ROS_INFO("Controller has been initialized");
}
