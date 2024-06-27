# copterControl.hpp
This repository provides a `C++` library for controlling Unmanned Aerial Vehicles (UAVs) using the `Robot Operating System (ROS)` and the `MAVLink` communication protocol. It specifically interacts with the `MAVROS` package, a ROS driver for autopilots like `ArduPilot`.

## Flight Control API
* `void initControler(ros::NodeHandle controller)` : Used to initialize all the Publisher, Subscribers and Services that have been used by this library in its various functionalities.
* `bool setMode(std::string mode = "GUIDED")` : Used to communicate to the autopilot the Flight Mode that is to be used during operation. If no argument is given, it is set to be in `GUIDED` mode.
* `bool arm()` : Used for arming of the drone before operations begin.
* `void cmdTakeoff(float alti)` : Used to command the UAV to takeoff to an altitude specified by the argument `alti`. Each time this function is called it also stored the altitude in `LOCAL_NED Frame` to be used by the landing function.
* `void cmdLand()` : Used to safely return the UAV to the local ground as recorded by cmdTakeoff() in the global variable initAlti.

## Control Functions
* `void gotoPos(waypoint Target)` : Used to define the target Pose(x,y,z,yaw) for the UAV to achive in LOCAL_NED Frame and wait till it is verified by the `bool checkPosTgt(float linTol = 0.5, float orientTol = 0.1)` function.
* `void changeHeading(float yaw)` : Used to change the heading of the UAV.
* `void attainVel(float vx, float vy, float vz)` : Used to define the target LinearTwist(Vx,Vy,Vz) for the UAV to achive in LOCAL_NED Frame and wait till it is verified by the `bool checkVelTgt(float velTol = 0.01)` function.
* `void attainAccel(float ax, float ay, float az)` : Used to define the force applied (or acceleration) on the UAV in LOCAL_NED Frame and is verified by `bool checkAccelTgt(float accelTol = 0.01)` function.

## Trajectory Generation
`void createTraj(trajectory desTraj[])` : This function taked as input an array of type `trajectory` defined in the library. Each element of this array contains the LOCAL_NED Pose, Velocity and Acceleration vectors as well as the yaw and yaw-rate at those points. `type` is also an argument in this structure which takes two possible values:
* `type = 0` for Waypoint Trajectory
* `type = 1` for Bezier Trajectory (Note: this feature has not been refined, hence it is advised not to use this until it has been).
This function utilizes the `mavros_msgs/Trajectory` message that generates a smooth trajectory between 5 well-defined points for any given frame.
