# copterControl.hpp
This repository provides a `C++` library for controlling Unmanned Aerial Vehicles (UAVs) using the `Robot Operating System (ROS)` and the `MAVLink` communication protocol. It specifically interacts with the `MAVROS` package, a ROS driver for autopilots like `ArduPilot`.

## Flight Control API
* `void initControler(ros::NodeHandle controller)` : Used to initialize all the Publisher, Subscribers and Services that have been used by this library in its various functionalities.
* `bool setMode(std::string mode = "GUIDED")` : Used to communicate to the autopilot the Flight Mode that is to be used during operation. If no argument is given, it is set to be in `GUIDED` mode.
* `bool arm()` : Used for arming of the drone before operations begin.
* `void cmdTakeoff(float alti)` : Used to command the UAV to takeoff to an altitude specified by the argument `alti`. Each time this function is called it also stored the altitude in `LOCAL_NED Frame` to be used by the landing function.
* `void cmdLand()` : Used to safely return the UAV to the local ground as recorded by cmdTakeoff() in the global variable initAlti.
