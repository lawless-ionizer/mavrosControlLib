# copterControl.hpp
This repository provides a `C++` library for controlling Unmanned Aerial Vehicles (UAVs) using the `Robot Operating System (ROS)` and the `MAVLink` communication protocol. It specifically interacts with the `MAVROS` package, a ROS driver for autopilots like `ArduPilot`.

## Flight Control API
`bool setMode(std::string mode = "GUIDED")`
Used to communicate to the autopilot the Flight Mode that is to be used during operation. If no argument is given, it is set to be in `GUIDED` mode.

`bool arm()`
Used for arming of the drone before operations begin.
