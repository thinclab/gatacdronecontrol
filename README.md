GaTACDroneControl
=================

Overview
-----

This API is designed to provide high level control of (up to 3) Parrot AR.Drone 
Quadcopters over a UDP socket.  The purpose of the API is to allow for programmatic control of drones from a client machine totally independent of ROS.  However, note that the machine running the server does indeed require ROS and all packages mentioned below.

Building
-----

Required libraries: Boost (threading and system), doxygen

A makefile is provided to compile the example programs.

Notes
-----

The machine running the Drone Server requires:
* https://github.com/thinclab/ardrone_thinc
* https://github.com/thinclab/thinc_simulator
* https://github.com/thinclab/uga_tum_ardrone

Writing Your Own Drone Server
-----

For a sample server program with comments, see `src/server.cpp`.  To write your own drone server program, make sure the following actions are performed in order (with possible steps in between):


Server Command Sequence:

1. #include "GaTACDroneControlServer.hpp".
2. Instantiate a GaTACDroneControl object.
3. Call the `startServer("IP", Port, # of drones)` method with the IP and Port that clients will use to connect.  The third parameter tells the server how many clients to expect for a given simulation.  This method should be the last call in your program, as it will run the drone server until the program terminates.

Writing Your Own Drone Client
-----

For a sample client program with comments, see `src/randomClient.cpp`.  

One thing to note is that only the first setGridSize() call from ANY client is honored by the server, all others are ignored.  This means the first client to connect controls the size of the grid to fly on
