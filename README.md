GaTACDroneControl
=================

Notes
-----

The machine running the Drone Server requires:
* https://github.com/dmillard/ardrone_thinc
* https://github.com/dmillard/thinc_simulator

Usage
-----

This API is designed to provide high level control of (up to 3) Parrot AR.Drone 
Quadcopters over a UDP socket.  The purpose of the API is to allow for programmatic control of drones from a client machine totally independent of ROS.  However, note that the machine running the server does indeed require ROS and all packages mentioned above.

It currently runs only in the Gazebo simulator, but there are plans for extending its use to real world quadcopters in the future.  

Writing & Running a Drone Server
-----

For a sample server program with comments, see `src/sampleServer.cpp`.  To write your own drone server program, make sure the following actions are performed in order (with possible steps in between):

1. #include "GaTACDroneControl.hpp".
2. Instantiate a GaTACDroneControl object.
3. Call the 'runDroneServer("IP", "Port")' method with the IP and Port of the machine that will be running the client.  This method should be the last call in your program, as it will run the drone server until the program terminates.

Writing & Running a Drone Client
-----

For a sample client program with comments, see `src/sampleClient.cpp`.  To write your own client program, make sure the following actions are performed in order (with possible steps in between):

1. #include "GaTACDroneControl.hpp".
2. Instantiate a GaTACDroneControl object. 
3. Call the 'launchClient("IP", "Port")' method with the IP and Port of the machine that will be running the server.  
...
...
... (call methods in accordance with guidelines below)
...
...
4. Call the 'closeClient()' method to end the connection.


When writing your own client program, please abide by the following guidelines which outline the required sequence of method calls.  If you call any of these methods out of the logical order presented here, it is likely the program will terminate (with an error message).

Client Command Sequence:  
* 1. Launch the client with the 'launchClient("IP", "Port")' method. This sets up the socket connection. No other methods will work until this connection is set up.
* 2. Set the grid size with the 'setGridSize(int, int)' method.  
* 3. Set up each drone you want to control with the 'setUpDrone(int, int)' method. Drones are given ID #'s in increasing order, starting from 0.  The grid size must be set (as in step 2) before this method will work. 
* 4. Start the Gazebo Simulator with the 'startGrid()' method.  
* 5. Assuming all previous calls worked properly, you are now free to command the drones however you wish (e.g. using move(), land(), takeoff(), etc.).