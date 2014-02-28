#include <iostream>
#include <unistd.h>
#include "GaTACDroneControl.hpp"

/*
 * Sample client code to demonstrate usage of the GaTACDroneControl API.
 */

int main() {
	// Specifying the IP and port of server machine
	char *ip = "128.192.76.247";
	char *port = "4999";

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac;

	// Launch Drone Client
	gatac.launchClient(ip, port);

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);

	// Set up 3 drones
	gatac.setupDrone(2, 2); // Spawn drone at (2, 2)
	gatac.setupDrone(1, 1); // Spawn drone at (1, 1)
	gatac.setupDrone(0, 0); // Spawn drone at (0, 0)

	// Launch Gazebo Simulator
	gatac.startGrid();

	//drone0 moves and lands
	gatac.move(0, 2, 5);
	gatac.land(0);

	//drone1 moves, should return true that 0 & 1 are sharing a space
	gatac.move(1, 2, 5);
	
	//drone2 moves to two empty spaces then lands	
	gatac.move(2, 1, 6);
	gatac.move(2, 3, 4);
	gatac.land(2);

	//drone1 moves, should return true that 1 & 2 are sharing a space
	gatac.move(1, 3, 4);
	gatac.move(1, 4, 3);
	gatac.land(1);


	// Close client socket connection.
	gatac.closeClient();

	return 0;
}
