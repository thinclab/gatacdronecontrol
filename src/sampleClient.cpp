#include <iostream>
#include <unistd.h>
#include "GaTACDroneControl.hpp"

/*
 * Sample client code to demonstrate usage of the GaTACDroneControl API.
 */

int main() {
	// Specifying the IP and port of server machine
	char *ip = "000.000.00.000";
	char *port = "0000";

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac;

	// Launch Drone Client
	gatac.launchClient(ip, port);

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);

	// Set up 3 drones
	gatac.setupDrone(3, 3); // Spawn drone at (3, 3)
	gatac.setupDrone(2, 4); // Spawn drone at (2, 4)
	gatac.setupDrone(0, 0); // Spawn drone at (0, 0)

	// Launch Gazebo Simulator
	gatac.startGrid();

	// Move drone with ID 0 to (0, 1)
	gatac.move(0, 0, 1);

	// Move drone with ID 1 to (1, 2)
	gatac.move(1, 1, 2);

	// Land drone with ID 0
	gatac.land(0);

	// Land drone with ID 1
	gatac.land(1);

	// Take off drone with ID 0
	gatac.takeoff(0);

	// Close client socket connection.
	gatac.closeClient();

	return 0;
}
