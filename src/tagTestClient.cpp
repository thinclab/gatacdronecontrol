#include <iostream>
#include <unistd.h>
#include "GaTACDroneControl.hpp"

/*
 * Sample client code to demonstrate usage of the GaTACDroneControl API.
 */

int main() {
	// Specifying the IP and port of server machine
	char *ip = "127.0.0.1";
	unsigned int port = 4999;
	unsigned int dp = 5000;

	// Instantiate GaTACDroneControl object
	const char* c = "r";
	GaTACDroneControl gatac(c);

	// Launch Drone Client
	gatac.launchClient(ip, port, dp);

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);
	
	//set up drone
	gatac.setupDrone(0, 0); // Spawn drone at (2, 2)

	// Sending ready message
	gatac.readyUp();
	
	//Setting id of drone to client's unique id
	int id = gatac.getClientUniqueId();

	gatac.move(id, 0, 7);

	gatac.land(id);

	// Close client socket connection.
	gatac.closeClient();

	return 0;
}
