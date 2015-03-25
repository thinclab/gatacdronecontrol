#include <iostream>
#include <unistd.h>
#include "GaTACDroneControlClient.hpp"

/*
 * Sample client code to demonstrate usage of the GaTACDroneControl API.
 */

int main() {
	// Specifying the IP and port of server machine
	string ip = "127.0.0.1";
	unsigned int port = 4999;
	// Instantiate GaTACDroneControl object

	GaTACDroneControl gatac("TAGDRONE");

	// Launch Drone Client
	gatac.launchClient(ip, port);

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);

	//set up drone
	gatac.setupDrone(0, 0); // Spawn drone at (2, 2)

	// Sending ready message
	gatac.readyUp();

	gatac.move(0, 7);

	gatac.land();

	// Close client socket connection.
	gatac.closeClient();

	return 0;
}
