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
	char *dp = "4998";

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac;

	// Launch Drone Client
	gatac.launchClient(ip, port, dp);

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);
	
	//set up drone
	gatac.setupDrone(0, 0); // Spawn drone at (0, 0)

	// Sending ready message
	gatac.readyUp();

	//Setting id of drone to client's unique id
	int id = gatac.getClientUniqueId();

	//Drones will move, intersecting at various points, reported on console
	while(gatac.getClientReadyToCommand() == true){
	gatac.move(id, 0, 7);
	gatac.move(id, 0, 0);
	gatac.move(id, 0, 7);
	gatac.move(id, 0, 0);
	gatac.move(id, 0, 7);
	gatac.move(id, 0, 0);
	
	//Drones land
	gatac.land(id);

	// Close client socket connection.
	gatac.closeClient();
	}
	return 0;
}
