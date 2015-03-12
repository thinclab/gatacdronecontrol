#include <iostream>
#include <unistd.h>
#include "GaTACDroneControlClient.hpp"
#include <stdlib.h>
#include <time.h>

/*
 * Sample client code to demonstrate usage of the GaTACDroneControl API.
 */

int main(int argc, char * argv[]) {
	// Specifying the IP and port of server machine
	char *ip = "127.0.0.1";
	int startport = 4999;
	int dronenum = 0;
	sscanf(argv[1], "%d", &dronenum);

	unsigned int port = startport + dronenum * 2;
	unsigned int dp = startport + dronenum * 2 + 1;

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac;

	// Launch Drone Client
	gatac.launchClient(ip, port, dp);

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);

	//set up drone
	gatac.setupDrone(dronenum % 5, dronenum / 5); // Spawn drone at (0, 0)

	// Sending ready message
	gatac.readyUp();

	//Setting id of drone to client's unique id
	int id = gatac.getClientUniqueId();

	srand (time(NULL) * dronenum);

	//Drones will move, intersecting at various points, reported on console
	while(gatac.getClientReadyToCommand() == true){

	     gatac.move(id, rand() % 5, rand() % 8);

	}

        //Drones land
        gatac.land(id);

        // Close client socket connection.
        gatac.closeClient();
	return 0;
}
