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

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac;

	// Launch Drone Client
	gatac.launchClient(ip, startport);

	int id = gatac.getClientUniqueId();

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);

	//set up drone
	gatac.setupDrone(id % 5, id / 5); // Spawn drone at (0, 0)

	// Sending ready message
	gatac.readyUp();

	//Setting id of drone to client's unique id

	srand (time(NULL) * (id + 1));

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
