#include <iostream>
#include <unistd.h>
#include "GaTACDroneControl.hpp"

/*
 * Sample client code to demonstrate usage of the GaTACDroneControl API.
 */

int main() {
	// Specifying the IP and port of server machine
	char *ip = "128.192.76.247";
	char *port = "5999";

	// Instantiate GaTACDroneControl object
	const char* c = "r";
 	GaTACDroneControl gatac(c);
	// Launch Drone Client
	gatac.launchClient(ip, port);

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);
	
	//set up drone
	gatac.setupDrone(1, 1); // Spawn drone at (1, 1)

	// Sending ready message
	gatac.readyUp();

	//Setting id of drone to client's unique id
	int id = gatac.getClientUniqueId();

	//Drones will move, intersecting at various points, reported on console
	while(gatac.getClientReadyToCommand() == true){
	gatac.takeoff(id);
	gatac.move(id, 1, 2);
	gatac.move(id, 2, 3);
	gatac.move(id, 1, 1);

	//Drones land
	gatac.land(id);
	
	//Uncomment to check errors: invalid drone ID and invalid location
/*
	//Out-of-bounds test	
	gatac.takeoff(1);
	std::cout << "Client test: passing out-of-bounds waypoint..." << std::endl;	
	gatac.move(1, 5, 6);
	
	//Incorrect ID Test
	gatac.takeoff(1);	
	std::cout << "Client test: passing incorrect drone ID..." << std::endl;
	gatac.move(3, 1, 6);
*/
	//End error checks

	// Close client socket connection.
	gatac.closeClient();
	}
	return 0;
}
