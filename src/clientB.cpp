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
	
	//set up drone
	gatac.setupDrone(1, 2); // Spawn drone at (2, 2)

	// Launch Gazebo Simulator
	gatac.startGrid();

	//Drones will move, intersecting at various points, reported on console
	gatac.move(0, 3, 2);
	gatac.move(0, 1, 4);
	gatac.move(0, 1, 7);

	//Drones land

	
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

	return 0;
}
