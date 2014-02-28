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

	//Drone0 moves
	gatac.move(0, 4, 2);
	gatac.move(0, 4, 4);
	gatac.move(0, 2, 4);
	gatac.land(0);
	//Drone1 moves 
	gatac.move(1, 3, 1);
	gatac.move(1, 3, 3);
	gatac.move(1, 1, 3);
	gatac.land(1);
	//Drone2 moves 
	gatac.move(2, 2, 0);
	gatac.move(2, 2, 2);
	gatac.move(2, 0, 2);
	gatac.land(2);
	
	
	//Uncomment to check errors: invalid drone ID and invalid location

/*	gatac.takeoff(1);
	  	std::cout << "Client test: passing incorrect drone ID..." << std::endl;
	gatac.move(3, 1, 5);
	std::cout << "Client passing correct ID now, move drone1 to (1, 5)..." << std::endl;
	gatac.move(1, 1, 5);  
	std::cout << "Client test: passing out-of-bounds waypoint..." << std::endl;	
	gatac.move(1, 5, 3);
	std::cout << "Client passing correct bounds now, move drone1 to (4, 4)..." << std::endl;
	gatac.move(1, 4, 4);
*/
	//End error checks


	//Drone1 flies to origin (0, 0)
	gatac.takeoff(1);
	gatac.move(1, 0, 0);

	//drone1 lands
	gatac.land(1);
	


	// Close client socket connection.
	gatac.closeClient();

	return 0;
}
