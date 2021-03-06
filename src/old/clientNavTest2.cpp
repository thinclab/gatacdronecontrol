#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "GaTACDroneControl.hpp"
#include <boost/thread.hpp> // For concurrent commands and data reads
#include <boost/date_time.hpp>


using namespace std;
/*
 * Sample client code to demonstrate usage of the GaTACDroneControl API.
 */

int main() {
	// Specifying the IP and port of server machine
	char *ip = "128.192.76.247";
	char *port = "5999";

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac;

	// Launch Drone Client
	gatac.launchClient(ip, port);

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);

	//set up drone
	gatac.setupDrone(3, 0); // Spawn drone at (2, 2)

	// Sending ready message
	gatac.readyUp();

	//Setting id of drone to client's unique id
	int id = gatac.getClientUniqueId();

	//Drones will move, intersecting at various points, reported on console
	while(gatac.getClientReadyToCommand() == true){
	boost::thread *dataThread;
	dataThread = new boost::thread(boost::bind(&GaTACDroneControl::receiveData, gatac, id));
	gatac.move(id, 3, 7);
	cout << gatac.getBattery() << endl;
	cout << gatac.getForwardVelocity() << endl;
	boost::thread *moveThread;
	moveThread = new boost::thread(boost::bind(&GaTACDroneControl::move, gatac,id, 0, 0));
	while(1){
	cout << gatac.getForwardVelocity() << endl;
	sleep(1);
	}




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
//	gatac.closeClient();
	}
	return 0;
}
