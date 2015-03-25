#include <iostream>
#include <unistd.h>
#include "GaTACDroneControlClient.hpp"
#include <boost/thread.hpp>
/*
 * Sample client code to demonstrate usage of the GaTACDroneControl API.
 */

int main() {
	// Specifying the IP and port of server machine
	string ip = "127.0.0.1";
	unsigned int port = 4999;

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac("TESTDRONE");

	// Launch Drone Client
	gatac.launchClient(ip, port);

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);

	//set up drone
	gatac.setupDrone(0, 4); // Spawn drone at (0, 0)

	// Sending ready message
	gatac.readyUp();

	//Setting id of drone to client's unique id
//	int id = gatac.getClientUniqueId();

	//Drones will move, intersecting at various points, reported on console
	while(gatac.getClientReadyToCommand() == true){
        gatac.senseNorth(); //should return 0
        sleep(3);
        gatac.senseSouth(); //should return 2
//        boost::thread *moveThread;
//        moveThread = new boost::thread(boost::bind(&GaTACDroneControl::move, &gatac,0, 1));
        gatac.move(0, 1);
        sleep(5);
        gatac.senseNorth(); //should return 0
        sleep(3);
        gatac.senseSouth(); //should return 1
        sleep(3);
        gatac.senseEast(); //should return 1 or 2
        sleep(3);
        gatac.senseWest(); //should return 0

gatac.receiveData();
        //Drones land
        gatac.land();

        // Close client socket connection.
        gatac.closeClient();
	}
	return 0;
}
