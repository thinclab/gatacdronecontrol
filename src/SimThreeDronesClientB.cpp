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
	char *ip = "127.0.0.1";
	unsigned int port = 5001; //command port
	unsigned int dp = 5002; //data port

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac;

	// Launch Drone Client
	gatac.launchClient(ip, port, dp);

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);

	//set up drone
	gatac.setupDrone(4, 0); // Spawn drone at (0, 0)

	// Sending ready message
	gatac.readyUp();

	//Setting id of drone to client's unique id
	int id = gatac.getClientUniqueId();

	//Drones will move, intersecting at various points, reported on console
	while(gatac.getClientReadyToCommand() == true){
        boost::thread *dataThread;
        dataThread = new boost::thread(boost::bind(&GaTACDroneControl::receiveData, gatac, id));
        gatac.move(id, 4, 7);
        boost::thread *moveThread;
        moveThread = new boost::thread(boost::bind(&GaTACDroneControl::move, gatac,id, 4, 0));
        for(int i = 1; i < 6; i++){
            cout << gatac.getBattery() << endl;
            sleep(i);
        }
        boost::thread *moveThread2;
        moveThread2 = new boost::thread(boost::bind(&GaTACDroneControl::move, gatac,id, 3, 6));
        cout << gatac.getBattery() << endl;

        //Drones land
        gatac.land(id);

        gatac.closeClient();
	}
	return 0;
}
