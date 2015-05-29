#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "GaTACDroneControlClient.hpp"
#include <boost/thread.hpp> // For concurrent commands and data reads
#include <boost/date_time.hpp>


using namespace std;
/*
 * Sample client code to demonstrate usage of the GaTACDroneControl API.
 */

int main() {
	// Specifying the IP and port of server machine
	string ip = "127.0.0.1";
	unsigned int port = 4999; //command port

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac("TESTDRONE2");

	// Launch Drone Client
	gatac.launchClient(ip, port);

	// Set grid size to [5 x 8]
	gatac.setGridSize(5, 8);

	//set up drone
	gatac.setupDrone(4, 0); // Spawn drone at (0, 0)

	// Sending ready message
	gatac.readyUp();

	//Drones will move, intersecting at various points, reported on console
	while(gatac.getClientReadyToCommand() == true && ! gatac.isScenarioOver()){
        gatac.move( 4, 7);
        boost::thread *moveThread;
        moveThread = new boost::thread(boost::bind(&GaTACDroneControl::move, &gatac, 4, 0));
        for(int i = 1; i < 6; i++){
            cout << gatac.getBattery() << endl;
            sleep(i);
        }
        moveThread->join();

        boost::thread *moveThread2;
        moveThread2 = new boost::thread(boost::bind(&GaTACDroneControl::move, &gatac, 3, 6));
        cout << gatac.getBattery() << endl;

        moveThread2->join();

        delete moveThread;
        delete moveThread2;

        //Drones land
        gatac.land();

	}

    if (gatac.isScenarioOver()) {
        cout << "Scenario Ended with Message: " << gatac.getScenarioOverMessage() << endl;
    }
    gatac.closeClient();
	return 0;
}
