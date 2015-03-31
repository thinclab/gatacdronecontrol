#include <iostream>
#include <unistd.h>
#include "GaTACDroneControlClient.hpp"
#include <stdlib.h>
#include <time.h>

using std::endl;
using std::cout;

#define CHECK_AND_MOVE( deltaX, deltaY)                             \
do {                                                                \
    int newX = x + deltaX;                                          \
    int newY = y + deltaY;                                          \
                                                                    \
    if (newX >= sizeX || newY >= sizeY || newX <0 || newY < 0) {    \
                                                                    \
    } else {                                                        \
        x = newX;                                                   \
        y = newY;                                                   \
                                                                    \
        gatac.move(x, y);                                           \
    }                                                               \
} while(0)

/*
 * Sample client code to demonstrate usage of the GaTACDroneControl API.
 */

int main(int argc, char ** argv) {
    if (argc < 5) {
        std::cerr << "Invalid arguments, correct format is: " << argv[0] << " size_X size_Y start_x start_Y" << endl;
        exit(1);
    }

    int x, y, startX, startY, sizeX, sizeY;
    sscanf(argv[1], "%d", &sizeX);
    sscanf(argv[2], "%d", &sizeY);
    sscanf(argv[3], "%d", &startX);
    sscanf(argv[4], "%d", &startY);

	x = startX;
	y = startY;

	// Specifying the IP and port of server machine
	string ip = "127.0.0.1";
	int startport = 4999;

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac("RANDOM");

	// Launch Drone Client
	gatac.launchClient(ip, startport);

	// Set grid size to [5 x 8]
	gatac.setGridSize(sizeX, sizeY);

	//set up drone
	gatac.setupDrone(startX, startY); // Spawn drone at (0, 0)

	// Sending ready message
	gatac.readyUp();

	//Setting id of drone to client's unique id

	int id = gatac.getClientUniqueId();
	srand (time(NULL) * (id + 1));

	//Drones will move, intersecting at various points, reported on console
	while(gatac.getClientReadyToCommand() == true && ! gatac.isScenarioOver()){

        int c = rand() % 4;

        switch (c) {
        case 0:
            CHECK_AND_MOVE(0, 1);
            break;
        case 1:
            CHECK_AND_MOVE(0, -1);
            break;
        case 2:
            CHECK_AND_MOVE(-1, 0);
            break;
        case 3:
            CHECK_AND_MOVE(1, 0);
            break;
        default:
            cout << "Should never be here: " << c << endl;
        }


	}

    //Drones land
    gatac.land();

    // Close client socket connection.
    gatac.closeClient();


	return 0;
}
