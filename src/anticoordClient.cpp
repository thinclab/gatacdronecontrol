#include <iostream>
#include <unistd.h>
#include "GaTACDroneControlClient.hpp"
#include <stdlib.h>
#include <time.h>
#include "policytree.h"

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
        std::cerr << "Invalid arguments, correct format is: " << argv[0] << " size_X size_Y start_x start_Y \"policy file\"" << endl;
        exit(1);
    }

    int x, y, startX, startY, sizeX, sizeY;
    char * mode = argv[0];
    sscanf(argv[1], "%d", &sizeX);
    sscanf(argv[2], "%d", &sizeY);
    sscanf(argv[3], "%d", &startX);
    sscanf(argv[4], "%d", &startY);
    char * policy_file = argv[5];

	x = startX;
	y = startY;


    PolicyTree policyTree;
    policyTree.readPolicyTree(policy_file);

    PolicyTreeNode * curNode = policyTree.root;

	// Specifying the IP and port of server machine
	string ip = "127.0.0.1";
	int startport = 4999;

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac("ANTICOORD");

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


	//Drones will move, intersecting at various points, reported on console
	while(gatac.getClientReadyToCommand() == true && ! gatac.isScenarioOver()){

        // perform action at the current node

        switch (curNode->action) {
        case 0:
            // move North
            CHECK_AND_MOVE(0, 1);

            break;
        case 1:
            // move South
            CHECK_AND_MOVE(0, -1);

            break;
        case 2:
            // move East
            CHECK_AND_MOVE(1, 0);

            break;
        case 3:
            // move West
            CHECK_AND_MOVE(-1, 0);

            break;
        case 4:
            gatac.landHere();
            break;
        default:
            // NoOp
            sleep(3);

            break;


        }

        // gather and interpret percepts

        int per = -1;

        percept p = gatac.senseNorth(sizeX * sizeY);
        for (int i = 0; i < p.size(); i ++) {
            per = 0;
        }

        p = gatac.senseSouth(sizeX * sizeY);
        for (int i = 0; i < p.size(); i ++) {
            per = 1;
        }

        p = gatac.senseEast(sizeX * sizeY);
        for (int i = 0; i < p.size(); i ++) {
            per = 2;
        }

        p = gatac.senseWest(sizeX * sizeY);
        for (int i = 0; i < p.size(); i ++) {
            per = 3;
        }


        if (curNode->children.size() == 0) {
            // ran out of tree!
//            gatac.sendScenarioIsOver("POLICY ENDED");

            break;

        }
        curNode = curNode->children.at(per);



	}


//    gatac.land();

    // Close client socket connection.
    gatac.closeClient();


	return 0;
}

