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
    if (argc < 6) {
        std::cerr << "Invalid arguments, correct format is: " << argv[0] << " role(f/u) size_X size_Y start_x start_Y \"policy file\"" << endl;
        exit(1);
    }

    int x, y, startX, startY, sizeX, sizeY;
    char * mode = argv[1];
    sscanf(argv[2], "%d", &sizeX);
    sscanf(argv[3], "%d", &sizeY);
    sscanf(argv[4], "%d", &startX);
    sscanf(argv[5], "%d", &startY);
    char * policy_file = argv[6];

    char * role;
    bool isFugitive = false;
cout << policy_file << endl;
    if (strncasecmp(mode, "f", 1) == 0) {
        isFugitive = true;
    }

    if (isFugitive) {
        role = "FUGITIVE";
    } else {
        role = "UAV";
    }

	x = startX;
	y = startY;


    PolicyTree policyTree;
    policyTree.readPolicyTree(policy_file);

    PolicyTreeNode * curNode = policyTree.root;

	// Specifying the IP and port of server machine
	string ip = "127.0.0.1";
	int startport = 4999;

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac(role);

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
            CHECK_AND_MOVE(1, 0);

            break;
        case 1:
            // move South
            CHECK_AND_MOVE(-1, 0);

            break;
        case 2:
            // move East
            CHECK_AND_MOVE(0, 1);

            break;
        case 3:
            // move West
            CHECK_AND_MOVE(0, -1);

            break;
        default:
            // NoOp

            break;


        }

        if (curNode->children.size() == 0) {
            // ran out of tree!

            gatac.sendScenarioIsOver("POLICY ENDED");

            break;

        }

        // gather and interpret percepts

        std::pair<std::string, int> * uav1 = NULL;
        int uav1dir = -1;
        std::pair<std::string, int> * uav2 = NULL;
        int uav2dir = -1;
        std::pair<std::string, int> * fug = NULL;
        int fugdir = -1;

        percept n = gatac.senseNorth(sizeX + 1);
        for (int i = 0; i < n.size(); i ++) {
            if (strncmp(n.at(i).first.c_str(), "UAV", 3) == 0) {
                if (uav1 == NULL) {
                    uav1 = &(n.at(i));
                    uav1dir = 0;
                } else {
                    uav2 = &(n.at(i));
                    uav2dir = 0;
                }
            } else {
                fug = &(n.at(i));
                fugdir = 0;
            }

        }

        percept s = gatac.senseSouth(sizeX + 1);
        for (int i = 0; i < n.size(); i ++) {
            if (strncmp(n.at(i).first.c_str(), "UAV", 3) == 0) {
                if (uav1 == NULL) {
                    uav1 = &(n.at(i));
                    uav1dir = 1;
                } else {
                    uav2 = &(n.at(i));
                    uav2dir = 1;
                }
            } else {
                fug = &(n.at(i));
                fugdir = 1;
            }

        }

        percept e = gatac.senseEast(sizeY + 1);
        for (int i = 0; i < n.size(); i ++) {
            if (strncmp(n.at(i).first.c_str(), "UAV", 3) == 0) {
                if (uav1 == NULL) {
                    uav1 = &(n.at(i));
                    uav1dir = 2;
                } else {
                    uav2 = &(n.at(i));
                    uav2dir = 2;
                }
            } else {
                fug = &(n.at(i));
                fugdir = 2;
            }

        }

        percept w = gatac.senseWest(sizeY + 1);
        for (int i = 0; i < n.size(); i ++) {
            if (strncmp(n.at(i).first.c_str(), "UAV", 3) == 0) {
                if (uav1 == NULL) {
                    uav1 = &(n.at(i));
                    uav1dir = 3;
                } else {
                    uav2 = &(n.at(i));
                    uav2dir = 3;
                }
            } else {
                fug = &(n.at(i));
                fugdir = 3;
            }

        }



        int percept = 0;
        bool sameSquare = false;

        if (isFugitive) {

            percept = (uav1dir << 2) + uav2dir;

            if (uav1->second == 0 || uav2->second == 0)
                sameSquare = true;


        } else {
            percept = fugdir;
            if (fug->second == 0)
                sameSquare = true;
        }



        if (!isFugitive && sameSquare) {
            // we've caught the fugitive!

            gatac.sendScenarioIsOver("CAUGHT THE FUGITIVE");

            break;

        }

        // move forward in tree

        curNode = curNode->children.at(percept);



	}


    gatac.land();

    // Close client socket connection.
    gatac.closeClient();


	return 0;
}

