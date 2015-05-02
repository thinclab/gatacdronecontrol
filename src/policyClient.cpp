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
    if (argc < 8) {
        std::cerr << "Invalid arguments, correct format is: " << argv[0] << " role(f/u) size_X size_Y start_x start_Y safe_house_X safe_house_Y \"policy file\"" << endl;
        exit(1);
    }

    int x, y, startX, startY, sizeX, sizeY, safehouseX, safehouseY;
    char * mode = argv[1];
    sscanf(argv[2], "%d", &sizeX);
    sscanf(argv[3], "%d", &sizeY);
    sscanf(argv[4], "%d", &startX);
    sscanf(argv[5], "%d", &startY);
    sscanf(argv[6], "%d", &safehouseX);
    sscanf(argv[7], "%d", &safehouseY);
    char * policy_file = argv[8];

    const char * role;
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
        default:
            // NoOp
            sleep(5);
            break;


        }

        // gather and interpret percepts

        std::pair<std::string, int> uav1;
        int uav1dir = -1;
        std::pair<std::string, int> uav2;
        int uav2dir = -1;
        std::pair<std::string, int> fug;
        int fugdir = -1;

        percept p = gatac.senseNorth(sizeX * sizeY);
        for (int i = 0; i < p.size(); i ++) {
            if (strncmp(p.at(i).first.c_str(), "UAV", 3) == 0) {
                if (uav1dir < 0) {
                    uav1 = p.at(i);
                    uav1dir = 0;
                } else {
                    uav2 = p.at(i);
                    uav2dir = 0;
                }
            } else {
                fug = p.at(i);
                fugdir = 0;
            }

        }

        p = gatac.senseSouth(sizeX * sizeY);
        for (int i = 0; i < p.size(); i ++) {
            if (strncmp(p.at(i).first.c_str(), "UAV", 3) == 0) {
                if (uav1dir < 0) {
                    uav1 = p.at(i);
                    uav1dir = 1;
                } else {
                    uav2 = p.at(i);
                    uav2dir = 1;
                }
            } else {
                fug = p.at(i);
                fugdir = 1;
            }

        }

        p = gatac.senseEast(sizeX * sizeY);
        for (int i = 0; i < p.size(); i ++) {
            if (strncmp(p.at(i).first.c_str(), "UAV", 3) == 0) {
                if (uav1dir < 0) {
                    uav1 = p.at(i);
                    uav1dir = 2;
                } else {
                    uav2 = p.at(i);
                    uav2dir = 2;
                }
            } else {
                fug = p.at(i);
                fugdir = 2;
            }

        }

        p = gatac.senseWest(sizeX * sizeY);
        for (int i = 0; i < p.size(); i ++) {
            if (strncmp(p.at(i).first.c_str(), "UAV", 3) == 0) {
                if (uav1dir < 0) {
                    uav1 = p.at(i);
                    uav1dir = 3;
                } else {
                    uav2 = p.at(i);
                    uav2dir = 3;
                }
            } else {
                fug = p.at(i);
                fugdir = 3;
            }

        }



        int percept = 0;
        bool sameSquare = false;

        if (isFugitive) {

//            percept = (uav1dir << 2) + uav2dir;

            if (safehouseY > y) {
                percept = 0;
            } else if (safehouseY < y) {
                percept = 1;
            } else if(safehouseX >= x) {
                percept = 2;
            } else if(safehouseX <= x) {
                percept = 3;
            }

            if ((uav1.second == 0 && uav1dir >= 0) || (uav2.second == 0 && uav2dir >= 0))
                sameSquare = true;


        } else {
            percept = fugdir;
            if (fug.second == 0)
                sameSquare = true;
        }


        if (isFugitive && (safehouseX == x && safehouseY == y)) {
            gatac.landHere();
            gatac.sendScenarioIsOver("FUGITIVE ESCAPED");
            gatac.closeClient();
            return 0;

        }


        if (!isFugitive && sameSquare) {
            // we've caught the fugitive!
            gatac.sendScenarioIsOver("CAUGHT THE FUGITIVE");

            break;

        } else if (isFugitive && sameSquare) {
            // I've been caught!
            gatac.sendScenarioIsOver("I AM CAUGHT");

            break;

        }

        // move forward in tree

        if (curNode->children.size() == 0) {
            // ran out of tree!
            gatac.sendScenarioIsOver("POLICY ENDED");

            break;

        }
        curNode = curNode->children.at(percept);



	}


    gatac.land();

    // Close client socket connection.
    gatac.closeClient();


	return 0;
}

