#include <iostream>
#include <unistd.h>
#include "GaTACDroneControlClient.hpp"

using std::cout;
using std::endl;
using std::flush;


// Taken from: http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
// could also conio.h if it's ok to depend on ncurses
#include <unistd.h>
#include <termios.h>

void print_status(int gridsize_x, int gridsize_y, int posX, int posY, string percept) {
    cout << endl << "Current Percept: " << percept << endl;

    for (int i = gridsize_y-1; i >=0 ; i --) {

        for (int j = 0; j < gridsize_x*2 + 1; j ++) {
            cout << "-";
        }
        cout << endl;

        for (int j = 0; j < gridsize_x*2 + 1; j ++) {
            if (j % 2 == 0)
                cout << "|";

            else if (posX == j/2 && posY == i)
                cout << "X";
            else
                cout << " ";
        }
        cout << endl;

    }
    for (int j = 0; j < gridsize_x*2 + 1; j ++) {
        cout << "-";
    }
    cout << endl << endl;

}

char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

// Hack to get around this SH*T language that doesn't have nested functions

#define CHECK_AND_MOVE( deltaX, deltaY)                             \
do {                                                                \
    int newX = x + deltaX;                                          \
    int newY = y + deltaY;                                          \
                                                                    \
    if (newX >= sizeX || newY >= sizeY || newX <0 || newY < 0) {    \
        cout << "ERROR, can't move there, out of bounds." << endl;  \
    } else {                                                        \
        x = newX;                                                   \
        y = newY;                                                   \
                                                                    \
        gatac.move(x, y);                                           \
    }                                                               \
} while(0)

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
	unsigned int port = 4999;

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac("KEYBOARDDRONE");

	// Launch Drone Client
	gatac.launchClient(ip, port);

	// Set grid size to [5 x 8]
	gatac.setGridSize(sizeX, sizeY);

	//set up drone
	gatac.setupDrone(x, y); // Spawn drone at (0, 0)

	// Sending ready message
	gatac.readyUp();

    string lastPercept = "None";

    while (gatac.getClientReadyToCommand() == true && ! gatac.isScenarioOver()) {

        print_status(sizeX, sizeY, x, y, lastPercept);
        char c;
        cout << "Next command? " << flush;
        c = getch();

        cout << c << endl << flush;

        switch (c) {
        case 'w':
            CHECK_AND_MOVE(0, 1);
            break;
        case 's':
            CHECK_AND_MOVE(0, -1);
            break;
        case 'a':
            CHECK_AND_MOVE(-1, 0);
            break;
        case 'd':
            CHECK_AND_MOVE(1, 0);
            break;
        case 'l':
            gatac.land();
            x = startX;
            y = startY;
            break;
        case 't':
            gatac.takeoff();
            break;
        case 'X':
            gatac.sendScenarioIsOver("User requested scenario end");
            break;
        default:
            cout << "Unrecognized command: " << c << endl;
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

        percept = fugdir;
        if (fug.second == 0)
            sameSquare = true;



        if (sameSquare) {
            // we've caught the fugitive!

            gatac.sendScenarioIsOver("CAUGHT THE FUGITIVE");

            break;

        }

        switch (percept) {
        case 0:
            lastPercept = "Enemy North";
            break;
        case 1:
            lastPercept = "Enemy South";
            break;
        case 2:
            lastPercept = "Enemy East";
            break;
        case 3:
            lastPercept = "Enemy West";
            break;
        default:
            lastPercept = "Unknown";
            break;
        }

    }

    return 0;

}
