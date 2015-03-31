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


    while (gatac.getClientReadyToCommand() == true && ! gatac.isScenarioOver()) {
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

    }

    return 0;

}
