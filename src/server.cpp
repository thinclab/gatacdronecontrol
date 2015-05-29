#include <iostream>
#include "GaTACDroneControlServer.hpp"

/*
 * Sample server code to demonstrate usage of the GaTACDroneControl API.
 */
int main(int argc, char ** argv) {
    if (argc < 3) {
        std::cerr << "Invalid arguments, correct format is: " << argv[0] << " mode(r/s) drone_count (rondevous port)" << std::endl;
        exit(1);
    }

    bool isReal = false;
    int drones, port;
    char * mode = argv[1];
    sscanf(argv[2], "%d", &drones);
    if (argc > 3)
        sscanf(argv[3], "%d", &port);
    else
        port = 4999;


    if (strncasecmp(mode, "r", 1) == 0) {
        isReal = true;
    }


	// IP and port of client machine
	const char *ip = "127.0.0.1";

        //cout message

	std::cout << "Server ready for clients." << std::endl;

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac(isReal);

	// Run drone server
	gatac.startServer(ip, port, drones);

	return 0;
}

