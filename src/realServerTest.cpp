#include <iostream>
#include "GaTACDroneControlServer.hpp"

/*
 * Sample server code to demonstrate usage of the GaTACDroneControl API.
 */
int main() {
	// IP and port of client machine
	const char *ip = "127.0.0.1";
	int port = 4999;

        //cout message
        std::cout << "This server/client pair provides a simple check to link two real drones to a core, have them takeoff, then land."<<std::endl;


	// Instantiate GaTACDroneControl object
	const char* c = "r";
	GaTACDroneControl gatac(c);

	// Run drone server
	gatac.startServer(ip, port, 1);
        sleep(300);

	return 0;
}

