#include <iostream>
#include "GaTACDroneControlServer.hpp"

/*
 * Sample server code to demonstrate usage of the GaTACDroneControl API.
 */
int main() {
	// IP and port of client machine
	const char *ip = "128.192.76.247";
	const int port = 4999;

        //cout message
        std::cout << "This server/client pair provides movement and shared cell checking, as well as error cases commented out at the bottom of the client."<<std::endl;


	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac;

	// Run drone server
	gatac.startServer(ip, port, 3);

	return 0;
}

