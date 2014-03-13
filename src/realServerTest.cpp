#include <iostream>
#include "GaTACDroneControl.hpp"

/*
 * Sample server code to demonstrate usage of the GaTACDroneControl API.
 */
int main() {
	// IP and port of client machine
	char *ip = "128.192.76.248";
	char *port = "4999";
       
        //cout message
        std::cout << "This server/client pair provides a simple check to link two real drones to a core, have them takeoff, then land."<<std::endl;


	// Instantiate GaTACDroneControl object
	const char* c = "r";
	GaTACDroneControl gatac(c);

	// Run drone server
	gatac.runServer(ip, port);
        
       
	return 0;
}

