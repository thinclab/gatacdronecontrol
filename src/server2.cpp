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
        std::cout << "This server/client pair provides more drone instruction and error testing."<<std::endl;
	std::cout << "Simply uncomment the test procedures to run them." << std::endl;

	// Instantiate GaTACDroneControl object
	GaTACDroneControl gatac;

	// Run drone server
	gatac.runServer(ip, port);
        
       
	return 0;
}

