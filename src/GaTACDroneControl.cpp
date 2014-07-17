#include <iostream>
#include <errno.h> // For printing error #'s/debugging
#include <stdlib.h>
#include <stdio.h>
#include <sstream> // For converting chars to ints to update positions vector
#include <fstream> // For editing files
#include <boost/thread.hpp> // For concurrent flight
#include <boost/date_time.hpp>

// For tokenizing command input
#include <sstream>

// GaTACDroneControl header
#include "GaTACDroneControl.hpp"

using namespace std;

#define BUFLEN 256
#define DEFAULTCLIENTPORT1 4999
#define DEFAULTCLIENTPORT2 5999
#define DEFAULTCLIENTPORT3 6999
#define DEFAULTDATAPORT1 4998
#define DEFAULTDATAPORT2 5998
#define DEFAULTDATAPORT3 6998

/**
 * @file	GaTACDroneControl.cpp
 * @author  	Vince Capparell, Casey Hetzler
 * @version	1.0
 *
 * @section BRIEF A program designed to allow one server to control 1-3 quadcopter clients
 * in a grid-based environment, providing various functions such as movement, communication of navdata, and the spotting of other drones.
 *
 * @section LICENSE
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/licenses/quick-guide-gplv3.html
 *
 * @section DESCRIPTION
 * GaTACDroneControl allows grid-based movement, facilitated with client-server communication, for up to three (3) "Parrot AR.Drone" drone clients and one (1) multi-threaded
 * server process.
 * Made for cooperative use with UGA THINC Lab's "ardrone_thinc" package and Autonomy Lab's "ardrone_autonomy" package.
 */

//defining static data members for navdata, unique per client
string GaTACDroneControl::clientCurrentBattery;
string GaTACDroneControl::clientCurrentSonar;
string GaTACDroneControl::clientCurrentForwardVelocity;
string GaTACDroneControl::clientCurrentSidewaysVelocity;
string GaTACDroneControl::clientCurrentVerticalVelocity;
string GaTACDroneControl::clientCurrentTagsSpotted;

/**
 * Default constructor. Initializes all member variables.
 * If no char provided to constructor, this gatac object will be used as a server or client object involving SIMULATED drones.
 */
GaTACDroneControl::GaTACDroneControl() {
	serverSocket, dataSocket, numberOfColumns, numberOfRows, numberOfDrones = 0;
	gridSizeSet, gridStarted = false;
	simulatorMode = true;
	datsrv = NULL;
	srv = NULL;
	serverThreads = 0;
	readyForData = false;


}

/**
 * Overloaded constructor. Used when flying real drones as opposed to the simulator. All members initialized, with bool simulatorMode init'd to false.
 * @param c If char provided to constructor, this gatac object will be used as a server or client object involving REAL drones.
 */
GaTACDroneControl::GaTACDroneControl(const char* c) {
	serverSocket, dataSocket, numberOfColumns, numberOfRows, numberOfDrones = 0;
	gridSizeSet, gridStarted = false;
	simulatorMode = false;
	datsrv = NULL;
	srv = NULL;
	serverThreads = 0;
	readyForData = false;
}

/**
 * This method is called by the GaTAC server. It begins a new server thread for each drone started.
 * @param remoteIP The IP supplied for a client command socket
 * @param remotePort The port number supplied for a client command socket, by default 4999, 5999, and 6999
 * @param expectedDrones The number of drones expected for this flight/server session
 */
void GaTACDroneControl::startServer(const char *remoteIP, unsigned int remotePort, int expectedDrones){
	cout << "Main server running." << endl;

	this->expectedDrones = expectedDrones;
	for (int i = 0; i < expectedDrones; i ++) {
        clientsReady.push_back(false);
        dronesReady.push_back(false);
        dronesSharingSpace.push_back(false);
	}

	for(int i = 0; i < expectedDrones; i++)
	{
		serverThreads++;
		boost::thread* thread;
		thread = new boost::thread(boost::bind(&GaTACDroneControl::runServer,this,remoteIP,remotePort, serverThreads));
		threads[2*i] = thread;
		cout<<"starting thread "<<serverThreads <<endl;

		serverThreads++;
		remotePort ++;
		thread = new boost::thread(boost::bind(&GaTACDroneControl::dataServer,this,remoteIP,remotePort, serverThreads));
		threads[2*i + 1] = thread;
		cout<<"starting data thread "<<serverThreads<<endl;
		remotePort ++;
	}

}
/**
 * This method sets up the data socket for each client and listens for navdata requests. It loops continuously, updating the navdata for each client navdata data members.
 * @param remoteIP The IP supplied for a client data socket
 * @param remotePort The port number supplied for a client data socket, by default 4998, 5998, and 6998
 * @param threadNo The ID of the thread this method is starting
 */
void GaTACDroneControl::dataServer(const char *remoteIp, unsigned int remotePort, int threadNo) {
	char localport[5];
	int errorCheck, datsock;
	struct addrinfo dathints, *datsrv, *datinfo;
	struct sockaddr_storage client_addr;
	socklen_t addr_len = sizeof client_addr;

	sprintf(localport, "%d", remotePort);

	// Specifying socket parameters
	bzero(&dathints, sizeof dathints);
	dathints.ai_family = AF_UNSPEC;
	dathints.ai_socktype = SOCK_DGRAM;
	dathints.ai_flags = AI_PASSIVE; // use my IP

	// Filling 'srv' object with info from 'hints'
	if ((errorCheck = getaddrinfo(NULL, localport, &dathints, &datsrv)) != 0) {
		perror("Server: get address info");
		exit(1);
	}

	// Creating and binding socket.
	for (datinfo = datsrv; datinfo != NULL; datinfo = datinfo->ai_next) {
		if ((datsock = socket(datinfo->ai_family, datinfo->ai_socktype, datinfo->ai_protocol)) == -1) {
			perror("Server: socket");
			continue;
		}

		if (bind(datsock, datinfo->ai_addr, datinfo->ai_addrlen) == -1) {
			close(datsock);
			perror("Server: bind");
			continue;
		}

		break;
	}

	// Ensuring valid address info was found
	if (datinfo == NULL) {
		perror("Server: no valid address info found.\n");
		exit(1);
	}


	//Storing data socket info for later use
	dataSocket = datsock;
	this->datsrv = datinfo;


	// Loop forever. Read commands from socket and perform the action specified.
	int bytesReceived = 0;
	while (1) {
		char receiveBuffer[BUFLEN];
		char publishMessage[BUFLEN];

		if ((bytesReceived = recvfrom(datsock, receiveBuffer, BUFLEN, 0, (struct sockaddr *) &client_addr, &addr_len)) == -1) {
			perror("Error receiving command.");
			exit(1);
		}
		receiveBuffer[bytesReceived] = '\0';

		// Splitting command input into tokens by whitespace
		string buffer;
		vector<string> tokens;
		string stringCommand(receiveBuffer);
		stringstream ss(stringCommand);

		// Storing tokens in vector
		while (ss >> buffer) {
			tokens.push_back(buffer);
		}

		char rawCommand = receiveBuffer[0];
		switch (rawCommand) {
		case 'n':
			break;

		default:
			cout << "Error parsing nav command - invalid command character received." << endl;
			break;
		}

		// Sending acknowledgment message to client
		int len = BUFLEN;
		char sendBuffer[len];
		char navBuffer[BUFLEN];
		strcpy(sendBuffer, getData(0));
		//If command received was to spawn a drone, first char of ACK set to id
		if(rawCommand == 'n'){
		int numSent = 0;
		sleep(5);
		if ((numSent = sendto(datsock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
			perror("Server: error sending acknowledgment.");
			exit(1);
		}
		}
		else{
		int numSent = 0;
		if ((numSent = sendto(datsock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
			perror("Server: error sending acknowledgment.");
			exit(1);
		}
		}
	}
	// Cleaning up socket information
	freeaddrinfo(datinfo);
	freeaddrinfo(datsrv);
	close(datsock);
}

/**
 * This method sets up the main UDP socket server. It loops continuously, parsing the input
 * received from the UDP socket and launching the correct ROS services on the machine it's running on.
 * The machine running this main server must therefore have all necessary ROS packages installed.
 * @param remoteIP The IP supplied for a client socket
 * @param remotePort The port number supplied for a client socket
 * @param threadNo The ID of the thread this method is starting
 */
void GaTACDroneControl::runServer(const char *remoteIp, unsigned int remotePort, int threadNo) {
	const char *publishCommand = "rostopic pub -1 /drone%s/ardrone/%s std_msgs/Empty&";
	const char *serviceCall = "rosservice call /drone%d/%s";

	char localport[5];
	int errorCheck, sock;
	struct addrinfo hints, *srv, *info;
	struct sockaddr_storage client_addr;
	socklen_t addr_len = sizeof client_addr;

	sprintf(localport, "%d", remotePort);

	// Specifying socket parameters
	bzero(&hints, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE; // use my IP

	// Filling 'srv' object with info from 'hints'
	if ((errorCheck = getaddrinfo(NULL, localport, &hints, &srv)) != 0) {
		perror("Server: get address info");
		exit(1);
	}

	// Creating and binding socket.
	for (info = srv; info != NULL; info = info->ai_next) {
		if ((sock = socket(info->ai_family, info->ai_socktype, info->ai_protocol)) == -1) {
			perror("Server: socket");
			continue;
		}

		if (bind(sock, info->ai_addr, info->ai_addrlen) == -1) {
			close(sock);
			perror("Server: bind");
			continue;
		}

		break;
	}

	// Ensuring valid address info was found
	if (info == NULL) {
		perror("Server: no valid address info found.\n");
		exit(1);
	}



	// Loop forever. Read commands from socket and perform the action specified.
	int bytesReceived = 0;
	int myDroneId = -1;
	while (1) {
		const char *droneNumber, *x, *y;
		string temp;
		string errorMessage = "none";
		string invalidDroneId = "No drone with ID has been spawned. Please specify a valid drone ID.";
       		string invalidLocation = "Location entered is beyond the grid parameters.";
		char receiveBuffer[BUFLEN];
		char publishMessage[BUFLEN];
		int initialColumn, initialRow;
		int droneInt;
		int sleepCtr; // to ensure drones don't send commands before server can process, and drones begin in sync
		bool allReady = false;
		int xInt, yInt, droneNumberInt = 0;
		char * senseOption;
		int senseResult, senseInt;
		const char *navDataToSend = ""; // Holds string of navdata server will send to client on request
		std::stringstream strX;
		std::stringstream strY;
		std::stringstream strID;


		cout << "Waiting for a command..." << endl;
		if ((bytesReceived = recvfrom(sock, receiveBuffer, BUFLEN, 0, (struct sockaddr *) &client_addr, &addr_len)) == -1) {
			perror("Error receiving command.");
			exit(1);
		}
		receiveBuffer[bytesReceived] = '\0';

		cout << "Command Received!----> ";

		// Splitting command input into tokens by whitespace
		string buffer;
		string stringCommand(receiveBuffer);
		stringstream ss(stringCommand);

		std::istream_iterator<std::string> begin(ss);
		std::istream_iterator<std::string> end;

		// Storing tokens in vector
		vector<string> tokens(begin, end);

		char rawCommand = receiveBuffer[0];
		switch (rawCommand) {
		case 's':

			cout << "Spawn drone." << endl;
			initialColumn = atoi((tokens.at(1)).c_str());
			initialRow = atoi((tokens.at(2)).c_str());
			// If grid size hasn't been set
			if (gridSizeCheck() == false) {
			errorMessage = "No grid size set. You must specify a grid size before spawning a drone.";
			}
			else if (maxDrones()) {
			errorMessage = "The requested number of drones has already been spawned";
			}
			// If start position isn't valid
			else if (validLocation(initialRow, initialColumn) == false) {
			errorMessage = "The starting location you specified does not lie within the grid. Please choose a valid starting location.";
			}

			if (errorMessage != "none") {
				printf("Error: couldn't spawn drone. %s\n", errorMessage.c_str());
				exit(1);
			}
			else{
			/* If server passes all checks, client message processed */
			dronePositions.push_back(make_pair(initialColumn, initialRow));
			printf("Ready to spawn drone at [%d, %d].\n", initialColumn, initialRow);
			myDroneId = numberOfDrones;
			numberOfDrones++;
			}
			break;

		case 't':
			cout << "Take off." << endl;
			droneNumber = (tokens.at(1)).c_str();
			droneInt = atoi(droneNumber);
			// If droneID isn't valid
			if (validDroneId(droneInt) == false) {
			printf("Error: No drone with ID %s has been spawned.  Please specify a valid drone ID.\n", droneNumber);
			exit(1);
			}
			else if (dronesReady.at(myDroneId)) {
                printf("Error: Drone ID %s is already flying.\n", droneNumber);
			}
			else{
			/* If server passes all checks, client message processed */
			sprintf(publishMessage, serviceCall, droneInt, "takeoff_thinc_smart");
			system(publishMessage);

			dronesReady.at(myDroneId) = true;

			}
			break;

		case 'l':
			cout << "Land." << endl;
			droneNumber = (tokens.at(1)).c_str();
			droneInt = atoi(droneNumber);
			if (validDroneId(droneInt) == false) {
			printf("Error: No drone with ID %s has been spawned.  Please specify a valid drone ID.\n", droneNumber);
			exit(1);
			}
			else if (!dronesReady.at(myDroneId)) {
                printf("Error: Drone ID %s is not flying.\n", droneNumber);
			}
			else{
			/* If server passes all checks, client message processed */
			sprintf(publishMessage, serviceCall, droneInt, "land_at_home");
			system(publishMessage);

			dronesReady.at(myDroneId) = false;

			// Delete drone's navdata file
			if(droneInt == 0)
				remove("currentNavdata0.txt");
			else if(droneInt == 1)
				remove("currentNavdata1.txt");
			else if(droneInt == 2)
				remove("currentNavdata2.txt");
			}
			break;

		case 'h':
			cout << "Hover." << endl;
			droneNumber = (tokens.at(1)).c_str();
			strID << droneNumber;
			strID >> droneNumberInt;
		/* Now the server checks if the drone ID and location entered are valid */
			// If grid hasn't been started
			if (gridStartCheck() == false) {
			errorMessage = "The grid has not yet been started. Grid must be started before sending commands to a drone.";
			}
			// If drone ID isn't valid
			else if (validDroneId(droneNumberInt) == false) {
			errorMessage = invalidDroneId;
			}
		/* If server passes checks, client message processed */
			else{
			int x = dronePositions.at(droneNumberInt).first;
			int y = dronePositions.at(droneNumberInt).second;
			moveAndCheck(x, y, droneNumberInt);
			}
		//end moveAndCheck
			break;

		case 'r':
			cout << "Reset." << endl;
			droneNumber = (tokens.at(1)).c_str();
			droneInt = atoi(droneNumber);
			if (validDroneId(droneInt) == false) {
			printf("Error: No drone with ID %s has been spawned.  Please specify a valid drone ID.\n", droneNumber);
			exit(1);
			}
			else{
			/* If server passes all checks, client message processed */
			sprintf(publishMessage, publishCommand, droneInt, "reset");
			system(publishMessage);
			sleep(3); // Wait for drone to reset
			}
			break;

		case 'm':
			cout << "Move." << endl;
			droneNumber = (tokens.at(1)).c_str();
			x = (tokens.at(2)).c_str();
			y = (tokens.at(3)).c_str();
		//Desired coordinates & drone ID sent to moveAndCheck, where movement messages are sent one move at a time and positions are checked after each move using the
		//sharedSpace method
			strX << x;
			strX >> xInt;
			strY << y;
			strY >> yInt;
			strID << droneNumber;
			strID >> droneNumberInt;
		/* Now the server checks if the drone ID and location entered are valid */
			// If grid hasn't been started
			if (gridStartCheck() == false) {
			errorMessage = "The grid has not yet been started. Grid must be started before sending commands to a drone.";
			}
			// If drone ID isn't valid
			else if (validDroneId(droneNumberInt) == false) {
			errorMessage = invalidDroneId;
			}
			// If destination isn't valid
			else if (validLocation(xInt, yInt) == false) {
				errorMessage = invalidLocation;
			}
			if (errorMessage != "none") {
			printf("Error: couldn't move drone. %s\n", errorMessage.c_str());
			}
		/* If server passes all checks, client message processed */
			else{
			moveAndCheck(xInt, yInt, droneNumberInt);
			}
		//end moveAndCheck
			break;

		case 'g':
			cout << "Set grid size." << endl;
			numberOfColumns = atoi(tokens.at(1).c_str());
			numberOfRows = atoi(tokens.at(2).c_str());
			// If size has already been set
			if (gridSizeCheck() == true) {
			cout << "Error: The grid size has already been set to ["<<numberOfColumns<<" x "<<numberOfRows<<"]. Specify grid size only once." << endl;
			}
			// If size isn't valid
			 if (validGridSize(numberOfRows, numberOfColumns) == false) {
			cout << "Error: The grid size specified was too large. The maximum grid size is 10x10." << endl;
			exit(1);
			}
			/* If server passes all checks, client message processed */
			else{
			gridSizeSet = true;
			this->numberOfColumns = numberOfColumns;
			this->numberOfRows = numberOfRows;
			}
			break;

			//When running multi clients, have each use readyUp() to start the grid
		case 'y':

            clientsReady.at(myDroneId) = true;

			allReady = true;
			for(int i=0; i < clientsReady.size(); i++)
			{
                if(clientsReady.at(i) == false){
                    allReady = false;
                }
			}
			if(allReady == true){
                readyForData = true;
                // If grid size has been set
                if (this->gridSizeCheck() == true && this->gridStartCheck() == false) {
                    cout << "All clients ready. Starting grid!" << endl;
                    cout << "Waiting a few seconds before server will receive commands..." << endl;
                    launchGrid();
                    gridStarted = true;

                }
                // If grid has already been started
                else if(this->gridStartCheck() == true) {
                    cout << "Error: Grid already started." << endl;
                }
                //If grid size has not been set
                else if(this->gridSizeCheck() == false) {
                    cout << "Error: No grid size set. You must specify a grid size before starting the grid." << endl;
                    exit(1);
                }
			}
			break;

			//Kind of deprecated, usable only for single client start
		case 'i':
			/*cout << "Start gazebo." << endl;
			// If grid size has been set
			if (this->gridSizeCheck() == true && this->gridStartCheck() == false) {
			launchGrid();
			gridStarted = true;
			}
			// If grid has already been started
			else if(this->gridStartCheck() == true) {
			cout << "Error: Grid already started." << endl;
			}
			//If grid size has not been set
			else if(this->gridSizeCheck() == false) {
			cout << "Error: No grid size set. You must specify a grid size before starting the grid." << endl;
			exit(1);
			}*/
			break;

		case 'u':
			cout << "Sense North" << endl;
			droneNumber = (tokens.at(1)).c_str();
			droneInt = atoi(droneNumber);
			senseInt = 0;
			senseResult = sense(droneInt, senseInt);
  			break;

		case 'd':
			cout << "Sense South" << endl;
			droneNumber = (tokens.at(1)).c_str();
			droneInt = atoi(droneNumber);
			senseInt = 1;
			senseResult = sense(droneInt, senseInt);
  			break;

		case 'k':
			cout << "Sense East" << endl;
			droneNumber = (tokens.at(1)).c_str();
			droneInt = atoi(droneNumber);
			senseInt = 2;
			senseResult = sense(droneInt, senseInt);
  			break;

		case 'j':
			cout << "Sense West" << endl;
			droneNumber = (tokens.at(1)).c_str();
			droneInt = atoi(droneNumber);
			senseInt = 3;
			senseResult = sense(droneInt, senseInt);
  			break;

			//Default case, for command characters that are undefined
		default:
			cout << "Error parsing raw command - invalid command character received." << endl;
			break;
		}

		// Sending acknowledgment message to client
		int len = BUFLEN;
		char sendBuffer[len];
		char navBuffer[BUFLEN];
		strcpy(sendBuffer, receiveBuffer);
		//If command received was to spawn a drone, first char of ACK set to id
		if(rawCommand == 's'){
			sprintf(sendBuffer, "%d %s", myDroneId, receiveBuffer);
            int numSent = 0;
            if ((numSent = sendto(sock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
                perror("Server: error sending acknowledgment.");
                exit(1);
            }
		}
		//If command received was ready up command, server waits for other drones or starts grid
		else if(rawCommand == 'y'){
            cout<<"Waiting for other drones..." <<endl;

            while (! this->gridStartCheck() ) {
                sleep(0.5);
            }

			sprintf(publishMessage, serviceCall, myDroneId, "takeoff_thinc_smart");
			system(publishMessage);
			dronesReady.at(myDroneId) = true;

            while (! this->droneStartCheck() ) {
                sleep(0.25);
            }

            int numSent = 0;
            if ((numSent = sendto(sock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
                perror("Server: error sending acknowledgment.");
                exit(1);
            }
            cout<<"All clients ready, sending reply to " << myDroneId << endl;

		}
		//If command received was a sense command, first char of ACK set to sense return integer
		else if(rawCommand == 'u' || rawCommand == 'j' || rawCommand == 'k' || rawCommand == 'd'){
		char senseChar = (char)(((int)'0')+senseResult);
		sendBuffer[0] = senseChar;
		int numSent = 0;
			if ((numSent = sendto(sock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
			perror("Server: error sending acknowledgment.");
			exit(1);
			}
		}
		else{
		int numSent = 0;
		if ((numSent = sendto(sock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
			perror("Server: error sending acknowledgment.");
			exit(1);
		}
		}
	}
	// Cleaning up socket information
	freeaddrinfo(info);
	freeaddrinfo(srv);
	close(sock);
}


bool GaTACDroneControl::droneStartCheck() {
    for(int i=0; i < dronesReady.size(); i++)
    {
        if(dronesReady.at(i) == false){
            return false;
        }
    }

    return true;
}

/**
 * This method sets up the main UDP socket client. Once created, all relevant socket information
 * is then stored in the current instance of GaTACDroneControl for later communication with the server.
 * @param serverIp The IP supplied for the server's socket
 * @param serverPort The port number supplied for the server's socket
 * @param dataPort The port number supplied for the client's navdata socket
 */
void GaTACDroneControl::launchClient(char *serverIp, unsigned int serverPort, unsigned int dataPort) {
	char *host = serverIp;
	char port[5];
	char dp[5];

        sprintf(port, "%d", serverPort);
        sprintf(dp, "%d", dataPort);

	int errorCheck, sock, datsock;

	struct addrinfo hints, *srv, *info;
	struct addrinfo dathints, *datsrv, *datinfo;


	// Specifying socket parameters
	bzero(&hints, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;

	// Filling 'srv' object with info from 'hints'
	if ((errorCheck = getaddrinfo(host, port, &hints, &srv)) != 0) {
		perror("Client: get address info function.");
		exit(1);
	}

	// Creating socket
	for (info = srv; info != NULL; info = info->ai_next) {
		if ((sock = socket(info->ai_family, info->ai_socktype, info->ai_protocol)) == -1) {
			perror("Client: socket function.");
			continue;
		}

		break;
	}

	// Ensuring valid address info was found
	if (info == NULL) {
		perror("Client: no valid address info found.\n");
		exit(1);
	}

	// Storing server socket data for later
	serverSocket = sock;
	this->srv = info;


	// Specifying socket parameters
	bzero(&dathints, sizeof dathints);
	dathints.ai_family = AF_UNSPEC;
	dathints.ai_socktype = SOCK_DGRAM;

	// Filling 'srv' object with info from 'hints'
	if ((errorCheck = getaddrinfo(host, dp, &dathints, &datsrv)) != 0) {
		perror("Client: get address info function.");
		exit(1);
	}

	// Creating socket
	for (datinfo = datsrv; datinfo != NULL; datinfo = datinfo->ai_next) {
		if ((datsock = socket(datinfo->ai_family, datinfo->ai_socktype, datinfo->ai_protocol)) == -1) {
			perror("Client: socket function.");
			continue;
		}

		break;
	}

	// Ensuring valid address info was found
	if (datinfo == NULL) {
		perror("Client: no valid address info found.\n");
		exit(1);
	}

	// Storing server socket data for later
	dataSocket = datsock;
	this->datsrv = datinfo;

	// Sleep to ensure that the connection is complete before commands are sent
	sleep(3);
}

/**
 * This method will start the drone simulator with size and number/location of drones
 * as specified by previous method calls.
 */
void GaTACDroneControl::startGrid() {
	bool worked = false;
		cout << "Sending command to start grid." << endl;
		char message[2] = "i";
		worked = sendMessage(message, serverSocket, srv);
}

/**
 * This method is called by a client to send a ready message in multi-client environments.
 * When a server has received one from each client, it makes the decision to start the grid.
 */
void GaTACDroneControl::readyUp() {
	bool worked = false;
		cout << "Sending ready message to server." << endl;
		char message[2] = "y";
		worked = sendMessage(message, serverSocket, srv);
}

/**
 * This method is called by a client to send a senseNorth message to the server.
 * @param droneId Integer denoting which drone is calling the method
 */
void GaTACDroneControl::senseNorth(int droneId) {
	printf("Sending sense north command, drone #%d.\n", droneId);
	// Send command to server
	bool worked = commandDrone('u', droneId);
	if (!worked) {
		cout << "Couldn't push sense request. Please try again." << endl;
		exit(1);
	}
}

/**
 * This method is called by a client to send a senseSouth message to the server.
 * @param droneId Integer denoting which drone is calling the method
 */
void GaTACDroneControl::senseSouth(int droneId) {
	printf("Sending sense south command, drone #%d.\n", droneId);
	// Send command to server
	bool worked = commandDrone('d', droneId);
	if (!worked) {
		cout << "Couldn't push sense request. Please try again." << endl;
		exit(1);
	}
}

/**
 * This method is called by a client to send a senseEast message to the server.
 * @param droneId Integer denoting which drone is calling the method
 */
void GaTACDroneControl::senseEast(int droneId) {
	printf("Sending sense east command, drone #%d.\n", droneId);
	// Send command to server
	bool worked = commandDrone('k', droneId);
	if (!worked) {
		cout << "Couldn't push sense request. Please try again." << endl;
		exit(1);
	}
}

/**
 * This method is called by a client to send a senseWest message to the server.
 * @param droneId Integer denoting which drone is calling the method
 */
void GaTACDroneControl::senseWest(int droneId) {
	printf("Sending sense west command, drone #%d.\n", droneId);
	// Send command to server
	bool worked = commandDrone('j', droneId);
	if (!worked) {
		cout << "Couldn't push sense request. Please try again." << endl;
		exit(1);
	}
}

/**
 * This method sets up the size of the grid that all subsequently spawned drones will be spawned on.
 * @param numberOfColumns X-axis dimension
 * @param numberOfRows Y-axis dimension
 */
void GaTACDroneControl::setGridSize(int numberOfColumns, int numberOfRows) {
	bool worked = false;
	// Send command to server
		printf("Sending command to set grid size to %dx%d.\n", numberOfColumns, numberOfRows);

		char message[32];
		sprintf(message, "g %d %d", numberOfColumns, numberOfRows);
		worked = sendMessage(message, serverSocket, srv);

		if (!worked) {
			cout << "Couldn't set the grid size. Please try again." << endl;
			exit(1);
		}
}

/**
 * This method is called by the client and will move the specified drone to the desired (x, y) position.
 * @param droneId ID of drone to move
 * @param x Drone's desired position, X-axis
 * @param y Drone's desired position, Y-axis
 */
void GaTACDroneControl::move(int droneId, int x, int y) {
	bool worked = false;
	// Send command to server, checks for valid ID and location done server-side
		printf("Sending command to move drone #%d to (%d, %d).\n", droneId, x, y);
		char message[32];
		sprintf(message, "m %d %d %d", droneId, x, y);
		worked = sendMessage(message, serverSocket, srv);
}

/**
 * This method is called by the client and allows a drone to hover.
 * @param droneId ID of drone to hover
 */
void GaTACDroneControl::hover(int droneId) {

	// Send command to server
	bool worked = commandDrone('h', droneId);
	printf("Sending command to make drone hover.\n", droneId);
		char message[32];
		sprintf(message, "h %d", droneId);
		worked = sendMessage(message, serverSocket, srv);
}

/**
 * This method is called by the client and will land the specified drone.
 * @param droneId ID of drone to land
 */
void GaTACDroneControl::land(int droneId) {
	printf("Sending command to land drone #%d.\n", droneId);

	// Send command to server
	bool worked = commandDrone('l', droneId);
	if (!worked) {
		cout << "Couldn't land drone. Please try again." << endl;
		exit(1);
	}
}

/**
 * This method is called by the client and will make the specified drone take off.
 * @param droneId ID of drone to takeoff
 */
void GaTACDroneControl::takeoff(int droneId) {
	printf("Sending command to takeoff drone #%d.\n", droneId);

	// Send command to server
	bool worked = commandDrone('t', droneId);
	if (!worked) {
		cout << "Couldn't take off. Please try again." << endl;
		exit(1);
	}
}

/**
 * This method is called by the client and will trigger the reset mode for the specified drone.
 * @param droneId ID of drone to reset
 */
void GaTACDroneControl::reset(int droneId) {
	printf("Sending command to reset drone #%d.\n", droneId);

	// Send command to server
	bool worked = commandDrone('r', droneId);
	if (!worked) {
		cout << "Couldn't reset drone. Please try again." << endl;
		exit(1);
	}
}

/**
 * This method sets up a new drone. The size of the grid and initial
 * position of the drone on that grid must be specified.
 * @param droneCol Drone's initial position, X-axis
 * @param droneRow Drone's intiial position, Y-axis
 */
void GaTACDroneControl::setupDrone(int droneCol, int droneRow) {
	bool worked = false;
	// Send command to server
		printf("Sending command to spawn drone at (%d, %d).\n", droneCol, droneRow);
		char msg[10];
		sprintf(msg, "s %d %d", droneCol, droneRow);
		worked = sendMessage(msg, serverSocket, srv);
}

/**
 * This method closes the UDP client socket, as well as the navdata socket, and sets the client's readyToCommand boolean to false.
 */
void GaTACDroneControl::closeClient() {
	close(serverSocket);
	close(dataSocket);
	this->setClientReadyToCommand(false);
	freeaddrinfo(srv);
}

/**
 * Used to set a client's unique drone ID
 * @param toSet Integer to set this GaTAC instance's drone ID to (0, 1, or 2)
 */
void GaTACDroneControl::setClientUniqueId(int toSet)
{
 this->clientUniqueId = toSet;
}

/**
 * This method will set the GaTAC clientCurrentBattery data member to the correct navdata value.
 * @param toSet string to set battery to
 */
void GaTACDroneControl::setBattery(string toSet)
{
 this->clientCurrentBattery = toSet;
}

/**
 * This method will set the GaTAC clientCurrentSonar data member to the correct navdata value..
 * @param toSet string to set sonar to
 */
void GaTACDroneControl::setSonar(string toSet)
{
 this->clientCurrentSonar = toSet;
}

/**
 * This method will set the GaTAC clientCurrentTagsSpotted data member to the correct navdata value.
 * @param toSet string to set tags spotted data to
 */
void GaTACDroneControl::setTagsSpotted(string toSet)
{
 this->clientCurrentTagsSpotted = toSet;
}

/**
 * This method will set the GaTAC clientCurrentForwardVelocity data member to the correct navdata value.
 * @param toSet string to set forward velocity to
 */
void GaTACDroneControl::setForwardVelocity(string toSet)
{
 this->clientCurrentForwardVelocity = toSet;
}

/**
 * This method will set the GaTAC clientCurrentSidewaysVelocity data member to the correct navdata value.
 * @param toSet string to set sideways velocity to
 */
void GaTACDroneControl::setSidewaysVelocity(string toSet)
{
 this->clientCurrentSidewaysVelocity = toSet;
}

/**
 * This method will set the GaTAC clientCurrentVerticalVelocity data member to the correct navdata value.
 * @param toSet string to set vertical velocity to
 */
void GaTACDroneControl::setVerticalVelocity(string toSet)
{
 this->clientCurrentVerticalVelocity = toSet;
}

/**
 * Used to get a client's unique drone ID
 * @return Client's unique drone ID
 */
int GaTACDroneControl::getClientUniqueId()
{
 return this->clientUniqueId;
}

/**
 * This method will return and print the current battery percentage to the client's display.
 * @return Returns the current battery value in a human-readable string
 */
string GaTACDroneControl::getBattery()
{
 return this->clientCurrentBattery;
}

/**
 * This method will return and print the current sonar reading to the client's display.
 * @return Returns the current sonar value in a human-readable string
 */
string GaTACDroneControl::getSonar()
{
 return this->clientCurrentSonar;
}

/**
 * This method will return and print the current number of tags spotted to the client's display.
 * @return Human-readable string of current tag data
 */
string GaTACDroneControl::getTagsSpotted()
{
 return this->clientCurrentTagsSpotted;
}

/**
 * This method will return and print the current forward velocity to the client's display.
 * @return Returns the current forward velocity value in a human-readable string
 */
string GaTACDroneControl::getForwardVelocity()
{
 return this->clientCurrentForwardVelocity;
}

/**
 * This method will return and print the current sideways velocity to the client's display.
 * @return Returns the current sideways velocity value in a human-readable string
 */
string GaTACDroneControl::getSidewaysVelocity()
{
 return this->clientCurrentSidewaysVelocity;
}

/**
 * This method will return and print the current vertical velocity to the client's display.
 * @return Returns the current vertical velocity value in a human-readable string
 */
string GaTACDroneControl::getVerticalVelocity()
{
 return this->clientCurrentVerticalVelocity;
}

/**
 * Used to set a client's "readyForCommands" boolean value
 * @param toSet Boolean specifies whether or not client is ready to receive server commands
 */
void GaTACDroneControl::setClientReadyToCommand(bool toSet)
{
 this->readyToCommand = toSet;
}

/**
 * Used to get a client's "readyForCommands" boolean value
 * @return Boolean value that tells whether a client is ready to receive commands
 */
bool GaTACDroneControl::getClientReadyToCommand()
{
 return this->readyToCommand;
}

/*
 * Private Methods
 */
bool GaTACDroneControl::receiveData(int id)
{
	char message[3];
	sprintf(message, "n %d", id);
	bool success = false;
	char sendBuffer[BUFLEN];
	char receiveBuffer[BUFLEN] = { };
	strcpy(sendBuffer, message);
	char cmdCheck = sendBuffer[0];
	string bats = "";
	string fors = "";
	string sides = "";
	string verts = "";
	string sons = "";
	string tags = "";
	while(1){
	// Sending message
	int bytesSent = 0;
	if ((bytesSent = sendto(dataSocket, sendBuffer, BUFLEN, 0, datsrv->ai_addr, datsrv->ai_addrlen)) == -1) {
		cout << "Error sending message to server with errno: " << errno << endl;
		success = false;
	}

	// Waiting for feedback (ensuring that the server received our message)
	int bytesReceived = 0;
	if ((bytesReceived = recvfrom(dataSocket, receiveBuffer, BUFLEN, 0, NULL, NULL)) == -1) {
		cout << "Error receiving feedback from server." << endl;
		exit(1);
	} else {
		success = true;
		for(int i = 0; i < 30; i++)
		bats += receiveBuffer[i];
		for(int i = 30; i < 60; i++)
		fors += receiveBuffer[i];
		for(int i = 60; i < 90; i++)
		sides += receiveBuffer[i];
		for(int i = 90; i < 120; i++)
		verts += receiveBuffer[i];
		for(int i = 120; i < 150; i++)
		sons += receiveBuffer[i];
		for(int i = 150; i < 180; i++)
		tags += receiveBuffer[i];


		this->setBattery(bats);
		this->setForwardVelocity(fors);
		this->setSidewaysVelocity(sides);
		this->setVerticalVelocity(verts);
		this->setSonar(sons);
		this->setTagsSpotted(tags);

		bats.clear();
		fors.clear();
		sides.clear();
		verts.clear();
		sons.clear();
		tags.clear();
		}
	sleep(0.25);
	}
return success;
}


/**
 * This method sends the message specified through the main UDP socket
 * to whatever machine is currently running the drone server. Returns a bool value depending on
 * whether the specified destination received the message and sent an acknowledgement back.
 * @param message The message to be sent, as a string of characters
 * @param socket The socket to send to
 * @param addrinfo A struct of socket address information
 * @return Boolean value true if message successfully sent and echoed, false if there is a miscommunication
 */
bool GaTACDroneControl::sendMessage(char *message, int socket, struct addrinfo *addrInfo) {
	bool success = false;
	char sendBuffer[BUFLEN];
	char receiveBuffer[BUFLEN] = { };
	strcpy(sendBuffer, message);
	char cmdCheck = sendBuffer[0];

	// Sending message
	int bytesSent = 0;
	if ((bytesSent = sendto(socket, sendBuffer, BUFLEN, 0, addrInfo->ai_addr, addrInfo->ai_addrlen)) == -1) {
		cout << "Error sending message to server with errno: " << errno << endl;
		success = false;
	}

	// Waiting for feedback (ensuring that the server received our message)
	int bytesReceived = 0;
	if ((bytesReceived = recvfrom(socket, receiveBuffer, BUFLEN, 0, NULL, NULL)) == -1) {
		cout << "Error receiving feedback from server." << endl;
		exit(1);
	} else {
			int idInt;
			sscanf(receiveBuffer, "%d, %*s",&idInt);
		// If message we sent and message returned from server are the same, success
		if (strcmp(sendBuffer, receiveBuffer) == 0 && cmdCheck != 'y') {
			success = true;
			cout << "Server received the command!" << endl;
		//Special case: if message was a spawn command, server sends back the ID to the client
		} else if (strcmp(sendBuffer, receiveBuffer) != 0 && cmdCheck == 's') {
			success = true;
			this->setClientUniqueId(idInt);
 			cout << "Server received the spawn command!" << endl;
			cout << "This client is controlling drone #" << this->getClientUniqueId()<< "" <<endl;
		//Special case: if message was a ready up command, client sets readyToCommand boolean to true
		} else if (strcmp(sendBuffer, receiveBuffer) == 0 && cmdCheck =='y'){
			success = true;
			this->setClientReadyToCommand(true);
			cout << "Server received the command!" << endl;
		//Special case: if message was a navdata print command, client receives a string of data and prints it to display
		} else if(strcmp(sendBuffer, receiveBuffer) != 0 && (cmdCheck == 'b' || cmdCheck == 'f' ||
				cmdCheck == 'w' || cmdCheck == 'v' || cmdCheck == 'n' || cmdCheck == 'p')) {
			success = true;
			for(int i = 0; i < strlen(receiveBuffer); i++){
			cout << receiveBuffer[i];
			}
			cout << endl;
		//Special case: if message was a sense command, client receives an int to interpret
		} else if(strcmp(sendBuffer, receiveBuffer) != 0 && (cmdCheck == 'u' || cmdCheck == 'd' ||
				cmdCheck == 'k' || cmdCheck == 'j')) {
			success = true;
			cout << idInt <<endl;
		} else {
			cout << "Error: Server didn't receive the command. Exiting." << endl;
			success = false;
		}
	}

	return success;
}

/**
 * Sends a simple command (takeoff, land, or reset) to the specified drone. Returns a bool value based on whether the message is sent succesfully.
 * @param command Command to send to client, indicated by a single char
 * @param droneId ID of drone in question
 * @return Boolean value true if message successfully sent and echoed, false if there is a miscommunication
 */
bool GaTACDroneControl::commandDrone(char command, int droneId) {
	bool success = false;
	// Send command to server
		char message[32];
		sprintf(message, "%c %d", command, droneId);
		success = sendMessage(message, serverSocket, srv);

	return success;
}

/**
 * This method launches the Gazebo simulator with a grid of whatever size was specified via the setGridSize method,
 * and with any drones that have been set up via the setUpDrone method.
 *
 * *** NOTE: When using real drones, this method instead creates/initializes the ROS nodes for each drone. ***
 */
void GaTACDroneControl::launchGrid() {
	cout << "Launching Grid "<<endl;
	/* simulatorMode == true */
	if(simulatorMode == true){
		const char *gazeboMessage = "xterm -e roslaunch /tmp/grid_flight.launch&";
		const char *thincSmartCommand = "ROS_NAMESPACE=drone%d xterm -e rosrun ardrone_thinc thinc_smart %d %d %d %d %d %d %f s&";
		char thincSmartMessage[256];

		// Configure launch file and start gazebo
		configureLaunchFile();
		system(gazeboMessage);

		// Wait for gazebo to finish loading. This takes a while.
		sleep(10);

		// Starting a thinc_smart ROS node for each drone
		int droneID;
		for (int i = 0; i < numberOfDrones; i++) {
			droneID = i;
			sprintf(thincSmartMessage, thincSmartCommand, droneID, numberOfColumns, numberOfRows, dronePositions.at(droneID).first, dronePositions.at(droneID).second, 2, 2, (droneID + 1.0) * 0.4);
			cout << "publishing message: " << thincSmartMessage << endl;
			system(thincSmartMessage);
		}
		sleep(2);
	}
	/* simulatorMode == false */
	if(simulatorMode == false){

		const char *coreMessage = "xterm -e roscore&";
		const char *launchMessage = "xterm -e roslaunch /tmp/tagLaunch.launch&";
		const char *thincSmartCommand = "ROS_NAMESPACE=drone%d xterm -e rosrun ardrone_thinc thinc_smart %d %d %d %d %d %d %f r&";
		const char *ardroneDriverCommand = "ROS_NAMESPACE=drone%d rosrun ardrone_autonomy ardrone_driver %s&";
		const char *flattenTrim = "ROS_NAMESPACE=drone%d xterm -e rosservice call --wait /drone%d/ardrone/flattrim&";
		const char *toggleCam = "ROS_NAMESPACE=drone%d xterm -e rosservice call /drone%d/ardrone/togglecam&";
		char thincSmartMessage[256];
		char ardroneDriverMessage[256];
		char flatTrimMessage[256];
		char toggleCamMessage[256];

		// Configure launch file and start core
		configureLaunchFile();
//		system(coreMessage);
		system(launchMessage);

		sleep(10);

		// Starting a thinc_smart ROS node for each drone && an ardrone_autonomy ROS node for each drone
		int droneID;
		for (int i = 0; i < numberOfDrones; i++) {
			droneID = i;
			sprintf(thincSmartMessage, thincSmartCommand, droneID, numberOfColumns, numberOfRows, dronePositions.at(droneID).first, dronePositions.at(droneID).second, 0.5, 0.5, (droneID + 1.0) * .75);
			cout << "publishing message: " << thincSmartMessage << endl;
			system(thincSmartMessage);
		}
		sleep(2);

	}
}

/**
 * In simulation, this method modifies the grid_flight.launch file used by Gazebo to start the simulator.
 * It specifies the grid size and number/starting position of all drones.
 *
 * With actual drones, this method uses a modification of ardrone_autonomy's ardrone.launch file, to allow tag spotting and specify other parameters.
 *
 * *** NOTE: Directory in which to create launch file can be specified here. ***
 */
void GaTACDroneControl::configureLaunchFile() {
	cout << "Configuring launch file." << endl;
	/* simulatorMode == true */
	if(simulatorMode == true){
		// The following strings match the formatting of the grid_flight.launch file found in the thinc_sim_gazebo ros package (in the launch folder).
		const char *genTextureText =
			"<?xml version=\"1.0\"?>\n\n"
			"<launch>\n\t"
			"<node\n\t\t"
			"name=\"gen_texture\" pkg=\"cvg_sim_gazebo\" type=\"gen_texture\"\n\t\t"
			"args=\"%d %d 1 1 $(find cvg_sim_gazebo)/meshes/grid.png $(find cvg_sim_gazebo)/worlds/grid.world\"\n\t"
			"/>\n\t";
		const char *genDaeText =
			"<node\n\t\t"
			"name=\"gen_dae\" pkg=\"cvg_sim_gazebo\" type=\"gen_dae.py\"\n\t\t"
			"args=\"%d %d 2 2 $(find cvg_sim_gazebo)/meshes/grid.dae\"\n\t"
			"/>\n\n\t"
			"<!-- Start Gazebo with wg world running in (max) realtime -->\n\t"
			"<include file=\"$(find cvg_sim_gazebo)/launch/grid.launch\"/>\n\n\t"
			"<!-- Spawn simulated quadrotor uav -->\n";
		const char *droneText =
			"\t<group ns=\"drone%d\">\n\t\t"
			"<param name=\"tf_prefix\" value=\"drone%d\"/>\n\t\t"
			"<include file=\"$(find cvg_sim_gazebo)/launch/spawn_quadrotor.launch\" >\n\t\t\t"
			"<arg name=\"robot_namespace\" value=\"drone%d\"/>\n\t\t\t"
			"<arg name=\"x\" value=\"%.2f\"/>\n\t\t\t"
			"<arg name=\"y\" value=\"%.2f\"/>\n\t\t\t"
			"<arg name=\"z\" value=\"0.70\"/>\n\t\t"
			"</include>\n\t"
			"</group>\n\n";
		const char *endingText = "</launch>";

		// Open launch file. Check what ROS distribution is currently being used
		char *distro = getenv("ROS_DISTRO");
		if(distro == NULL){
			cout << "No ros distribution currently active. Please make sure your server machine has ros installed." << endl;
			exit(1);
		}

		// Assume launch file is located in default stacks directory for current ROS distribution
		char launchFilePath[100];
		// sprintf(launchFilePath, "/home/fuerte_workspace/gatacdronecontrol/launch/two_real_flight.launch", distro); /* File path on gray laptop */
		sprintf(launchFilePath, "/tmp/grid_flight.launch", distro);
		// Open file stream
		ofstream fileStream(launchFilePath, ios::trunc);

		// Write gen_texture and gen_dae text to file
		char textureBuffer[strlen(genTextureText) + 1];
		char daeBuffer[strlen(genDaeText) + 1];
		sprintf(textureBuffer, genTextureText, numberOfRows, numberOfColumns);
		sprintf(daeBuffer, genDaeText, numberOfColumns, numberOfRows);

		if (fileStream.is_open()) {
			fileStream << textureBuffer;
			fileStream << daeBuffer;
		}

		// Find coordinates of (0,0) in Gazebo terms
		int originX, originY;
		getGazeboOrigin(originX, originY);

		// Write all drone sub-launch text to file
		int droneID;
		float droneX, droneY;
		char droneBuffer[strlen(droneText) + 256];
		if (fileStream.is_open()) {
			for (int i = 0; i < numberOfDrones; i++) {
				droneID = i;
				droneX = originX + (2 * dronePositions.at(droneID).second);
				droneY = originY - (2 * dronePositions.at(droneID).first);
				sprintf(droneBuffer, droneText, droneID, droneID, droneID, droneID, droneX, droneY);
				fileStream << droneBuffer;
			}
			fileStream << endingText;
		}

		// Close file stream
		fileStream.close();
	}

	/* simulatorMode == false */
	if(simulatorMode == false){
		// The following strings match the formatting of the ardrone.launch file found in the ardrone_autonomy ros package (in the launch folder).
		const char *startingText =
			"<?xml version=\"1.0\"?>\n\n"
			"<launch>\n";
		const char *droneText =
			"\t<group ns=\"drone%d\">\n\t\t"
			"<param name=\"tf_prefix\" value=\"drone%d\"/>\n\t\t"
			"<include file=\"$(find ardrone_autonomy)/launch/vanilla.launch\" >\n\t\t\t"
			"<arg name=\"drone_ip\" value=\"192.168.1.1%d\"/>\n\t\t"
			"<arg name=\"drone_frame_id\" value=\"drone%d_base\"/>\n\t\t"
			"</include>\n\t"
			"<include file=\"$(find tum_ardrone)/launch/tum_ardrone.launch\" />\n"
			"</group>\n\n";

		const char *endingText = "</launch>";

		// Open launch file. Check what ROS distribution is currently being used
		char *distro = getenv("ROS_DISTRO");
			if(distro == NULL){
				cout << "No ros distribution currently active. Please make sure your server machine has ros installed." << endl;
				exit(1);
			}

		// Assume launch file is located in default stacks directory for current ROS distribution
		char launchFilePath[100];
		// sprintf(launchFilePath, "/home/fuerte_workspace/gatacdronecontrol/launch/two_real_flight.launch", distro); /* File path on gray laptop */
		sprintf(launchFilePath, "/tmp/tagLaunch.launch", distro);
		// Open file stream
		ofstream fileStream;
		fileStream.open(launchFilePath, ios::out);

		// Write gen_texture and gen_dae text to file
		char startingBuffer[strlen(startingText) + 1];
		strcpy(startingBuffer, startingText);
		if (fileStream.is_open()) {
			fileStream << startingBuffer;
		}

		// Find coordinates of (0,0) in Gazebo terms
		int originX, originY;
		getGazeboOrigin(originX, originY);

		// Write all drone sub-launch text to file
		int droneID;
		float droneX, droneY;
		char droneBuffer1[strlen(droneText) + 256];
		if (fileStream.is_open()) {
			for (int i = 0; i < numberOfDrones; i++) {
				droneID = i;
				droneX = originX + (2 * dronePositions.at(droneID).second);
				droneY = originY - (2 * dronePositions.at(droneID).first);
            			sprintf(droneBuffer1, droneText, droneID, droneID, droneID, droneID);
            			fileStream << droneBuffer1;
			}
			fileStream << endingText;

		}

		// Close file stream
		fileStream.close();
	}
}

/**
 * Gazebo places the grid at different locations within its own coordinate system depending on the size of the grid.
 * The user will specify a grid size (A x B), and this method will find the Gazebo coordinates of (0, 0) on the user's grid.
 * @param x User grid's X-axis origin
 * @param y User grid's Y-axis origin
 */
void GaTACDroneControl::getGazeboOrigin(int& x, int& y) {
	x = (-1) * (numberOfRows - 1);
	y = numberOfColumns - 1;
}


/**
 * This method takes movement parameters and sends waypoint messages one cell at a time.
 * As movements are made, it updates the position of moving drone and checks for shared cells
 * using sharedSpace() method. On completion of desired movement, drone's final destination is printed on the server terminal.
 * @param x Desired X-axis destination
 * @param y Desired Y-axis destination
 * @param Id ID of the drone being moved
 */
void GaTACDroneControl::moveAndCheck(int x, int y, int Id)
{
	char publishMessage[BUFLEN];
	const char *moveCommand = "rosservice call /drone%d/waypoint -- %d %d -1"; //id...  x y z id
	int droneId = Id;
	int dx = dronePositions.at(droneId).first - x;
	int dy = dronePositions.at(droneId).second - y;
	//Using same logic as ardrone_thinc.cpp file, send messages for movement one cell at a time
	/* simulatorMode == true and false */
	if(!(dx == 0 && dy == 0)){
	do
	{
	if(dx > 0){
	dronePositions.at(droneId).first -= 1;
	dx--;
	}
	else if(dx < 0){
	dronePositions.at(droneId).first += 1;
	dx++;
	}
	else if(dy < 0){
	dronePositions.at(droneId).second += 1;
	dy++;
	}
	else if(dy > 0){
	dronePositions.at(droneId).second -= 1;
	dy--;
	}
	int xSend = dronePositions.at(droneId).first;
	int ySend = dronePositions.at(droneId).second;
	sprintf(publishMessage, moveCommand, droneId, xSend, ySend);
	system(publishMessage);
	if(sharedSpace() == true){
		cout<<"Drones sharing a cell: " << endl;
		for(int i = 0; i < dronesSharingSpace.size(); i++)
		{
		if(dronesSharingSpace.at(i) == true)
		cout<<"==> drone "<<i<<" @ ("<<dronePositions.at(i).first<<", "<<dronePositions.at(i).second<<")"<<endl;
		}
	}
	}while ((dx != 0) || (dy != 0));
	cout << this->getGridPosition(droneId) << endl;
	}
	else if(dx == 0 && dy == 0)
	{
	sprintf(publishMessage, moveCommand, droneId, x, y);
	system(publishMessage);
	cout << "Drone " << droneId << "hovering." << endl;
	}
}

/**
 * This method compares current locations of drones and returns true if a cell is being shared.
 * Additionally, it updates a vector of which drones are sharing a cell.
 * If a shared cell is detected, the coordinates and drone ID's are printed to server terminal.
 * @return Boolean indicating whether a cell on the grid is occupied by two or more drones; true indicates a shared cell is detected
 */
bool GaTACDroneControl::sharedSpace()
{
	/* simulatorMode == true and false */
	bool sharing = false;
	for(int r = 0; r < dronesSharingSpace.size(); r++)
		dronesSharingSpace.at(r) = false;
	//cycles through current positions, if two/three drones have a matching position their dronesSharingSpace index is set to true
	for(int i = 0; i < dronePositions.size(); i++)
	{
		for(int k = 0; k <dronePositions.size(); k++)
		{
			if(i != k)
			{
			if((dronePositions.at(i).first == dronePositions.at(k).first)
			   && (dronePositions.at(i).second == dronePositions.at(k).second))
			{
				sharing = true;
				dronesSharingSpace.at(i) = true;
				dronesSharingSpace.at(k) = true;
			}
			}
		}
	}
	return sharing;
}

/**
 * This method returns true if the maximum number of drones has already been spawned.
 * @return Boolean indicating whether 3 drones are already on the grid, true if this is the case
 */
bool GaTACDroneControl::maxDrones()
{
	if(numberOfDrones >= expectedDrones)
	return true;
	else
	return false;
}

/**
 * This method returns true if a drone id sent by the client is a valid drone id that has previously been spawned.
 * @param id Drone ID to be verified
 * @return Boolean indicating whether the given drone ID is currently on the grid, true if the ID is in use and drone is present
 */
bool GaTACDroneControl::validDroneId(int id)
{
	if(id < 0 || id > (numberOfDrones))
	return false;
	else
	return true;
}

/**
 * This method returns the value of boolean gridSizeSet, which lets the server check if the grid size has been set.
 * @return Boolean indicating whether the grid size has been set, true if the size has already been set
 */
bool GaTACDroneControl::gridSizeCheck()
{
	return gridSizeSet;
}

/**
 * This method returns the value of boolean gridStarted, which lets the server know if the grid has been started and the drones' ROS nodes have been initialized
 * @return Boolean indicating whether the grid has started and ROS nodes have been initialized, true if the size has already been set
 */
bool GaTACDroneControl::gridStartCheck()
{
	return gridStarted;
}

/**
 * This method returns true if a location sent by the client is within the bounds of the grid.
 * @param x X-axis value to be verified
 * @param y Y-axis value to be verified
 * @return Boolean indicating the given location is on the grid, true if the location is valid
 */
bool GaTACDroneControl::validLocation(int x, int y)
{
	if(x < 0 || y < 0 || x >= numberOfColumns || y >= numberOfRows)
	return false;
	else
	return true;
}

/**
 * This method returns true if a client-set grid is between 0x0 and 10x10.
 * @param x X-axis dimension to be verified
 * @param y Y-axis dimension to be verified
 * @return Boolean indicating whether the specified grid dimensions are valid, true if valid
 */
bool GaTACDroneControl::validGridSize(int x, int y)
{
	if(x < 1 || y < 1 || x > 10 || y > 10)
	return false;
	else
	return true;
}

/**
 * This method will return and print the current position of a given drone on the grid.
 * @param droneId ID of drone to return navdata from
 * @return Human-readable string denoting the drone's current location on the grid
 */
string GaTACDroneControl::getGridPosition(int droneId)
{
	std::stringstream strID;
	std::stringstream strX;
	std::stringstream strY;
	int xPos = dronePositions.at(droneId).first;
	int yPos = dronePositions.at(droneId).second;
	strID << droneId;
	strX << xPos;
	strY << yPos;
	string id_str = strID.str();
	string x_str = strX.str();
	string y_str = strY.str();
 	string toReturn = "Drone " + id_str + " position: (" + x_str + ", " + y_str + ")";
	return toReturn;
}

/**
 * This method will call the PrintNavdata service to set the drone's data members to the correct values and return the requested data to the client.
 * @param droneId ID of drone to return navdata from
 * @return Character array of all navdata values, to be sent over a socket to the client, broken up into strings of thirty characters, and used to set navdata members
 */
const char* GaTACDroneControl::getData(int droneId)
{
	char printNavMessage[BUFLEN];
	const char *printNavdataCommand = "rosservice call /drone%d/printnavdata&"; //id...option id
	int id = droneId;
	sprintf(printNavMessage, printNavdataCommand, id);
	system(printNavMessage);
	string line1 = "";
	string line2 = "";
	string line3 = "";
	string line4 = "";
	string line5 = "";
	string line6 = "";
	ifstream stream0("currentNavdata0.txt");
	ifstream stream1("currentNavdata1.txt");
	ifstream stream2("currentNavdata2.txt");
	string allDataString = "";

	if(id == 0){
		//battery
		getline(stream0, line1);
		if(line1.size() < 30){
 			int whiteSpace = 30 - line1.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line1 += " ";
			 }
		}
		//forward velocity
		getline(stream0, line2);
		if(line2.size() < 30){
 			int whiteSpace = 30 - line2.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line2 += " ";
			 }
		}
		//sideways velocity
		getline(stream0, line3);
		if(line3.size() < 30){
 			int whiteSpace = 30 - line3.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line3 += " ";
			 }
		}
		//vert velocity
		getline(stream0, line4);
		if(line4.size() < 30){
 			int whiteSpace = 30 - line4.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line4 += " ";
			 }
		}
		//sonar
		getline(stream0, line5);
		if(line5.size() < 30){
 			int whiteSpace = 30 - line5.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line5 += " ";
			 }
		}
		//tags spotted
		getline(stream0, line6);
		if(line6.size() < 30){
 			int whiteSpace = 30 - line6.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line6 += " ";
			 }
		}
		allDataString = line1 + line2 + line3 + line4 + line5 + line6;
	}

	else if(id == 1){
		//battery
		getline(stream1, line1);
		if(line1.size() < 30){
 			int whiteSpace = 30 - line1.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line1 += " ";
			 }
		}
		//forward velocity
		getline(stream1, line2);
		if(line2.size() < 30){
 			int whiteSpace = 30 - line2.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line2 += " ";
			 }
		}
		//sideways velocity
		getline(stream1, line3);
		if(line3.size() < 30){
 			int whiteSpace = 30 - line3.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line3 += " ";
			 }
		}
		//vert velocity
		getline(stream1, line4);
		if(line4.size() < 30){
 			int whiteSpace = 30 - line4.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line4 += " ";
			 }
		}
		//sonar
		getline(stream1, line5);
		if(line5.size() < 30){
 			int whiteSpace = 30 - line5.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line5 += " ";
			 }
		}
		//tags spotted
		getline(stream1, line6);
		if(line6.size() < 30){
 			int whiteSpace = 30 - line6.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line6 += " ";
			 }
		}
		allDataString = line1 + line2 + line3 + line4 + line5 + line6;
	}

	else if(id == 2){
		//battery
		getline(stream2, line1);
		if(line1.size() < 30){
 			int whiteSpace = 30 - line1.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line1 += " ";
			 }
		}
		//forward velocity
		getline(stream2, line2);
		if(line2.size() < 30){
 			int whiteSpace = 30 - line2.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line2 += " ";
			 }
		}
		//sideways velocity
		getline(stream2, line3);
		if(line3.size() < 30){
 			int whiteSpace = 30 - line3.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line3 += " ";
			 }
		}
		//vert velocity
		getline(stream2, line4);
		if(line4.size() < 30){
 			int whiteSpace = 30 - line4.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line4 += " ";
			 }
		}
		//sonar
		getline(stream2, line5);
		if(line5.size() < 30){
 			int whiteSpace = 30 - line5.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line5 += " ";
			 }
		}
		//tags spotted
		getline(stream2, line6);
		if(line6.size() < 30){
 			int whiteSpace = 30 - line6.size();
 			for(int w = 0 ; w < whiteSpace; w++)
			 {
				line6 += " ";
			 }
		}
		allDataString = line1 + line2 + line3 + line4 + line5 + line6;
	}

	return allDataString.c_str();
}

/**
 * This method allows a client to query the server whether another drone is north, south, east, or west of the client's drone on the grid.
 * @param droneId The drone ID of the client sending sense request.
 * @param option Integer denoting the direction to sense; 0 -> North, 1 -> South, 2 -> East, 3 -> West
 * @return 0 if no drone is on that side of subject drone, 1 if another drone is within one square above, 2 if another drone is greater than one square above
 */
int GaTACDroneControl::sense(int droneId, int option)
{
	if(option == 0)
	{
	int xCurrent = dronePositions.at(droneId).first;
	int yCurrent = dronePositions.at(droneId).second;

	//cycles through current drone positions and tests them against querying client position
	for(int k = 0; k <dronePositions.size(); k++)
		{
		if(droneId != k){
			//case: another drone is not North of subject drone
			if((dronePositions.at(k).second <= yCurrent))
				return 0;
			//case: another drone is 1 square North of subject drone
			else if((dronePositions.at(k).second == yCurrent + 1))
				return 1;
			//case: another drone is 2 or more squares North of subject drone
			else if((dronePositions.at(k).second >= yCurrent + 2))
				return 2;
			}
		}
	}
	else if(option == 1)
	{
	int xCurrent = dronePositions.at(droneId).first;
	int yCurrent = dronePositions.at(droneId).second;

	//cycles through current drone positions and tests them against querying client position
	for(int k = 0; k <dronePositions.size(); k++)
		{
		if(droneId != k){
			//case: another drone is not South of subject drone
			if((dronePositions.at(k).second >= yCurrent))
				return 0;
			//case: another drone is 1 square South of subject drone
			else if((dronePositions.at(k).second == yCurrent - 1))
				return 1;
			//case: another drone is 2 or more squares South of subject drone
			else if((dronePositions.at(k).second <= yCurrent - 2))
				return 2;
			}
		}
	}
	else if(option == 2)
	{
	int xCurrent = dronePositions.at(droneId).first;
	int yCurrent = dronePositions.at(droneId).second;

		//cycles through current drone positions and tests them against querying client position
		for(int k = 0; k <dronePositions.size(); k++)
		{
		if(droneId != k){
			//case: another drone is not East of subject drone
			if((dronePositions.at(k).first <= xCurrent))
				return 0;
			//case: another drone is 1 square East of subject drone
			else if((dronePositions.at(k).first == xCurrent + 1))
				return 1;
			//case: another drone is 2 or more squares East of subject drone
			else if((dronePositions.at(k).first >= xCurrent + 2))
				return 2;
			}
		}
	}
	else if(option == 3)
	{
	int xCurrent = dronePositions.at(droneId).first;
	int yCurrent = dronePositions.at(droneId).second;

		//cycles through current drone positions and tests them against querying client position
		for(int k = 0; k <dronePositions.size(); k++)
		{
		if(droneId != k){
			//case: another drone is not West of subject drone
			if((dronePositions.at(k).first >= xCurrent))
				return 0;
			//case: another drone is 1 square West of subject drone
			else if((dronePositions.at(k).first == xCurrent - 1))
				return 1;
			//case: another drone is 2 or more squares West of subject drone
			else if((dronePositions.at(k).first <= xCurrent - 2))
				return 2;
			}
		}
	}
}

