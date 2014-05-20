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

#define BUFLEN 128
#define DEFAULTCLIENTPORT1 4999
#define DEFAULTCLIENTPORT2 5999
#define DEFAULTCLIENTPORT3 6999

GaTACDroneControl::GaTACDroneControl() {
	serverSocket, numberOfColumns, numberOfRows, numberOfDrones = 0;
	gridSizeSet, gridStarted = false;
	simulatorMode = true;
	srv = NULL;
	serverThreads = 0;	
}

GaTACDroneControl::GaTACDroneControl(const char* c) {
	serverSocket, numberOfColumns, numberOfRows, numberOfDrones = 0;
	gridSizeSet, gridStarted = false;
	simulatorMode = false;
	srv = NULL;
	serverThreads = 0;
}
void GaTACDroneControl::startServer(const char *remoteIP, const char *remotePort, int expectedDrones){
	cout << "Main server running." << endl;	
	for(int i = 0; i < expectedDrones; i++)
	{
	serverThreads++;
	boost::thread* moveThread;
	if(i == 0){
	moveThread = new boost::thread(boost::bind(&GaTACDroneControl::runServer,this,remoteIP,remotePort, serverThreads));	
	threads[i] = moveThread;
	cout<<"starting thread 1"<<endl;
	}
	else if(i == 1){
	const char* remotePort2 = "5999";
	moveThread = new boost::thread(boost::bind(&GaTACDroneControl::runServer,this,remoteIP,remotePort2, serverThreads));	
	threads[i] = moveThread;
	cout<<"starting thread 2"<<endl;
	}
	else if(i == 2){
	const char* remotePort3 = "6999";
	moveThread = new boost::thread(boost::bind(&GaTACDroneControl::runServer,this,remoteIP,remotePort3, serverThreads));	
	threads[i] = moveThread;
	cout<<"starting thread 3"<<endl;
	}   
	}

}
void GaTACDroneControl::runServer(const char *remoteIp, const char *remotePort, int threadNo) {
	const char *publishCommand = "rostopic pub -1 /drone%s/ardrone/%s std_msgs/Empty&";
	char localport[4];
	int errorCheck, sock;
	struct addrinfo hints, *srv, *info;
	struct sockaddr_storage client_addr;
	socklen_t addr_len = sizeof client_addr;
	if(threadNo == 1){
	sprintf(localport, "%d", DEFAULTCLIENTPORT1);
	}	
	if(threadNo == 2){
	sprintf(localport, "%d", DEFAULTCLIENTPORT2);
	}
	if(threadNo == 3){
	sprintf(localport, "%d", DEFAULTCLIENTPORT3);
	}
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
		vector<string> tokens;
		string stringCommand(receiveBuffer);
		stringstream ss(stringCommand);

		// Storing tokens in vector
		while (ss >> buffer) {
			tokens.push_back(buffer);
		}

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
			// If 3 drones already exist
			 if (maxDrones() == true) {
			errorMessage = "You have already spawned the maximum number of drones (3).";
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
			clientsReady.push_back(false);
			printf("Ready to spawn drone at [%d, %d].\n", initialColumn, initialRow);
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
			else{
			/* If server passes all checks, client message processed */
			sprintf(publishMessage, publishCommand, droneNumber, "takeoff");
			system(publishMessage);
			sleep(3); // Wait for takeoff to complete
			if(simulatorMode == false)
			varyHeights(droneInt);
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
			else{
			/* If server passes all checks, client message processed */
			sprintf(publishMessage, publishCommand, droneNumber, "land");
			system(publishMessage);
			sleep(3); // Wait for drone to land completely
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
			sprintf(publishMessage, publishCommand, droneNumber, "reset");
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
			for(int i=0; i < clientsReady.size(); i++)
			{
			if(clientsReady.at(i) == false){
			cout << "Client ready: Drone" << i << endl;
			if(i < clientsReady.size())
			sleepCtr = 35;
			if(i == clientsReady.size()-1)
			sleepCtr = 7;
			clientsReady.at(i) = true;
			break;
			}
			}
			allReady = true;
			for(int i=0; i < clientsReady.size(); i++)
			{
			if(clientsReady.at(i) == false){
			allReady = false;
			}
			}
			if(allReady == true){
			// If grid size has been set
			if (this->gridSizeCheck() == true && this->gridStartCheck() == false) {
			cout << "All clients ready. Starting grid!" << endl;
			cout << "Waiting a few seconds before server will receive commands..." << endl;
			launchGrid();
			gridStarted = true;
			if(simulatorMode == true){
			for(int i = 0; i < numberOfDrones; i++)
				varyHeights(i);
			}
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
			cout << "Start gazebo." << endl;
			// If grid size has been set
			if (this->gridSizeCheck() == true && this->gridStartCheck() == false) {
			launchGrid();
			gridStarted = true;
			if(simulatorMode == true){
			for(int i = 0; i < numberOfDrones; i++)
				varyHeights(i);
			}
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
			break;

		default:
			cout << "Error parsing raw command - invalid command character received." << endl;
			break;
		}

		// Sending acknowledgment message to client
		int len = strlen(receiveBuffer);
		char sendBuffer[len];
		strcpy(sendBuffer, receiveBuffer);
		//If command received was to spawn a drone, first char of ACK set to id
		if(rawCommand == 's'){
		char idChar = (char)(((int)'0')+numberOfDrones-1);
		sendBuffer[0] = idChar;
		int numSent = 0;
		sleep(5);
		if ((numSent = sendto(sock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
			perror("Server: error sending acknowledgment.");
			exit(1);
		}
		}
		else if(rawCommand == 'y'){
		cout<<"Waiting for other drones..." <<endl;
		sleep(sleepCtr);
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

void GaTACDroneControl::launchClient(char *serverIp, char *serverPort) {
	char *host = serverIp;
	char *port = serverPort;
	int errorCheck, sock;

	struct addrinfo hints, *srv, *info;

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

	// Sleep to ensure that the connection is complete before commands are sent
	sleep(3);
}

void GaTACDroneControl::startGrid() {
	bool worked = false;
		cout << "Sending command to start grid." << endl;
		char message[2] = "i";
		worked = sendMessage(message, serverSocket, srv);
}

void GaTACDroneControl::readyUp() {
	bool worked = false;
		cout << "Sending ready message to server." << endl;
		char message[2] = "y";
		worked = sendMessage(message, serverSocket, srv);
}

void GaTACDroneControl::setGridSize(int numberOfColumns, int numberOfRows) {
	bool worked = false;
	// Send command to server
		printf("Sending command to set grid size to %dx%d.\n", numberOfColumns, numberOfRows);

		char message[10];
		sprintf(message, "g %d %d", numberOfColumns, numberOfRows);
		worked = sendMessage(message, serverSocket, srv);

		if (!worked) {
			cout << "Couldn't set the grid size. Please try again." << endl;
			exit(1);
		}
}

void GaTACDroneControl::move(int droneId, int x, int y) {
	bool worked = false;
	// Send command to server, checks for valid ID and location done server-side	
		printf("Sending command to move drone #%d to (%d, %d).\n", droneId, x, y);
		char message[10];
		sprintf(message, "m %d %d %d", droneId, x, y);
		worked = sendMessage(message, serverSocket, srv);
}

void GaTACDroneControl::hover(int droneId) {

	// Send command to server
	bool worked = commandDrone('h', droneId);
	printf("Sending command to make drone hover.\n", droneId);
		char message[3];
		sprintf(message, "h %d", droneId);
		worked = sendMessage(message, serverSocket, srv);
}

void GaTACDroneControl::land(int droneId) {
	printf("Sending command to land drone #%d.\n", droneId);

	// Send command to server
	bool worked = commandDrone('l', droneId);
	if (!worked) {
		cout << "Couldn't land drone. Please try again." << endl;
		exit(1);
	}
}

void GaTACDroneControl::takeoff(int droneId) {
	printf("Sending command to takeoff drone #%d.\n", droneId);

	// Send command to server
	bool worked = commandDrone('t', droneId);
	if (!worked) {
		cout << "Couldn't take off. Please try again." << endl;
		exit(1);
	}
}

void GaTACDroneControl::reset(int droneId) {
	printf("Sending command to reset drone #%d.\n", droneId);

	// Send command to server
	bool worked = commandDrone('r', droneId);
	if (!worked) {
		cout << "Couldn't reset drone. Please try again." << endl;
		exit(1);
	}
}

void GaTACDroneControl::setupDrone(int droneCol, int droneRow) {
	bool worked = false;
	// Send command to server
		printf("Sending command to spawn drone at (%d, %d).\n", droneCol, droneRow);
		char msg[10];
		sprintf(msg, "s %d %d", droneCol, droneRow);
		worked = sendMessage(msg, serverSocket, srv);
}

void GaTACDroneControl::closeClient() {
	close(serverSocket);
	this->setClientReadyToCommand(false);
	freeaddrinfo(srv);
}

void GaTACDroneControl::setClientUniqueId(int toSet)
{
 this->clientUniqueId = toSet;
}

int GaTACDroneControl::getClientUniqueId()
{
 return this->clientUniqueId;
}

void GaTACDroneControl::setClientReadyToCommand(bool toSet)
{
 this->readyToCommand = toSet;
}

bool GaTACDroneControl::getClientReadyToCommand()
{
 return this->readyToCommand;
}

/*
 * Private Methods
 */

bool GaTACDroneControl::sendMessage(char *message, int socket, struct addrinfo *addrInfo) {
	bool success = false;
	char sendBuffer[BUFLEN];
	char receiveBuffer[BUFLEN] = { };
	strcpy(sendBuffer, message);
	char cmdCheck = sendBuffer[0];

	// Sending message
	int bytesSent = 0;
	if ((bytesSent = sendto(socket, sendBuffer, strlen(sendBuffer), 0, addrInfo->ai_addr, addrInfo->ai_addrlen)) == -1) {
		cout << "Error sending message to server with errno: " << errno << endl;
		success = false;
	}

	// Waiting for feedback (ensuring that the server received our message)
	int bytesReceived = 0;
	if ((bytesReceived = recvfrom(socket, receiveBuffer, strlen(sendBuffer), 0, NULL, NULL)) == -1) {
		cout << "Error receiving feedback from server." << endl;
		exit(1);
	} else {
			char* idCheck = &receiveBuffer[0];
			int idInt = atoi(idCheck);
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
		} else if (strcmp(sendBuffer, receiveBuffer) == 0 && cmdCheck =='y'){
			success = true;
			this->setClientReadyToCommand(true);
			cout << "Server received the command!" << endl;
		} else {
			cout << "Error: Server didn't receive the command. Exiting." << endl;
			success = false;
		}
	}

	return success;
}

bool GaTACDroneControl::commandDrone(char command, int droneId) {
	bool success = false;
	// Send command to server
		char message[3];
		sprintf(message, "%c %d", command, droneId);
		success = sendMessage(message, serverSocket, srv);

	return success;
}

void GaTACDroneControl::launchGrid() {
	/* simulatorMode == true */	
	if(simulatorMode == true){
	const char *gazeboMessage = "xterm -e roslaunch thinc_sim_gazebo grid_flight.launch&";
	const char *thincSmartCommand = "ROS_NAMESPACE=drone%d xterm -e rosrun ardrone_thinc thinc_smart %d %d %d %d %d s&";
	char thincSmartMessage[100];

	// Configure launch file and start gazebo
	configureLaunchFile();
	system(gazeboMessage);

	// Wait for gazebo to finish loading. This takes a while.
	sleep(5);

	// Starting a thinc_smart ROS node for each drone
	int droneID;
	for (int i = 0; i < numberOfDrones; i++) {
		droneID = i;
		sprintf(thincSmartMessage, thincSmartCommand, droneID, numberOfColumns, numberOfRows, droneID, dronePositions.at(droneID).first, dronePositions.at(droneID).second);
		cout << "publishing message: " << thincSmartMessage << endl;
		system(thincSmartMessage);
		sleep(3);
	}
	}
	/* simulatorMode == false */	
	if(simulatorMode == false){
	const char *coreMessage = "xterm -e roscore&";
	const char *launchMessage = "xterm -e roslaunch gatacdronecontrol two_real_flight.launch&";
	const char *thincSmartCommand = "ROS_NAMESPACE=drone%d xterm -e rosrun ardrone_thinc thinc_smart %d %d %d %d %d r&";
	const char *ardroneDriverCommand = "ROS_NAMESPACE=drone%d xterm -e rosrun ardrone_autonomy ardrone_driver %s&";
	const char *flattenTrim = "ROS_NAMESPACE=drone%d xterm -e rosservice call --wait /drone%d/ardrone/flattrim&";
	const char *toggleCam = "ROS_NAMESPACE=drone%d xterm -e rosservice call /drone%d/ardrone/togglecam&";
	char thincSmartMessage[100];
	char ardroneDriverMessage[100];
	char flatTrimMessage[100];
	char toggleCamMessage[100];

	// Configure launch file and start core
	configureLaunchFile();
	system(coreMessage);
	system(launchMessage);

	sleep(5);

	// Starting a thinc_smart ROS node for each drone && an ardrone_autonomy ROS node for each drone
	int droneID;
	for (int i = 0; i < numberOfDrones; i++) {
		droneID = i;
		sprintf(thincSmartMessage, thincSmartCommand, droneID, numberOfColumns, numberOfRows, droneID, dronePositions.at(droneID).first, dronePositions.at(droneID).second);
		cout << "publishing message: " << thincSmartMessage << endl;
		system(thincSmartMessage);
		sleep(3);
		if(droneID == 0)
		sprintf(ardroneDriverMessage, ardroneDriverCommand, droneID, "-ip 192.168.1.10");
		if(droneID == 1)
		sprintf(ardroneDriverMessage, ardroneDriverCommand, droneID, "-ip 192.168.1.11");
		cout << "publishing message: " << ardroneDriverMessage << endl;
		system(ardroneDriverMessage);
		sleep(5);
		sprintf(flatTrimMessage, flattenTrim, droneID, droneID);
		system(flatTrimMessage);
		cout << "Flattened trim for drone " << droneID << endl;
		cout << "Toggled cam for drone " << droneID << endl;
		sleep(5);
		sprintf(toggleCamMessage, toggleCam, droneID, droneID);
		system(toggleCamMessage);
	}
	}
}

void GaTACDroneControl::configureLaunchFile() {
	cout << "Configuring launch file." << endl;
	/* simulatorMode == true */
	if(simulatorMode == true){
	// The following strings match the formatting of the grid_flight.launch file found in the thinc_sim_gazebo ros package (in the launch folder).
	const char *genTextureText =
			"<?xml version=\"1.0\"?>\n\n"
			"<launch>\n\t"
				"<node\n\t\t"
					"name=\"gen_texture\" pkg=\"thinc_sim_gazebo\" type=\"gen_texture\"\n\t\t"
					"args=\"%d %d 1 1 $(find thinc_sim_gazebo)/Media/models/grid.png $(find thinc_sim_gazebo)/worlds/grid.world\"\n\t"
				"/>\n\t";
	const char *genDaeText =
			"<node\n\t\t"
				"name=\"gen_dae\" pkg=\"thinc_sim_gazebo\" type=\"gen_dae.py\"\n\t\t"
				"args=\"%d %d 2 2 $(find thinc_sim_gazebo)/Media/models/grid.dae\"\n\t"
			"/>\n\n\t"
			"<!-- Start Gazebo with wg world running in (max) realtime -->\n\t"
			"<include file=\"$(find thinc_sim_gazebo)/launch/grid.launch\"/>\n\n\t"
			"<!-- Spawn simulated quadrotor uav -->\n";
	const char *droneText =
			"\t<group ns=\"drone%d\">\n\t\t"
				"<param name=\"tf_prefix\" value=\"drone%d\"/>\n\t\t"
				"<include file=\"$(find thinc_sim_gazebo)/launch/spawn_quadrotor.launch\" >\n\t\t\t"
					"<arg name=\"model\" value=\"$(find thinc_sim_gazebo)/urdf/quadrotor_sensors%d.urdf\"/>\n\t\t\t"
					"<arg name=\"modelname\" value=\"drone%d\"/>\n\t\t\t"
					"<arg name=\"spawncoords\" value=\"-x %.2f -y %.2f -z 0.7 \"/>\n\t\t"
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
        // sprintf(launchFilePath, "/home/fuerte_workspace/gatacdronecontrol/launch/two_real_flight.launch", distro);    /* File path on gray laptop */
        sprintf(launchFilePath, "/home/caseyhetzler/fuerte_workspace/sandbox/thinc_simulator/thinc_sim_gazebo/launch/grid_flight.launch", distro);
	
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
	char droneBuffer[strlen(droneText)];
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
				"<include file=\"$(find ardrone_autonomy)/launch/ardrone.launch\" >\n\t\t\t"
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
        // sprintf(launchFilePath, "/home/fuerte_workspace/gatacdronecontrol/launch/two_real_flight.launch", distro);    /* File path on gray laptop */
        sprintf(launchFilePath, "/home/caseyhetzler/fuerte_workspace/sandbox/gatacdronecontrol/src/launch/two_real_flight.launch", distro);
	// Open file stream
	ofstream fileStream(launchFilePath, ios::trunc);

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
	char droneBuffer[strlen(droneText)];
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
}

void GaTACDroneControl::getGazeboOrigin(int& x, int& y) {
	x = (-1) * (numberOfRows - 1);
	y = numberOfColumns - 1;
}
void GaTACDroneControl::varyHeights(int droneNumber)
{
	/* simulatorMode == true and false */
	string temp;
	std::stringstream strID;
	char publishMessage[BUFLEN];
	const char *variableHeightTakeoff1 = "rostopic pub -1 /drone%s/cmd_vel geometry_msgs/Twist '[0,0,%.2f]' '[0,0,0]'"; //%.2f = speed of altitude increase
	const char *variableHeightTakeoff2 = "rostopic pub -1 /drone%s/cmd_vel geometry_msgs/Twist '[0,0,0]' '[0,0,0]'&"; //stops lifting

		double kDub = (double) droneNumber;
		float vHt = (float) (kDub+1.0)/10;
		strID << droneNumber;
		temp = strID.str();		
		sprintf(publishMessage, variableHeightTakeoff1, temp.c_str(), vHt);
		system(publishMessage);
		sprintf(publishMessage, variableHeightTakeoff2, temp.c_str());
		system(publishMessage);
		strID.str("");
		dronesSharingSpace.push_back(false);
		
	cout<< "Drone "<< droneNumber<< " increased altitude successfully"<<endl;
}
void GaTACDroneControl::moveAndCheck(int x, int y, int Id)
{	
	char publishMessage[BUFLEN];
	const char *moveCommand = "rosservice call /drone%d/waypoint %d %d 0 %d"; //id...  x y z id
	int droneId = Id;
	int dx = dronePositions.at(droneId).first - x;
	int dy = dronePositions.at(droneId).second - y;
	//Using same logic as ardrone_thinc.cpp file, send messages for movement one cell at a time
	/* simulatorMode == true and false */
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
	sprintf(publishMessage, moveCommand, droneId, xSend, ySend, droneId);
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
bool GaTACDroneControl::maxDrones()
{	
	if(numberOfDrones == 3)
	return true;
	else
	return false;
}

bool GaTACDroneControl::validDroneId(int id)
{	
	if(id < 0 || id > (numberOfDrones))
	return false;
	else
	return true;
}
bool GaTACDroneControl::gridSizeCheck()
{	
	return gridSizeSet;
}
bool GaTACDroneControl::gridStartCheck()
{
	return gridStarted;
}
bool GaTACDroneControl::validLocation(int x, int y)
{	
	if(x < 0 || y < 0 || x >= numberOfColumns || y >= numberOfRows)
	return false;
	else
	return true;
}

bool GaTACDroneControl::validGridSize(int x, int y)
{	
	if(x < 1 || y < 1 || x > 10 || y > 10)
	return false;
	else
	return true;
}

string GaTACDroneControl::getBattery(int droneId)
{
	this->getData(droneId, 0);
	return "";
}

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

string GaTACDroneControl::getForwardVelocity(int droneId)
{
	this->getData(droneId, 1);
	return "";
}

string GaTACDroneControl::getSidewaysVelocity(int droneId)
{
	this->getData(droneId, 2);
	return "";
}

string GaTACDroneControl::getVerticalVelocity(int droneId)
{
	this->getData(droneId, 3);
	return "";
}

string GaTACDroneControl::getSonar(int droneId)
{
	this->getData(droneId, 4);
	return "";
}

string GaTACDroneControl::getTagSpotted(int droneId)
{
	this->getData(droneId, 5);
	return "";
}
//Publishes request for relevant data to drone's PrintNavdata service, prints result in nice human readable string
	
string GaTACDroneControl::getData(int droneId, int option)
{
	char printNavMessage[BUFLEN];
	const char *printNavdataCommand = "rosservice call /drone%d/printnavdata&"; //id...option id
	int id = droneId;
	string line;
	ifstream stream0("currentNavdata0.txt");
	ifstream stream1("currentNavdata1.txt");
	ifstream stream2("currentNavdata2.txt");
	cout<< "Printing requested Navdata: " << endl;
	//battery
	if(option == 0){
	sprintf(printNavMessage, printNavdataCommand, id);
	system(printNavMessage);
	if(id == 0)
	getline(stream0, line);
	else if(id == 1)
	getline(stream1, line);
	else if(id == 2)
	getline(stream2, line);
	}
	//forward velocity
	else if(option == 1){
	sprintf(printNavMessage, printNavdataCommand, id);
	system(printNavMessage);
	if(id == 0){
	for(int i = 0; i < 1; ++i)
  		getline(stream0, line);
	getline(stream0, line);
	}
	else if(id == 1){
	for(int i = 0; i < 1; ++i)
  		getline(stream1, line);
	getline(stream1, line);
	}
	else if(id == 2){
	for(int i = 0; i < 1; ++i)
  		getline(stream2, line);
	getline(stream2, line);
	}
	}
	//sideways velocity
	else if(option == 2){
	sprintf(printNavMessage, printNavdataCommand, id);
	system(printNavMessage);
	if(id == 0){
	for(int i = 0; i < 2; ++i)
  		getline(stream0, line);
	getline(stream0, line);
	}
	else if(id == 1){
	for(int i = 0; i < 2; ++i)
  		getline(stream1, line);
	getline(stream1, line);
	}
	else if(id == 2){
	for(int i = 0; i < 2; ++i)
  		getline(stream2, line);
	getline(stream2, line);
	}
	}
	//vertical velocity
	else if(option == 3){
	sprintf(printNavMessage, printNavdataCommand, id);
	system(printNavMessage);
	if(id == 0){
	for(int i = 0; i < 3; ++i)
  		getline(stream0, line);
	getline(stream0, line);
	}
	else if(id == 1){
	for(int i = 0; i < 3; ++i)
  		getline(stream1, line);
	getline(stream1, line);
	}
	else if(id == 2){
	for(int i = 0; i < 3; ++i)
  		getline(stream2, line);
	getline(stream2, line);
	}
	}
	//sonar 
	else if(option == 4){
	sprintf(printNavMessage, printNavdataCommand, id);
	system(printNavMessage);
	if(id == 0){
	for(int i = 0; i < 4; ++i)
  		getline(stream0, line);
	getline(stream0, line);
	}
	else if(id == 1){
	for(int i = 0; i < 4; ++i)
  		getline(stream1, line);
	getline(stream1, line);
	}
	else if(id == 2){
	for(int i = 0; i < 4; ++i)
  		getline(stream2, line);
	getline(stream2, line);
	}
	}
	//tags spotted data
	else if(option == 5){
	sprintf(printNavMessage, printNavdataCommand, id);
	system(printNavMessage);
	if(id == 0){
	for(int i = 0; i < 5; ++i)
  		getline(stream0, line);
	getline(stream0, line);
	}
	else if(id == 1){
	for(int i = 0; i < 5; ++i)
  		getline(stream1, line);
	getline(stream1, line);
	}
	else if(id == 2){
	for(int i = 0; i < 5; ++i)
  		getline(stream2, line);
	getline(stream2, line);
	}
	}
	cout<< line << endl;
	return "getData";
}
