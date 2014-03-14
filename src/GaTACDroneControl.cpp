#include <iostream>
#include <errno.h> // For printing error #'s/debugging
#include <stdlib.h>
#include <sstream> // For converting chars to ints to update positions vector
#include <fstream> // For editing files

// For tokenizing command input
#include <sstream>

// GaTACDroneControl header
#include "GaTACDroneControl.hpp"

using namespace std;

#define BUFLEN 128
#define DEFAULTCLIENTPORT 4999

GaTACDroneControl::GaTACDroneControl() {
	serverSocket, numberOfColumns, numberOfRows, numberOfDrones = 0;
	gridSizeSet, gridStarted = false;
	simulatorMode = true;
	srv = NULL;
}

GaTACDroneControl::GaTACDroneControl(const char* c) {
	serverSocket, numberOfColumns, numberOfRows, numberOfDrones = 0;
	gridSizeSet, gridStarted = false;
	simulatorMode = false;
	srv = NULL;
}

void GaTACDroneControl::runServer(char *remoteIp, char *remotePort) {
	const char *publishCommand = "rostopic pub -1 /drone%s/ardrone/%s std_msgs/Empty";
	char localport[4];
	int errorCheck, sock;


	struct addrinfo hints, *srv, *info;
	struct sockaddr_storage client_addr;
	socklen_t addr_len = sizeof client_addr;

	sprintf(localport, "%d", DEFAULTCLIENTPORT);

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

	cout << "Main server running." << endl;

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
			else if (validDroneId(droneInt) == false) {
			errorMessage = invalidDroneId;
			}
			// If destination isn't valid
			else if (validLocation(xInt, yInt) == false) {
				errorMessage = invalidLocation;
			}

			if (errorMessage != "none") {
			printf("Error: couldn't move drone. %s\n", errorMessage.c_str());
			exit(1);
			}
		/* If server passes all checks, client message processed */
			else{
			moveAndCheck(xInt,yInt,droneNumberInt);		
			}
		//end moveAndCheck
			break;

		case 'g':
			cout << "Set grid size." << endl;
			numberOfColumns = atoi(tokens.at(1).c_str());
			numberOfRows = atoi(tokens.at(2).c_str());
			// If size has already been set
			if (gridSizeCheck() == true) {
			cout << "Error: The grid size has already been set. Specify grid size only once." << endl;
			exit(1);
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

		case 'i':
			cout << "Start gazebo." << endl;
			// If grid size has been set
			if (gridSizeCheck() == true) {
			launchGazebo();
			varyHeights();
			gridStarted = true;
			}
			// If grid size hasn't been set yet
			else {
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

		int numSent = 0;
		if ((numSent = sendto(sock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
			perror("Server: error sending acknowledgment.");
			exit(1);
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
		printf("Sending command to spawn drone with ID #%d at (%d, %d).\n", numberOfDrones, droneCol, droneRow);
		char msg[10];
		sprintf(msg, "s %d %d", droneCol, droneRow);
		worked = sendMessage(msg, serverSocket, srv);
}

void GaTACDroneControl::closeClient() {
	close(serverSocket);
	freeaddrinfo(srv);
}

/*
 * Private Methods
 */

bool GaTACDroneControl::sendMessage(char *message, int socket, struct addrinfo *addrInfo) {
	bool success = false;
	char sendBuffer[BUFLEN];
	char receiveBuffer[BUFLEN] = { };
	strcpy(sendBuffer, message);

	// Sending message
	int bytesSent = 0;
	if ((bytesSent = sendto(socket, sendBuffer, strlen(sendBuffer), 0, addrInfo->ai_addr, addrInfo->ai_addrlen)) == -1) {
		cout << "Error sending message to server with errno: " << errno << endl;
		exit(1);
	}

	// Waiting for feedback (ensuring that the server received our message)
	int bytesReceived = 0;
	if ((bytesReceived = recvfrom(socket, receiveBuffer, strlen(sendBuffer), 0, NULL, NULL)) == -1) {
		cout << "Error receiving feedback from server." << endl;
		exit(1);
	} else {
		// If message we sent and message returned from server are the same, success
		if (strcmp(sendBuffer, receiveBuffer) == 0) {
			success = true;
			cout << "Server received the command!" << endl;
		} else {
			cout << "Error: Server didn't receive the command. Exiting." << endl;
			exit(1);
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

void GaTACDroneControl::launchGazebo() {
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
	char thincSmartMessage[100];
	char ardroneDriverMessage[100];

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
		sprintf(ardroneDriverMessage, ardroneDriverCommand, droneID, "-ip 192.168.1.11");
		if(droneID == 1)
		sprintf(ardroneDriverMessage, ardroneDriverCommand, droneID, "-ip 192.168.1.10");
		cout << "publishing message: " << ardroneDriverMessage << endl;
		system(ardroneDriverMessage);
		sleep(3);
	}
	}
}

void GaTACDroneControl::configureLaunchFile() {
	cout << "Configuring launch file." << endl;
	/* simulatorMode == true */
	if(simulatorMode == true){
	// The following strings match the formatting of the grid_flight.launch file found in the thinc_sim_gazebo ros package (in the launch folder).
	char *genTextureText =
			"<?xml version=\"1.0\"?>\n\n"
			"<launch>\n\t"
				"<node\n\t\t"
					"name=\"gen_texture\" pkg=\"thinc_sim_gazebo\" type=\"gen_texture\"\n\t\t"
					"args=\"%d %d 1 1 $(find thinc_sim_gazebo)/Media/models/grid.png $(find thinc_sim_gazebo)/worlds/grid.world\"\n\t"
				"/>\n\t";
	char *genDaeText =
			"<node\n\t\t"
				"name=\"gen_dae\" pkg=\"thinc_sim_gazebo\" type=\"gen_dae.py\"\n\t\t"
				"args=\"%d %d 2 2 $(find thinc_sim_gazebo)/Media/models/grid.dae\"\n\t"
			"/>\n\n\t"
			"<!-- Start Gazebo with wg world running in (max) realtime -->\n\t"
			"<include file=\"$(find thinc_sim_gazebo)/launch/grid.launch\"/>\n\n\t"
			"<!-- Spawn simulated quadrotor uav -->\n";
	char *droneText =
			"\t<group ns=\"drone%d\">\n\t\t"
				"<param name=\"tf_prefix\" value=\"drone%d\"/>\n\t\t"
				"<include file=\"$(find thinc_sim_gazebo)/launch/spawn_quadrotor.launch\" >\n\t\t\t"
					"<arg name=\"model\" value=\"$(find thinc_sim_gazebo)/urdf/quadrotor_sensors%d.urdf\"/>\n\t\t\t"
					"<arg name=\"modelname\" value=\"drone%d\"/>\n\t\t\t"
					"<arg name=\"spawncoords\" value=\"-x %.2f -y %.2f -z 0.7 \"/>\n\t\t"
				"</include>\n\t"
			"</group>\n\n";
	char *endingText = "</launch>";

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
	char *startingText =
			"<?xml version=\"1.0\"?>\n\n"
			"<launch>\n";
	char *droneText =
			"\t<group ns=\"drone%d\">\n\t\t"
				"<param name=\"tf_prefix\" value=\"drone%d\"/>\n\t\t"
				"<include file=\"$(find ardrone_autonomy)/launch/ardrone.launch\" >\n\t\t\t"
				"</include>\n\t"
			"</group>\n\n";
	char *endingText = "</launch>";

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
	sprintf(startingBuffer, startingText);
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
void GaTACDroneControl::varyHeights()
{
	/* simulatorMode == true and false */
	string temp;
	std::stringstream strID;
	char publishMessage[BUFLEN];
	const char *variableHeightTakeoff1 = "rostopic pub -1 /drone%s/cmd_vel geometry_msgs/Twist '[0,0,%.2f]' '[0,0,0]'"; //%.2f = speed of altitude increase
	const char *variableHeightTakeoff2 = "rostopic pub -1 /drone%s/cmd_vel geometry_msgs/Twist '[0,0,0]' '[0,0,0]'"; //stops lifting
	for(int k = 0; k < numberOfDrones; k++)
		{ 
		double kDub = (double) k;
		float vHt = (float) (kDub+1.0)/10;
		strID << k;
		temp = strID.str();		
		sprintf(publishMessage, variableHeightTakeoff1, temp.c_str(), vHt);
		system(publishMessage);
		sprintf(publishMessage, variableHeightTakeoff2, temp.c_str());
		system(publishMessage);
		strID.str("");
		dronesSharingSpace.push_back(false);
		}
	cout<< "Varied heights successfully"<<endl;
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
	cout<<"-drone"<<droneId<<" moving-"<<endl;
	sprintf(publishMessage, moveCommand, droneId, xSend, ySend, droneId);
	system(publishMessage);	
	}while ((dx != 0) || (dy != 0));
	if(sharedSpace() == true){
		cout<<"Drones sharing a cell: " << endl; 		
		for(int i = 0; i < dronesSharingSpace.size(); i++)
		{		
		if(dronesSharingSpace.at(i) == true)
		cout<<"  drone"<<i<<" @ ("<<dronePositions.at(i).first<<", "<<dronePositions.at(i).second<<")"<<endl;
		}
	}	
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
	if(id < 0 || id > (numberOfDrones - 1))
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
	if(x > 10 || y > 10)
	return false;
	else
	return true;
}

