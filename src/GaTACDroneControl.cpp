#include <iostream>
#include <errno.h> // For printing error #'s/debugging
#include <stdlib.h>
#include <fstream> // For editing files
#include <math.h>

// For tokenizing command input
#include <sstream>

// GaTACDroneControl header
#include "GaTACDroneControl.hpp"

using namespace std;

#define BUFLEN 64
#define DEFAULTCLIENTPORT 4999

GaTACDroneControl::GaTACDroneControl() {
	serverSocket, numberOfColumns, numberOfRows, numberOfDrones = 0;
	gridSizeSet, gridStarted = false;
	simulatorMode = true;
	srv = NULL;
}

void GaTACDroneControl::runServer(char *remoteIp, char *remotePort) {
	const char *publishCommand = "rostopic pub -1 /drone%s/ardrone/%s std_msgs/Empty";
	const char *moveCommand = "rosservice call /drone%s/waypoint %s %s 0 %s"; // x y z id
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
		char receiveBuffer[BUFLEN];
		char publishMessage[BUFLEN];
		int initialColumn, initialRow;

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
			dronePositions.push_back(make_pair(initialColumn, initialRow));
			printf("Ready to spawn drone at [%d, %d].\n", initialColumn, initialRow);
			numberOfDrones++;
			break;

		case 't':
			cout << "Take off." << endl;
			droneNumber = (tokens.at(1)).c_str();
			sprintf(publishMessage, publishCommand, droneNumber, "takeoff");
			system(publishMessage);
			sleep(3); // Wait for takeoff to complete
			break;

		case 'l':
			cout << "Land." << endl;
			droneNumber = (tokens.at(1)).c_str();
			sprintf(publishMessage, publishCommand, droneNumber, "land");
			system(publishMessage);
			sleep(3); // Wait for drone to land completely
			break;

		case 'r':
			cout << "Reset." << endl;
			droneNumber = (tokens.at(1)).c_str();
			sprintf(publishMessage, publishCommand, droneNumber, "reset");
			system(publishMessage);
			sleep(3); // Wait for drone to reset
			break;

		case 'm':
			cout << "Move." << endl;
			droneNumber = (tokens.at(1)).c_str();
			x = (tokens.at(2)).c_str();
			y = (tokens.at(3)).c_str();
			sprintf(publishMessage, moveCommand, droneNumber, x, y, droneNumber);
			system(publishMessage);
			break;

		case 'g':
			cout << "Set grid size." << endl;
			numberOfColumns = atoi(tokens.at(1).c_str());
			numberOfRows = atoi(tokens.at(2).c_str());
			break;

		case 'i':
			cout << "Start gazebo." << endl;
			launchGazebo();
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

	// If grid size hasn't been set yet
	if (gridSizeSet) {
		cout << "Sending command to start grid." << endl;
		char message[2] = "i";

		worked = sendMessage(message, serverSocket, srv);
		if (worked) {
			gridStarted = true;
		}
	}
	// If grid size hasn't been set yet
	else {
		cout << "Error: No grid size set. You must specify a grid size before starting the grid." << endl;
		exit(1);
	}
}

void GaTACDroneControl::setGridSize(int numberOfColumns, int numberOfRows) {
	bool worked = false;

	// If size has already been set
	if (gridSizeSet) {
		cout << "Error: The grid size has already been set.  Specify grid size only once." << endl;
		exit(1);
	}
	// If size isn't valid
	else if (numberOfColumns > 10 || numberOfRows > 10) {
		cout << "Error: The grid size specified was too large. The maximum grid size is 10x10." << endl;
		exit(1);
	}
	// Send command to server
	else {
		printf("Sending command to set grid size to %dx%d.\n", numberOfColumns, numberOfRows);

		char message[10];
		sprintf(message, "g %d %d", numberOfColumns, numberOfRows);
		worked = sendMessage(message, serverSocket, srv);

		if (!worked) {
			cout << "Couldn't set the grid size. Please try again." << endl;
			exit(1);
		}

		gridSizeSet = true;
		this->numberOfColumns = numberOfColumns;
		this->numberOfRows = numberOfRows;
	}
}

void GaTACDroneControl::move(int droneId, int x, int y) {
	bool worked = false;
	string errorMessage;
	string invalidDroneId = "No drone with ID has been spawned. Please specify a valid drone ID.";

	// If grid hasn't been started
	if (!gridStarted) {
		errorMessage = "The grid has not yet been started. Grid must be started before sending commands to a drone.";
	}
	// If drone ID isn't valid
	else if (droneId < 0 || droneId > (numberOfDrones - 1)) {
		errorMessage = invalidDroneId;
	}
	// If destination isn't valid
	else if (x < 0 || y < 0 || x >= numberOfColumns || y >= numberOfRows) {
		errorMessage = invalidDroneId;
	}
	// Send command to server
	else {
		printf("Sending command to move drone #%d to (%d, %d).\n", droneId, x, y);
		char message[10];
		sprintf(message, "m %d %d %d", droneId, x, y);
		worked = sendMessage(message, serverSocket, srv);
	}

	if (!worked) {
		printf("Error: couldn't move drone. %s\n", errorMessage.c_str());
		exit(1);
	}
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
	string errorMessage;

	// If grid size hasn't been set
	if (!gridSizeSet) {
		errorMessage = "No grid size set. You must specify a grid size before spawning a drone.";
		exit(1);
	}
	// If 3 drones already exist
	else if (numberOfDrones == 3) {
		errorMessage = "You have already spawned the maximum number of drones (3).";
		exit(1);
	}
	// If start position isn't valid
	else if (droneCol > (numberOfColumns - 1) || droneRow > (numberOfRows - 1)) {
		errorMessage = "The starting location you specified does not lie within the grid. Please choose a valid starting location.";
		exit(1);
	}
	// Send command to server
	else {
		printf("Sending command to spawn drone with ID #%d at (%d, %d).\n", numberOfDrones, droneCol, droneRow);
		char msg[10];
		sprintf(msg, "s %d %d", droneCol, droneRow);
		worked = sendMessage(msg, serverSocket, srv);

		if (!worked) {
			printf("Error: couldn't spawn drone. %s\n", errorMessage.c_str());
			exit(1);
		}
		numberOfDrones++;
	}
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

	// If droneID isn't valid
	if (droneId < 0 || droneId > (numberOfDrones - 1)) {
		printf("Error: No drone with ID %d has been spawned.  Please specify a valid drone ID.\n", droneId);
		exit(1);
	}
	// Send command to server
	else {
		char message[3];
		sprintf(message, "%c %d", command, droneId);
		success = sendMessage(message, serverSocket, srv);
	}

	return success;
}

void GaTACDroneControl::launchGazebo() {
	const char *gazeboMessage = "xterm -e roslaunch thinc_sim_gazebo test_grid_flight.launch&";
	const char *thincSmartCommand = "ROS_NAMESPACE=drone%d xterm -e rosrun ardrone_thinc thinc_smart %d %d %d %d %d&";
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

void GaTACDroneControl::configureLaunchFile() {
	cout << "Configuring launch file." << endl;

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
					"<arg name=\"spawncoords\" value=\"-x %.2f -y %.2f -z 0.75\"/>\n\t\t"
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
	sprintf(launchFilePath, "/opt/ros/%s/stacks/thinc_simulator/thinc_sim_gazebo/launch/grid_flight.launch", distro);

	// Open file stream
	ofstream fileStream(launchFilePath, ios::trunc);

	// Open file stream
	//ofstream fileStream(
		//	"/home/vincecapparell/fuerte_workspace/sandbox/thinc_simulator/thinc_sim_gazebo/launch/test_grid_flight.launch",
			//ios::trunc);

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

void GaTACDroneControl::getGazeboOrigin(int& x, int& y) {
	x = (-1) * (numberOfRows - 1);
	y = numberOfColumns - 1;
}
