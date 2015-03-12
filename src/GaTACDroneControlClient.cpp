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
#include "GaTACDroneControlClient.hpp"

using std::cout;
using std::endl;
using std::stringstream;
using std::make_pair;
using std::ofstream;
using std::ifstream;
using std::ios;

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
	datsrv = NULL;
	srv = NULL;
}

/**
 * Overloaded constructor. Used when flying real drones as opposed to the simulator. All members initialized, with bool simulatorMode init'd to false.
 * @param c If char provided to constructor, this gatac object will be used as a server or client object involving REAL drones.
 */
GaTACDroneControl::GaTACDroneControl(const char* c) {
	serverSocket, dataSocket, numberOfColumns, numberOfRows, numberOfDrones = 0;
	datsrv = NULL;
	srv = NULL;
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
	char port[256];
	char dp[256];

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

		char message[BUFLEN];
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
		char message[BUFLEN];
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
		char message[BUFLEN];
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
		char msg[BUFLEN];
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
	char message[BUFLEN];
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
	sleep(1);
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
		char message[BUFLEN];
		sprintf(message, "%c %d", command, droneId);
		success = sendMessage(message, serverSocket, srv);

	return success;
}

