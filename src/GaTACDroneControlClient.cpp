#include <iostream>
#include <errno.h> // For printing error #'s/debugging
#include <stdlib.h>
#include <stdio.h>
#include <sstream> // For converting chars to ints to update positions vector
#include <fstream> // For editing files
#include <boost/thread.hpp> // For concurrent flight
#include <boost/date_time.hpp>
#include <algorithm>
#include <cctype>
#include <locale>

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
#define DEFAULTRONDPORT 4999

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



/**
 * Overloaded constructor. Used when flying real drones as opposed to the simulator. All members initialized, with bool simulatorMode init'd to false.
 * @param c If char provided to constructor, this gatac object will be used as a server or client object involving REAL drones.
 */
GaTACDroneControl::GaTACDroneControl(string c) {
//    c.erase(std::remove_if(c.begin(), c.end(),[](char c){ return (c =='\r' || c =='\t' || c == ' ' || c == '\n');} ), c.end());
    c.erase(std::remove_if(c.begin(), c.end(), []( char ch ) { return std::isspace<char>( ch, std::locale::classic() ); } ), c.end());
    if (c.size() == 0) {
        cout << "Drone Role name cannot be whitespace or zero length." << endl;
        exit(1);
    }
    cout << "Drone Role Name: " << c << endl;
	serverSocket, numberOfColumns, numberOfRows, numberOfDrones = 0;
	srv = NULL;
	myRole = c;
	scenarioOver = false;
	scenarioOverMessage = "";
}



/**
 * This method sets up the main UDP socket client. Once created, all relevant socket information
 * is then stored in the current instance of GaTACDroneControl for later communication with the server.
 * @param serverIp The IP supplied for the server's socket
 * @param serverPort The port number supplied for the server's socket
 * @param dataPort The port number supplied for the client's navdata socket
 */
void GaTACDroneControl::launchClient(string serverIp, unsigned int rondPort) {
	const char *host = serverIp.c_str();
	char port[256];

    sprintf(port, "%u", rondPort);

	int errorCheck, sock;

	struct addrinfo hints, *srv, *info;

    // Connect to the server and get our connection info



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

	char buffer[BUFLEN];
	sprintf(buffer, "CONNECT");

    int bytesSent = 0;
	if ((bytesSent = sendto(sock, buffer, strlen(buffer), 0, info->ai_addr, info->ai_addrlen)) == -1) {
		cout << "Error sending message to server with errno: " << errno << endl;
		exit(1);
	}

	int bytesReceived = 0;
	if ((bytesReceived = recvfrom(sock, buffer, BUFLEN - 1, 0, NULL, NULL)) == -1) {
		cout << "Error receiving feedback from server." << endl;
		exit(1);
	}
	buffer[bytesReceived] = '\0';

    unsigned int controlPort;
    sscanf(buffer, "%u %u", &clientUniqueId, &controlPort);

    freeaddrinfo(srv);
    close(sock);

    sprintf(port, "%u", controlPort);


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
	sleep(1);
}

/**
 * This method will start the drone simulator with size and number/location of drones
 * as specified by previous method calls.
 */
void GaTACDroneControl::startGrid() {
    if (scenarioOver)
        return;

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
    if (scenarioOver)
        return;

	bool worked = false;
		cout << "Sending ready message to server." << endl;
		char message[2] = "y";
		worked = sendMessage(message, serverSocket, srv);
}

/**
 * This method is called by a client to send a senseNorth message to the server.
 * @param droneId Integer denoting which drone is calling the method
 */
vector<pair<string, int>> GaTACDroneControl::senseNorth(int maxdist) {
    percepts.clear();
    if (scenarioOver)
        return percepts;

	printf("Sending sense north command, drone #%d.\n", clientUniqueId);
	// Send command to server
    char message[BUFLEN];
	sprintf(message, "u %d %d", clientUniqueId, maxdist);
	bool worked = sendMessage(message, serverSocket, srv);
	if (!worked) {
		cout << "Couldn't push sense request. Please try again." << endl;
		exit(1);
	}
	return percepts;
}

/**
 * This method is called by a client to send a senseSouth message to the server.
 * @param droneId Integer denoting which drone is calling the method
 */
vector<pair<string, int>> GaTACDroneControl::senseSouth(int maxdist) {
    percepts.clear();
    if (scenarioOver)
        return percepts;


	printf("Sending sense south command, drone #%d.\n", clientUniqueId);
	// Send command to server
    char message[BUFLEN];
	sprintf(message, "d %d %d", clientUniqueId, maxdist);
	bool worked = sendMessage(message, serverSocket, srv);
	if (!worked) {
		cout << "Couldn't push sense request. Please try again." << endl;
		exit(1);
	}
	return percepts;
}

/**
 * This method is called by a client to send a senseEast message to the server.
 * @param droneId Integer denoting which drone is calling the method
 */
vector<pair<string, int>> GaTACDroneControl::senseEast(int maxdist) {
    percepts.clear();
    if (scenarioOver)
        return percepts;


	printf("Sending sense east command, drone #%d.\n", clientUniqueId);
	// Send command to server
    char message[BUFLEN];
	sprintf(message, "k %d %d", clientUniqueId, maxdist);
	bool worked = sendMessage(message, serverSocket, srv);
	if (!worked) {
		cout << "Couldn't push sense request. Please try again." << endl;
		exit(1);
	}
	return percepts;
}

/**
 * This method is called by a client to send a senseWest message to the server.
 * @param droneId Integer denoting which drone is calling the method
 */
vector<pair<string, int>> GaTACDroneControl::senseWest(int maxdist) {
    percepts.clear();
    if (scenarioOver)
        return percepts;


	printf("Sending sense west command, drone #%d.\n", clientUniqueId);
	// Send command to server
    char message[BUFLEN];
	sprintf(message, "j %d %d", clientUniqueId, maxdist);
	bool worked = sendMessage(message, serverSocket, srv);
	if (!worked) {
		cout << "Couldn't push sense request. Please try again." << endl;
		exit(1);
	}
	return percepts;
}

/**
 * This method sets up the size of the grid that all subsequently spawned drones will be spawned on.
 * @param numberOfColumns X-axis dimension
 * @param numberOfRows Y-axis dimension
 */
void GaTACDroneControl::setGridSize(int numberOfColumns, int numberOfRows) {
    if (scenarioOver)
        return;

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
void GaTACDroneControl::move(int x, int y) {
    if (scenarioOver)
        return;

	bool worked = false;
	// Send command to server, checks for valid ID and location done server-side
		printf("Sending command to move drone #%d to (%d, %d).\n", clientUniqueId, x, y);
		char message[BUFLEN];
		sprintf(message, "m %d %d %d", clientUniqueId, x, y);
		worked = sendMessage(message, serverSocket, srv);
}

/**
 * This method is called by the client and allows a drone to hover.
 * @param droneId ID of drone to hover
 */
void GaTACDroneControl::hover() {
    if (scenarioOver)
        return;


	// Send command to server
	bool worked = commandDrone('h', clientUniqueId);
	printf("Sending command to make drone hover.\n");
		char message[BUFLEN];
		sprintf(message, "h %d", clientUniqueId);
		worked = sendMessage(message, serverSocket, srv);
}

/**
 * This method is called by the client and will land the specified drone.
 * @param droneId ID of drone to land
 */
void GaTACDroneControl::land() {
    if (scenarioOver)
        return;

	printf("Sending command to land drone #%d.\n", clientUniqueId);

	// Send command to server
	bool worked = commandDrone('l', clientUniqueId);
	if (!worked) {
		cout << "Couldn't land drone. Please try again." << endl;
		exit(1);
	}
}

/**
 * This method is called by the client and will land the specified drone at its current location.
 * @param droneId ID of drone to land
 */
void GaTACDroneControl::landHere() {
    if (scenarioOver)
        return;

	printf("Sending command to land drone #%d where it is.\n", clientUniqueId);

	// Send command to server
	bool worked = commandDrone('L', clientUniqueId);
	if (!worked) {
		cout << "Couldn't land drone. Please try again." << endl;
		exit(1);
	}
}

/**
 * This method is called by the client and will make the specified drone take off.
 * @param droneId ID of drone to takeoff
 */
void GaTACDroneControl::takeoff() {
    if (scenarioOver)
        return;

	printf("Sending command to takeoff drone #%d.\n", clientUniqueId);

	// Send command to server
	bool worked = commandDrone('t', clientUniqueId);
	if (!worked) {
		cout << "Couldn't take off. Please try again." << endl;
		exit(1);
	}
}

/**
 * This method is called by the client and will trigger the reset mode for the specified drone.
 * @param droneId ID of drone to reset
 */
void GaTACDroneControl::reset() {
    if (scenarioOver)
        return;

	printf("Sending command to reset drone #%d.\n", clientUniqueId);

	// Send command to server
	bool worked = commandDrone('r', clientUniqueId);
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
    if (scenarioOver)
        return;

	bool worked = false;
	// Send command to server
		printf("Sending command to spawn drone at (%d, %d).\n", droneCol, droneRow);
		char msg[BUFLEN];
		sprintf(msg, "s %s %d %d", myRole.c_str(), droneCol, droneRow);
		worked = sendMessage(msg, serverSocket, srv);
}

/**
 * This method closes the UDP client socket, as well as the navdata socket, and sets the client's readyToCommand boolean to false.
 */
void GaTACDroneControl::closeClient() {
	close(serverSocket);
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
	if ((bytesSent = sendto(socket, sendBuffer, strlen(sendBuffer), 0, addrInfo->ai_addr, addrInfo->ai_addrlen)) == -1) {
		cout << "Error sending message to server with errno: " << errno << endl;
		success = false;
	}

	// Waiting for feedback (ensuring that the server received our message)
	int bytesReceived = 0;
	if ((bytesReceived = recvfrom(socket, receiveBuffer, BUFLEN - 1, 0, NULL, NULL)) == -1) {
		cout << "Error receiving feedback from server." << endl;
		exit(1);
	} else {
        // Check if the scenario has ended
        receiveBuffer[bytesReceived] = '\0'
        ;
        if (receiveBuffer[0] == 'X')
        {
            scenarioOver = true;
            readyToCommand = false;
            scenarioOverMessage = string(&(receiveBuffer[2]));
            success = true;
        } else
        {
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
                cout << receiveBuffer <<endl;
                // process the returned value into percepts

                percepts.clear();

                string stringCommand(receiveBuffer);
                stringstream ss(stringCommand);

                std::istream_iterator<std::string> begin(ss);
                std::istream_iterator<std::string> end;

                // Storing tokens in vector
                vector<string> tokens(begin, end);

                for (int i = 0; i < tokens.size() / 2; i ++) {
                    percepts.push_back(make_pair(tokens.at((i * 2) + 1), atoi(tokens.at((i * 2) + 2).c_str())));
                }

            } else {
                cout << "Error: Server didn't receive the command. Exiting." << endl;
                success = false;
            }
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


void GaTACDroneControl::sendScenarioIsOver(string msg) {
	bool worked = false;
	// Send command to server
    printf("Sending command to end the scenario.\n");

    char message[BUFLEN];
    sprintf(message, "X %s", msg.c_str());
    worked = sendMessage(message, serverSocket, srv);

    if (!worked) {
        cout << "Couldn't send the end scenario command. Please try again." << endl;
        exit(1);
    }
}

bool GaTACDroneControl::isScenarioOver() {
    return scenarioOver;
}


string GaTACDroneControl::getScenarioOverMessage() {
    return scenarioOverMessage;
}
