#ifndef GATAC_DRONE_CONTROL_CLIENT_HPP
#define GATAC_DRONE_CONTROL_CLIENT_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <utility>
#include <boost/date_time.hpp>
#include <mutex>
#include <vector>

using std::pair;
using std::string;
using std::vector;


typedef typename std::vector<std::pair<std::string, int>> percept;

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

class GaTACDroneControl {
public:

	/**
	 * @param c The role name of the drone, NOTE WHITESPACE IS AUTOMATICALLY REMOVED
	 */
	GaTACDroneControl(const string);

	/**
	 * This method sets up the main UDP socket client. Once created, all relevant socket information
	 * is then stored in the current instance of GaTACDroneControl for later communication with the server.
 	 * @param serverIp The IP supplied for the server's socket
	 * @param serverPort The port number supplied for the server's socket
	 * @param dataPort The port number supplied for the client's navdata socket
	 */
	void launchClient(string, unsigned int);

	/**
	 * This method sets up the size of the grid that all subsequently spawned drones will be spawned on.
	 * @param numberOfColumns X-axis dimension
	 * @param numberOfRows Y-axis dimension
	 */
	void setGridSize(int, int);

	/**
	 * This method closes the UDP client socket, as well as the navdata socket, and sets the client's readyToCommand boolean to false.
	 */
	void closeClient();

	/**
	 * This method sets up a new drone. The size of the grid and initial
	 * position of the drone on that grid must be specified.
	 * @param droneCol Drone's initial position, X-axis
	 * @param droneRow Drone's intiial position, Y-axis
	 */
	void setupDrone(int, int);

	/**
	 * This method will start the drone simulator with size and number/location of drones
	 * as specified by previous method calls.
	 */
	void startGrid();

	/**
	 * This method is called by a client to send a ready message in multi-client environments.
	 * When a server has received one from each client, it makes the decision to start the grid.
	 */
	void readyUp();

	/**
	 * This method is called by the client and will move the specified drone to the desired (x, y) position.
	 * @param x Drone's desired position, X-axis
	 * @param y Drone's desired position, Y-axis
	 */
	void move(int, int);

	/**
	 * This method is called by the client and will land the specified drone.
	 * @param droneId ID of drone to land
	 */
	void land();

	/**
	 * This method is called by the client and will land the specified drone at the current location.
	 * @param droneId ID of drone to land
	 */
	void landHere();

	/**
	 * This method is called by the client and allows a drone to hover.
	 * @param droneId ID of drone to hover
	 */
	void hover();

	/**
	 * This method is called by the client and will make the specified drone take off.
	 * @param droneId ID of drone to takeoff
	 */
	void takeoff();

	/**
	 * This method is called by the client and will trigger the reset mode for the specified drone.
	 * @param droneId ID of drone to reset
	 */
	void reset();

	/**
	 * Used to set a client's unique drone ID
	 * @param toSet Integer to set this GaTAC instance's drone ID to (0, 1, or 2)
	 */
	void setClientUniqueId(int);

	/**
	 * Used to get a client's unique drone ID
	 * @return Client's unique drone ID
	 */
	int getClientUniqueId();

	/**
	 * Used to get a client's "readyForCommands" boolean value
	 * @return Boolean value that tells whether a client is ready to receive commands
	 */
	bool getClientReadyToCommand();

	/**
	 * Used to set a client's "readyForCommands" boolean value
	 * @param toSet Boolean specifies whether or not client is ready to receive server commands
	 */
	void setClientReadyToCommand(bool);

	/**
	 * This method will set the GaTAC clientCurrentBattery data member to the correct navdata value.
	 * @param toSet string to set battery to
         */
	void setBattery(string);

	/**
	 * This method will set the GaTAC clientCurrentForwardVelocity data member to the correct navdata value.
	 * @param toSet string to set forward velocity to
         */
	void setForwardVelocity(string);

	/**
	 * This method will set the GaTAC clientCurrentSidewaysVelocity data member to the correct navdata value.
	 * @param toSet string to set sideways velocity to
         */
	void setSidewaysVelocity(string);

	/**
	 * This method will set the GaTAC clientCurrentVerticalVelocity data member to the correct navdata value.
	 * @param toSet string to set vertical velocity to
         */
	void setVerticalVelocity(string);

	/**
	 * This method will set the GaTAC clientCurrentSonar data member to the correct navdata value..
	 * @param toSet string to set sonar to
         */
	void setSonar(string);

	/**
	 * This method will set the GaTAC clientCurrentTagsSpotted data member to the correct navdata value.
	 * @param toSet string to set tags spotted data to
         */
	void setTagsSpotted(string);

	/**
	 * This method will return and print the current battery percentage to the client's display.
	 * @return Returns the current battery value in a human-readable string
         */
	string getBattery();

	/**
	 * This method will return and print the current forward velocity to the client's display.
	 * @return Returns the current forward velocity value in a human-readable string
         */
	string getForwardVelocity();

	/**
	 * This method will return and print the current sideways velocity to the client's display.
	 * @return Returns the current sideways velocity value in a human-readable string
         */
	string getSidewaysVelocity();

	/**
	 * This method will return and print the current vertical velocity to the client's display.
	 * @return Returns the current vertical velocity value in a human-readable string
         */
	string getVerticalVelocity();

	/**
	 * This method will return and print the current sonar reading to the client's display.
	 * @return Returns the current sonar value in a human-readable string
         */
	string getSonar();

	/**
	 * This method will return and print the current number of tags spotted to the client's display.
	 * @return Human-readable string of current tag data
         */
	string getTagsSpotted();

	/**
	 * This method will return and print the current position of a given drone on the grid.
	 * @param droneId ID of drone to return navdata from
	 * @return Human-readable string denoting the drone's current location on the grid
         */
	string getGridPosition(int);

	/**
 	 * This method is called by a client to send a senseNorth message to the server.
	 * @param droneId Integer denoting which drone is calling the method
	 */
	vector<pair<string, int>> senseNorth(int);

	/**
	 * This method is called by a client to send a senseSouth message to the server.
	 * @param droneId Integer denoting which drone is calling the method
	 */
	vector<pair<string, int>> senseSouth(int);

	/**
	 * This method is called by a client to send a senseEast message to the server.
	 * @param droneId Integer denoting which drone is calling the method
	 */
	vector<pair<string, int>> senseEast(int);

	/**
	 * This method is called by a client to send a senseWest message to the server.
	 * @param droneId Integer denoting which drone is calling the method
	 */
	vector<pair<string, int>> senseWest(int);

    void sendScenarioIsOver(string msg);

    bool isScenarioOver();
    string getScenarioOverMessage();

private:

    /**
     *  @brief Used to identify myself to the other drones
     */
    string myRole;

	/**
	 * @brief Used to keep track of command socket info.
	 */
	int serverSocket;

	/**
	 * @brief Used to store grid dimensions.
	 */
	int numberOfColumns;

	/**
	 * @brief Used to store grid dimensions.
	 */
	int numberOfRows;

	/**
	 * @brief Used to keep track of number of drones operating.
	 */
	int numberOfDrones;

	/**
	 * @brief Struct containing server socket data.
	 */
	struct addrinfo *srv;

	/**
	 * @brief Current string representation of client's battery navdata
	 */
	string clientCurrentBattery;

	/**
	 * @brief Current string representation of client's sonar navdata
	 */
	string clientCurrentSonar;

	/**
	 * @brief Current string representation of client's forward velocity navdata
	 */
	string clientCurrentForwardVelocity;

	/**
	 * @brief Current string representation of client's sideways velocity navdata
	 */
	string clientCurrentSidewaysVelocity;

	/**
	 * @brief Current string representation of client's vertical velocity navdata
	 */
	string clientCurrentVerticalVelocity;

	/**
	 * @brief Current string representation of client's tags spotted navdata
	 */
	string clientCurrentTagsSpotted;

	/**
	 * @brief ID unique to this client's drone.
	 */
	int clientUniqueId;

	/**
	 * @brief Indicates to server whether everything is ready before sending out commands.
	 */
	bool readyToCommand;

    bool scenarioOver;
    string scenarioOverMessage;

	/**
	 * Sends a simple command (takeoff, land, or reset) to the specified drone. Returns a bool value based on whether the message is sent succesfully.
	 * @param command Command to send to client, indicated by a single char
	 * @param droneId ID of drone in question
	 * @return Boolean value true if message successfully sent and echoed, false if there is a miscommunication
	 */
	bool commandDrone(char, int);

	/**
	 * This method sends the message specified through the main UDP socket
	 * to whatever machine is currently running the drone server. Returns a bool value depending on
	 * whether the specified destination received the message and sent an acknowledgement back.
	 * @param message The message to be sent, as a string of characters
	 * @param socket The socket to send to
	 * @param addrinfo A struct of socket address information
	 * @return Boolean value true if message successfully sent and echoed, false if there is a miscommunication
	 */
	bool sendMessage(char *, int, struct addrinfo *);


    vector<pair<string, int>> percepts;
};
#endif
