#ifndef GATAC_DRONE_CONTROL_HPP
#define GATAC_DRONE_CONTROL_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <pthread.h>
#include <vector>
#include <utility>
#include <boost/thread.hpp> // For concurrent flight
#include <boost/date_time.hpp>

using std::vector;
using std::pair;

/**
 * @file	GaTACDroneControl.hpp
 * @author  	Vince Capparell, Casey Hetzler
 * @version	1.0
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
	 * Default constructor. Initializes all member variables.
         * If no char provided to constructor, this gatac object will be used as a server or client object involving SIMULATED drones.
	 */
	GaTACDroneControl();

	/**
	 * Overloaded constructor. Used when flying real drones as opposed to the simulator. All members initialized, with bool simulatorMode init'd to false.
	 * @param c If char provided to constructor, this gatac object will be used as a server or client object involving REAL drones.
	 */
	GaTACDroneControl(const char*);

	/**
	 * This method is called by the GaTAC server. It begins a new server thread for each drone started.
	 * @param remoteIP The IP supplied for a client socket
	 * @param remotePort The port number supplied for a client socket
	 * @param expectedDrones The number of drones expected for this flight/server session
	 */
	void startServer(const char *, const char *, int);

	/**
	 * This method sets up the main UDP socket server. It loops continuously, parsing the input
	 * received from the UDP socket and launching the correct ROS services on the machine it's running on.
	 * The machine running this main server must therefore have all necessary ROS packages installed.
	 * @param remoteIP The IP supplied for a client socket
	 * @param remotePort The port number supplied for a client socket
	 * @param threadNo The ID of the thread this method is starting
	 */
	void runServer(const char *, const char *, int);

	/**
	 * This method sets up the main UDP socket client. Once created, all relevant socket information
	 * is then stored in the current instance of GaTACDroneControl for later communication with the server.
 	 * @param serverIp The IP supplied for the server's socket
	 * @param serverPort The port number supplied for the server's socket
	 */
	void launchClient(char *, char *);

	/**
	 * This method sets up the size of the grid that all subsequently spawned drones will be spawned on.
	 * @param numberOfColumns X-axis dimension
	 * @param numberOfRows Y-axis dimension
	 */
	void setGridSize(int, int);

	/**
	 * This method closes the UDP client socket.
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
	 * This method will move the specified drone to the desired (x, y) position.
	 * @param droneId ID of drone to move
	 * @param x Drone's desired position, X-axis
	 * @param y Drone's desired position, Y-axis
	 */
	void move(int, int, int);

	/**
	 * This method will land the specified drone.
	 * @param droneId ID of drone to land
	 */
	void land(int);

	/**
	 * This method allows a drone to hover.
	 * @param droneId ID of drone to hover
	 */
	void hover(int);

	/**
	 * This method will make the specified drone take off.
	 * @param droneId ID of drone to takeoff
	 */
	void takeoff(int);

	/**
	 * This method will trigger the reset mode for the specified drone.
	 * @param droneId ID of drone to reset
	 */
	void reset(int);
    
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
	 * This method will return and print the current battery percentage to the client's display.
	 * @param droneId ID of drone to return navdata from
         */
	void setBattery(std::string);

	/**
	 * This method will return and print the current forward velocity to the client's display.
	 * @param droneId ID of drone to return navdata from
         */
	void setForwardVelocity(std::string);

	/**
	 * This method will return and print the current sideways velocity to the client's display.
	 * @param droneId ID of drone to return navdata from
         */
	void setSidewaysVelocity(std::string);

	/**
	 * This method will return and print the current vertical velocity to the client's display.
	 * @param droneId ID of drone to return navdata from
         */
	void setVerticalVelocity(std::string);

	/**
	 * This method will return and print the current sonar reading to the client's display.
	 * @param droneId ID of drone to return navdata from
         */
	void setSonar(std::string);

	/**
	 * This method will return and print data related to tag spotting.
	 * @param droneId ID of drone to return navdata from
         */
	void setTagsSpotted(std::string);

	/**
	 * This method will return and print the current battery percentage to the client's display.
	 * @param droneId ID of drone to return navdata from
         */
	std::string getBattery();

	/**
	 * This method will return and print the current forward velocity to the client's display.
	 * @param droneId ID of drone to return navdata from
         */
	std::string getForwardVelocity();

	/**
	 * This method will return and print the current sideways velocity to the client's display.
	 * @param droneId ID of drone to return navdata from
         */
	std::string getSidewaysVelocity();

	/**
	 * This method will return and print the current vertical velocity to the client's display.
	 * @param droneId ID of drone to return navdata from
         */
	std::string getVerticalVelocity();

	/**
	 * This method will return and print the current sonar reading to the client's display.
	 * @param droneId ID of drone to return navdata from
         */
	std::string getSonar();

	/**
	 * This method will return and print data related to tag spotting.
	 * @param droneId ID of drone to return navdata from
         */
	std::string getTagsSpotted();

	/**
	 * This method will return and print the current position of a given drone on the grid.
	 * @param droneId ID of drone to return navdata from
	 * @return human-readable string denoting the drone's current location on the grid
         */
	std::string getGridPosition(int);

	/**
	 * This method will call the PrintNavdata service to set the drone's data members to the correct values and return the requested data to the client.
	 * @param droneId ID of drone to return navdata from
	 * @param option Option of which data to receive, calls correct helper method (transparent to user)
	 * @return Human-readable string of characters describing and displaying the value of navdata desired
         */
	void getData(int);
	
	/**
 	 * This method is called by a client to send a senseNorth message to the server.
	 * @param droneId Integer denoting which drone is calling the method
	 */
	void senseNorth(int);
	
	/**
	 * This method is called by a client to send a senseSouth message to the server.
	 * @param droneId Integer denoting which drone is calling the method
	 */
	void senseSouth(int);
	
	/**
	 * This method is called by a client to send a senseEast message to the server.
	 * @param droneId Integer denoting which drone is calling the method
	 */
	void senseEast(int);

	/**
	 * This method is called by a client to send a senseWest message to the server.
	 * @param droneId Integer denoting which drone is calling the method
	 */
	void senseWest(int);

	/**
	 * This message allows a client to query the server whether another drone is north, south, east, or west of the client's drone on the grid.
	 * @param droneId The drone ID of the client sending sense request. 
	 * @param option Integer denoting the direction to sense; 0 -> North, 1 -> South, 2 -> East, 3 -> West
	 * @return 0 if no drone is above client drone, 1 if another drone is within one square above, 2 if another drone is greater than one square above
	 */
	int sense(int, int);
	
private:
	int serverSocket, numberOfColumns, numberOfRows, numberOfDrones;

	/**
	 * @brief Updates at every movement with current position for each drone.
	 */
	vector<pair<int, int> > dronePositions; 

	/**
	 * @brief Struct containing server socket data.
	 */
	struct addrinfo *srv; 

	/**
	 * @brief Updates to keep track of shared cells.
	 */	
	vector<int> dronesSharingSpace; 

	/**
	 * @brief Indicates which clients are ready to receive commands.
	 */	
	vector<bool> clientsReady; 

	/**
	 * @brief Indicates whether the client-server session is operating on real or simulated drones.
	 */
	bool simulatorMode;  
	
	/**
	 * @brief Indicates to server whether the grid size has been set.
	 */
	bool gridSizeSet;
	
	/**
	 * @brief Indicates to server whether the grid has been started and ROS nodes for each drone have been initialized.
	 */
	bool gridStarted;

	/**
	 * @brief Indicates to server the number of threads currently running to process client requests.
	 */
	int serverThreads; 

	/**
	 * @brief ID unique to this client's drone.
	 */
	int clientUniqueId; 

	/**
	 * @brief Current string representation of client's battery navdata
	 */
	std::string clientCurrentBattery;

	/**
	 * @brief Current string representation of client's sonar navdata
	 */
	std::string clientCurrentSonar;

	/**
	 * @brief Current string representation of client's forward velocity navdata
	 */
	std::string clientCurrentForwardVelocity;

	/**
	 * @brief Current string representation of client's sideways velocity navdata
	 */
	std::string clientCurrentSidewaysVelocity;

	/**
	 * @brief Current string representation of client's vertical velocity navdata
	 */
	std::string clientCurrentVerticalVelocity;

	/**
	 * @brief Current string representation of client's tags spotted navdata
	 */
	std::string clientCurrentTagsSpotted;
	
	/**
	 * @brief Indicates to server whether everything is ready before sending out commands.
	 */
	bool readyToCommand;
	
	/**
	 * @brief Used to keep track of currently operating threads.
	 */
	boost::thread* threads[2];

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

	/**
	 * This method launches the Gazebo simulator with a grid of whatever size was specified via the setGridSize method,
	 * and with any drones that have been set up via the setUpDrone method.
	 * 
	 * ***NOTE: When using real drones, this method instead creates/initializes the ROS nodes for each drone.***
	 */
	void launchGrid();

	/**
	 * Gazebo places the grid at different locations within its own coordinate system depending on the size of the grid.
	 * The user will specify a grid size (A x B), and this method will find the Gazebo coordinates of (0, 0) on the user's grid.
	 * @param x User grid's X-axis origin
	 * @param y User grid's Y-axis origin
	 */
	void getGazeboOrigin(int&, int&);

	/**
	 * This method modifies the grid_flight.launch file used by Gazebo to start the simulator.
	 * It specifies the grid size and number/starting position of all drones.
	 * 
	 * ***NOTE: Directory to create launch file in can be specified here.***
	 */
	void configureLaunchFile();
	
	/**
	 * This method varies the altitude of each drone as a function of their ID
	 * (Larger ID = higher flight)
	 * @param droneNumber ID of the drone being elevated
	 */
	void varyHeights(int);
	
	/**
	 * This method takes movement parameters and sends waypoint messages one cell at a time.
	 * As movements are made, it updates the position of moving drone and checks for shared cells
	 * using sharedSpace() method. On completion of desired movement, drone's final destination is printed on the server terminal.
	 * @param x Desired X-axis destination
	 * @param y Desired Y-axis destination
	 * @param Id ID of the drone being moved
	 */
	void moveAndCheck(int, int, int);
	
	/**
	 * This method compares current locations of drones and returns true if a cell is being shared.
	 * Additionally, it updates a vector of which drones are sharing a cell.
   	 * If a shared cell is detected, the coordinates and drone ID's are printed to server terminal.
	 * @return Boolean indicating whether a cell on the grid is occupied by two or more drones; true indicates a shared cell is detected
	 */
	bool sharedSpace();
	
	/**
	 * This method returns the value of boolean gridSizeSet, which lets the server check if the grid size has been set.
	 * @return Boolean indicating whether the grid size has been set, true if the size has already been set
         */	
	bool gridSizeCheck();

	/**
	 * This method returns the value of boolean gridStarted, which lets the server know if the grid has been started and the drones' ROS nodes have been initialized
	 * @return Boolean indicating whether the grid has started and ROS nodes have been initialized, true if the size has already been set
         */	
	bool gridStartCheck();

	/**
	 * This method returns true if a drone id sent by the client is a valid drone id that has previously been spawned.
	 * @param id Drone ID to be verified
	 * @return Boolean indicating whether the given drone ID is currently on the grid, true if the ID is in use and drone is present
         */	
	bool validDroneId(int);

	/**
	 * This method returns true if a location sent by the client is within the bounds of the grid.
	 * @param x X-axis value to be verified
	 * @param y Y-axis value to be verified
	 * @return Boolean indicating the given location is on the grid, true if the location is valid
         */	
	bool validLocation(int, int);
	
	/**
	 * This method returns true if a client-set grid is between 0x0 and 10x10.
	 * @param x X-axis dimension to be verified
	 * @param y Y-axis dimension to be verified
	 * @return Boolean indicating whether the specified grid dimensions are valid, true if valid
         */	
	bool validGridSize(int, int);

	/**
	 * This method returns true if the maximum number of drones has already been spawned.
         * @return Boolean indicating whether 3 drones are already on the grid, true if this is the case
         */	
	bool maxDrones();

};
#endif
