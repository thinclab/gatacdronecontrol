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
using namespace std;

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
	 * @param remoteIP The IP supplied for a client command socket
	 * @param remotePort The port number supplied for a client command socket, by default 4999, 5999, and 6999
	 * @param expectedDrones The number of drones expected for this flight/server session
	 */
	void startServer(const char *, unsigned int, int);

	/**
	 * This method sets up the data socket for each client and listens for navdata requests. It loops continuously, updating the navdata for each client navdata data members.
	 * @param remoteIP The IP supplied for a client data socket
	 * @param remotePort The port number supplied for a client data socket, by default 4998, 5998, and 6998
	 * @param threadNo The ID of the thread this method is starting
	 */
	void dataServer(const char *remoteIp, unsigned int, int threadNo);

	/**
	 * This method sets up the main UDP socket server. It loops continuously, parsing the input
	 * received from the UDP socket and launching the correct ROS services on the machine it's running on.
	 * The machine running this main server must therefore have all necessary ROS packages installed.
	 * @param remoteIP The IP supplied for a client socket
	 * @param remotePort The port number supplied for a client socket
	 * @param threadNo The ID of the thread this method is starting
	 */
	void runServer(const char *, unsigned int, int);

	/**
	 * This method sets up the main UDP socket client. Once created, all relevant socket information
	 * is then stored in the current instance of GaTACDroneControl for later communication with the server.
 	 * @param serverIp The IP supplied for the server's socket
	 * @param serverPort The port number supplied for the server's socket
	 * @param dataPort The port number supplied for the client's navdata socket
	 */
	void launchClient(char *, unsigned int, unsigned int);

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
	 * @param droneId ID of drone to move
	 * @param x Drone's desired position, X-axis
	 * @param y Drone's desired position, Y-axis
	 */
	void move(int, int, int);

	/**
	 * This method is called by the client and will land the specified drone.
	 * @param droneId ID of drone to land
	 */
	void land(int);

	/**
	 * This method is called by the client and allows a drone to hover.
	 * @param droneId ID of drone to hover
	 */
	void hover(int);

	/**
	 * This method is called by the client and will make the specified drone take off.
	 * @param droneId ID of drone to takeoff
	 */
	void takeoff(int);

	/**
	 * This method is called by the client and will trigger the reset mode for the specified drone.
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
	 * This method will call the PrintNavdata service to set the drone's data members to the correct values and return the requested data to the client.
	 * @param droneId ID of drone to return navdata from
	 * @return Character array of all navdata values, to be sent over a socket to the client, broken up into strings of thirty characters, and used to set navdata members
         */
	const char* getData(int);

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
	 * This method allows a client to query the server whether another drone is north, south, east, or west of the client's drone on the grid.
	 * @param droneId The drone ID of the client sending sense request.
	 * @param option Integer denoting the direction to sense; 0 -> North, 1 -> South, 2 -> East, 3 -> West
	 * @return 0 if no drone is on that side of subject drone, 1 if another drone is within one square above, 2 if another drone is greater than one square above
	 */
	int sense(int, int);

	/**
	 * This method allows a client to "opt in" to a continuous stream of navdata to update its data members. Using boost threads, navdata can be accessed concurrently while 	  * drone commands are sent.
	 * @param id The ID of the drone requesting navdata.
	 * @return Boolean confirming the success of request being sent and received.
	 */
	bool receiveData(int);

private:
	/**
	 * @brief Used to keep track of command socket info.
	 */
	int serverSocket;

	/**
	 * @brief Used to keep track of data socket info.
	 */
	int dataSocket;

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
	 * @brief Updates at every movement with current position for each drone.
	 */
	vector<pair<int, int> > dronePositions;

	/**
	 * @brief Struct containing server socket data.
	 */
	struct addrinfo *srv;

	/**
	 * @brief Struct containing data socket data.
	 */
	struct addrinfo *datsrv;

	/**
	 * @brief Updates to keep track of shared cells.
	 */
	vector<bool> dronesSharingSpace;

	/**
	 * @brief Indicates which clients are ready to receive commands.
	 */
	vector<bool> clientsReady;

	/**
	 * @brief Indicates which drones are flying and ready to move
	 */
	vector<bool> dronesReady;

	/**
	 * @brief Current string representation of client's battery navdata
	 */
	static string clientCurrentBattery;

	/**
	 * @brief Current string representation of client's sonar navdata
	 */
	static string clientCurrentSonar;

	/**
	 * @brief Current string representation of client's forward velocity navdata
	 */
	static string clientCurrentForwardVelocity;

	/**
	 * @brief Current string representation of client's sideways velocity navdata
	 */
	static string clientCurrentSidewaysVelocity;

	/**
	 * @brief Current string representation of client's vertical velocity navdata
	 */
	static string clientCurrentVerticalVelocity;

	/**
	 * @brief Current string representation of client's tags spotted navdata
	 */
	static string clientCurrentTagsSpotted;

	/**
	 * @brief Indicates whether the client-server session is operating on real or simulated drones.
	 */
	bool simulatorMode;

	/**
	 * @brief Indicates whether the server and clients are ready to send and receive data through data ports
	 */
	bool readyForData;

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
	 * @brief Indicates to server whether everything is ready before sending out commands.
	 */
	bool readyToCommand;

	/**
	 * @brief Used to keep track of currently operating threads.
	 */
	boost::thread* threads[5];

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
	 * *** NOTE: When using real drones, this method instead creates/initializes the ROS nodes for each drone. ***
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
	 * In simulation, this method modifies the grid_flight.launch file used by Gazebo to start the simulator.
	 * It specifies the grid size and number/starting position of all drones.
	 *
	 * With actual drones, this method uses a modification of ardrone_autonomy's ardrone.launch file, to allow tag spotting and specify other parameters.
	 *
	 * *** NOTE: Directory in which to create launch file can be specified here. ***
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
	 * This method returns whether all requested drones are flying or not
	 * @return Boolean indicating whether all drones have successfully launched
         */
	bool droneStartCheck();

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

	int expectedDrones;

};
#endif
