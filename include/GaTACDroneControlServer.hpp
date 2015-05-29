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
#include <mutex>

using std::vector;
using std::pair;
using std::string;



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

static bool debug_terms = false;

class GaTACDroneControl {
public:
	/**
	 * Default constructor. Initializes all member variables.
	 */
	GaTACDroneControl(bool isReal);

	/**
	 * This method is called by the GaTAC server. It begins a new server thread for each drone started.
	 * @param remoteIP The IP supplied for a client command socket
	 * @param remotePort The port number supplied for a client command socket, by default 4999, 5999, and 6999
	 * @param expectedDrones The number of drones expected for this flight/server session
	 */
	void startServer(const char *, unsigned int, int);

	/**
	 * This method sets up the a thread for reading the drone state from thinc_smart
	 * @param threadNo The ID of the thread this method is starting
	 */
	void dataServer(const int, const int);

	/**
	 * This method sets up the main UDP socket server. It loops continuously, parsing the input
	 * received from the UDP socket and launching the correct ROS services on the machine it's running on.
	 * The machine running this main server must therefore have all necessary ROS packages installed.
	 * @param remoteIP The IP supplied for a client socket
	 * @param remotePort The port number supplied for a client socket
	 * @param threadNo The ID of the thread this method is starting
	 */
	void runServer(int, struct sockaddr_storage *, socklen_t, int, const int);

	/**
	 * This method will return and print the current position of a given drone on the grid.
	 * @param droneId ID of drone to return navdata from
	 * @return Human-readable string denoting the drone's current location on the grid
         */
	string getGridPosition(int);


	/**
	 * This method allows a client to query the server whether another drone is north, south, east, or west of the client's drone on the grid.
	 * @param droneId The drone ID of the client sending sense request.
	 * @param option Integer denoting the direction to sense; 0 -> North, 1 -> South, 2 -> East, 3 -> West
	 * @return 0 if no drone is on that side of subject drone, 1 if another drone is within one square above, 2 if another drone is greater than one square above
	 */
	vector<pair<string, int> > sense(int, int, int);


    void setScenarioIsOver(string msg);

    bool isScenarioOver();


	/**
	 * This method returns whether all requested drones are flying or not
	 * @return Boolean indicating whether all drones have successfully launched
         */
	bool droneStartCheck();

	vector<pid_t> subProcesses;


	/**
	 * This method returns the value of boolean gridStarted, which lets the server know if the grid has been started and the drones' ROS nodes have been initialized
	 * @return Boolean indicating whether the grid has started and ROS nodes have been initialized, true if the size has already been set
         */
	bool gridStartCheck();

private:
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
	std::mutex dronePositionMtx;

	/**
	 * @brief Indicates which clients are ready to receive commands.
	 */
	vector<bool> clientsReady;

	/**
	 * @brief Indicates which drones are flying and ready to move
	 */
	vector<bool> dronesReady;

	vector<string> droneRoles;

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

    bool scenarioOver;
    string scenarioOverMsg;
	std::mutex scenarioOverMtx;


	/**
	 * @brief Used to keep track of currently operating threads.
	 */
	boost::thread* threads[256];

	/**
	 * This method will call the PrintNavdata service to set the drone's data members to the correct values.
	 * @param droneId ID of drone to return navdata from
	 * @return nothing
         */
	void updateData(int);


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
	bool sharedSpace(vector<bool> * dronesSharingSpace);

	/**
	 * This method returns the value of boolean gridSizeSet, which lets the server check if the grid size has been set.
	 * @return Boolean indicating whether the grid size has been set, true if the size has already been set
         */
	bool gridSizeCheck();


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


    void runSubProcess(const char *);
};
#endif
