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

using std::vector;
using std::pair;

class GaTACDroneControl {
public:

	/*
	 * Default constructor. Initializes all member variables.
	 */
	GaTACDroneControl();

	/*
	 * This method sets up the main UDP socket server. It loops continuously, parsing the input
	 * received from the UDP socket and launching the correct ROS services on the machine it's running on.
	 * The machine running this main server must therefore have all necessary ROS packages installed.
	 */
	void runServer(char *, char *);

	/*
	 * This method sets up the main UDP socket client. Once created, all relevant socket information
	 * is then stored in the current instance of GaTACDroneControl for later communication with the server.
	 */
	void launchClient(char *, char *);

	/*
	 * This method sets up the size of the grid that all subsequently spawned drones will be spawned on.
	 */
	void setGridSize(int, int);

	/*
	 * This method closes the UDP client socket.
	 */
	void closeClient();

	/*
	 * This method sets up a new drone. The size of the grid and initial
	 * position of the drone on that grid must be specified.
	 */
	void setupDrone(int, int);

	/*
	 * This method will start the drone simulator with size and number/location of drones
	 * as specified by previous method calls.
	 */
	void startGrid();

	/*
	 * This method will move the specified drone to the desired (x, y) position.
	 */
	void move(int, int, int);

	/*
	 * This method will land the specified drone.
	 */
	void land(int);

	/*
	 * This method will make the specified drone take off.
	 */
	void takeoff(int);

	/*
	 * This method will trigger the reset mode for the specified drone.
	 */
	void reset(int);

private:
	int serverSocket, numberOfColumns, numberOfRows, numberOfDrones;
	vector<pair<int, int> > dronePositions;//updates at every movement with current position for each drone
	struct addrinfo *srv;
	vector<int> dronesSharingSpace; //for keeping track of shared cells
	bool simulatorMode;
	bool g_gridSizeSet;//global variables due to multi-client use
	bool g_gridStarted;
	int droneSetupId;
	/*
	 * Sends a simple command (takeoff, land, or reset) to the specified drone. Returns a bool value based on whether the message is sent succesfully.
	 */
	bool commandDrone(char, int);

	/*
	 * This method sends the message specified through the main UDP socket
	 * to whatever machine is currently running the drone server. Returns a bool value depending on
	 * whether the specified destination received the message and sent an acknowledgement back.
	 */
	bool sendMessage(char *, int, struct addrinfo *);

	/*
	 * This method launches the Gazebo simulator with a grid of whatever size was specified via the setGridSize method,
	 * and with any drones that have been set up via the setUpDrone method.
	 */
	void launchGazebo();

	/*
	 * Gazebo places the grid at different locations within its own coordinate system depending on the size of the grid.
	 * The user will specify a grid size (A x B), and this method will find the Gazebo coordinates of (0, 0) on the user's grid.
	 */
	void getGazeboOrigin(int&, int&);

	/*
	 * This method modifies the grid_flight.launch file used by Gazebo to start the simulator.
	 * It specifies the grid size and number/starting position of all drones.
	 */
	void configureLaunchFile();
	/*
	 * This method varies the altitude of each drone as a fucntion of their ID
	 * (Larger ID = higher flight)
	 */
	void varyHeights();
	/*
	 * This method takes movement parameters and sends waypoint messages one cell at a time.
	 * As movements are made, it updates the position of moving drone and checks for shared cells
	 * using sharedSpace() method.
	 */
	void moveAndCheck(int, int, int);
	/*
	 * This method compares current locations of drones and returns true if a cell is being shared.
	 * Additionally, it updates a vector of which drones are sharing a cell.
	 */
	bool sharedSpace();

};

#endif
