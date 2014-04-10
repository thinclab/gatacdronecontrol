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

class GaTACDroneControl {
public:
	/*
	 * Default constructor. Initializes all member variables.
	 */
	GaTACDroneControl();

	/*
	 * Overloaded constructor. Used when flying real drones as opposed to the simulator. All members initialized, with bool simulatorMode init'd to false.
	 */
	GaTACDroneControl(const char* c);

	/*
	 * This method is called by the GaTAC server. It begins a new server thread for each drone started.
	 */
	void startServer(const char *, const char *, int);
	/*
	 * This method sets up the main UDP socket server. It loops continuously, parsing the input
	 * received from the UDP socket and launching the correct ROS services on the machine it's running on.
	 * The machine running this main server must therefore have all necessary ROS packages installed.
	 */
	void runServer(const char *, const char *, int);

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
	 * This method is called by a client to send a ready message in multi-client environments.
	 * When a server has received one from each client, it makes the decision to start the grid.
	 */
	void readyUp();
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
    
	/*
	 * Used to set a client's unique drone ID
	 */
	void setClientUniqueId(int);

	/*
	 * Used to get a client's unique drone ID
	 */
	int getClientUniqueId();
	
	/*
	 * Used to get a client's "readyForCommands" boolean value
	 */
	bool getClientReadyToCommand();

	/*
	 * Used to set a client's "readyForCommands" boolean value
	 */
	void setClientReadyToCommand(bool);
	
private:
	int serverSocket, numberOfColumns, numberOfRows, numberOfDrones;
	vector<pair<int, int> > dronePositions;//updates at every movement with current position for each drone
	struct addrinfo *srv;
	vector<int> dronesSharingSpace; //for keeping track of shared cells
	vector<bool> clientsReady; //for keeping track of shared cells
	bool simulatorMode;
	bool gridSizeSet;//global variables due to multi-client use
	bool gridStarted;
	int serverThreads;
	int clientUniqueId; //used to assign a unique drone ID to a client
	bool readyToCommand; //so the server can ensure everything is ready before receiving commands
	boost::thread* threads[2]; // used for multi threaded server

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
	void varyHeights(int);
	
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
	
	/*
	 * This method returns the value of boolean gridSizeSet, which lets the server check if the grid size has been set.
         */	
	bool gridSizeCheck();

	/*
	 * This method returns the value of boolean gridStarted, which lets the server check if the grid size has been set.
         */	
	bool gridStartCheck();

	/*
	 * This method returns true if a drone id sent by the client is a valid drone id that has previously been spawned.
         */	
	bool validDroneId(int);

	/*
	 * This method returns true if a location sent by the client is within the bounds of the grid.
         */	
	bool validLocation(int, int);
	
	/*
	 * This method returns true if a client-set grid is between 0x0 and 10x10.
         */	
	bool validGridSize(int, int);

	/*
	 * This method returns true if the maximum number of drones has already been spawned.
         */	
	bool maxDrones();
	

};

#endif
