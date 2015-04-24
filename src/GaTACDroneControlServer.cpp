#include <iostream>
#include <errno.h> // For printing error #'s/debugging
#include <stdlib.h>
#include <stdio.h>
#include <sstream> // For converting chars to ints to update positions vector
#include <fstream> // For editing files
#include <boost/thread.hpp> // For concurrent flight
#include <boost/date_time.hpp>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>


// For tokenizing command input
#include <sstream>

// GaTACDroneControl header
#include "GaTACDroneControlServer.hpp"

using std::cout;
using std::endl;
using std::stringstream;
using std::make_pair;
using std::ofstream;
using std::ifstream;
using std::istream;
using std::ios;

#define BUFLEN 512
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



class Locker {
public:
	Locker(std::mutex * l) {
		theLock = l;
		isLocked = false;
		lock();
	}

	~Locker() {
		unlock();
	}

	void lock() {
		if (! isLocked)
			theLock->lock();
		isLocked = true;
	}

	void unlock() {
		if (isLocked)
			theLock->unlock();
		isLocked = false;
	}
private:
	bool isLocked;
	std::mutex * theLock;
};


int open_udp_port(unsigned int portnum, int * sockout) {
    char localPort[256];
    sprintf(localPort, "%u", portnum);

	int errorCheck, rondsock;
	struct addrinfo rondhints, *rondsrv, *rondinfo;

	// Specifying socket parameters
	bzero(&rondhints, sizeof rondhints);
	rondhints.ai_family = AF_UNSPEC;
	rondhints.ai_socktype = SOCK_DGRAM;
	rondhints.ai_flags = AI_PASSIVE; // use my IP

	// Filling 'srv' object with info from 'hints'
	if ((errorCheck = getaddrinfo(NULL, localPort, &rondhints, &rondsrv)) != 0) {
		perror("Server: get address info");
		return errorCheck;
	}

	// Creating and binding socket.
	for (rondinfo = rondsrv; rondinfo != NULL; rondinfo = rondinfo->ai_next) {
		if ((rondsock = socket(rondinfo->ai_family, rondinfo->ai_socktype, rondinfo->ai_protocol)) == -1) {
			perror("Server: socket");
			continue;
		}

		if (bind(rondsock, rondinfo->ai_addr, rondinfo->ai_addrlen)) {
			close(rondsock);
			perror("Server: bind");
			continue;
		}

		break;
	}

	// Ensuring valid address info was found
	if (rondinfo == NULL) {
		perror("Server: no valid address info found.\n");
		return -1;
	}

	*sockout = rondsock;
	freeaddrinfo(rondsrv);
	return 0;
}


pid_t system2(const char * command) {
    pid_t returnval;

    returnval = fork();

    if (returnval != 0) {
        return returnval;

    } else {
        setsid();
        if (debug_terms) {
            char * bashcommand = (char*)alloca(sizeof(char) * (20 + strlen(command)));
            strcpy(bashcommand, "/bin/bash -l -c \"");
            strcat(bashcommand, command);
            strcat(bashcommand, "\"");
            execlp("xterm", "xterm", "-e", bashcommand, (char*)NULL);
        } else {
            execlp("/bin/bash", "/bin/bash", "-l", "-c", command, (char*)NULL);
        }

        exit(1);
    }
}

static GaTACDroneControl * gatacref;

void handlesigint(int sig) {

    for (int i = 0; i < gatacref->subProcesses.size(); i ++) {
        kill (gatacref->subProcesses.at(i), SIGINT);
    }
    exit(1);

}

/**
 * Default constructor. Initializes all member variables.
 * If no char provided to constructor, this gatac object will be used as a server or client object involving SIMULATED drones.
 */
GaTACDroneControl::GaTACDroneControl() {
	numberOfColumns = numberOfRows = numberOfDrones = 0;
	gridSizeSet = gridStarted = false;
	simulatorMode = true;
	serverThreads = 0;
	readyForData = false;
    scenarioOver = false;
    scenarioOverMsg = "";

    gatacref = this;

    signal(SIGINT, handlesigint);
}

/**
 * Overloaded constructor. Used when flying real drones as opposed to the simulator. All members initialized, with bool simulatorMode init'd to false.
 * @param c If char provided to constructor, this gatac object will be used as a server or client object involving REAL drones.
 */
GaTACDroneControl::GaTACDroneControl(const char* c) {
	numberOfColumns = numberOfRows = numberOfDrones = 0;
	gridSizeSet = gridStarted = false;
	simulatorMode = false;
	serverThreads = 0;
	readyForData = false;
	scenarioOver = false;
	scenarioOverMsg = "";

    gatacref = this;

	signal(SIGINT, handlesigint);
}


/**
 * This method is called by the GaTAC server. It begins a new server thread for each drone started.
 * @param remoteIP The IP supplied for a client command socket
 * @param remotePort The port number supplied for a client command socket, by default 4999, 5999, and 6999
 * @param expectedDrones The number of drones expected for this flight/server session
 */
void GaTACDroneControl::startServer(const char *remoteIP, unsigned int remotePort, int expectedDrones){
	cout << "Main server running." << endl;

	this->expectedDrones = expectedDrones;
	for (int i = 0; i < expectedDrones; i ++) {
        clientsReady.push_back(false);
        dronesReady.push_back(false);
	}
	dronePositions.resize(expectedDrones);
	droneRoles.resize(expectedDrones);

    // open rondevous port

    int rondsock;
   	struct sockaddr_storage client_addr;
	socklen_t addr_len = sizeof client_addr;


    if (open_udp_port(remotePort, &rondsock) != 0) {
        exit(1);
    }

    size_t bytesReceived;
	for(int i = 0; i < expectedDrones; i++)
	{

        // wait for a connect message from a client

        do {
            char receiveBuffer[BUFLEN];

            if ((bytesReceived = recvfrom(rondsock, receiveBuffer, BUFLEN - 1, 0, (struct sockaddr *) &client_addr, &addr_len)) == -1) {
                perror("Rondevous: Error receiving command.");
                exit(1);
            }
            receiveBuffer[bytesReceived] = '\0';

            if (strcmp("CONNECT", receiveBuffer) == 0) {
                break;
            }
            perror("Rondevous: Invalid Command received.");

        } while (true);

        // open control and data ports


        int controlsock, datsock;
        unsigned int controlport, datport;
        struct sockaddr_in adr_inet;
        socklen_t len_inet = sizeof adr_inet;

        if (open_udp_port(0, &controlsock) != 0) {
            exit(1);
        }

        if (getsockname(controlsock, (struct sockaddr *)&adr_inet, &len_inet) != 0) {
            perror("Rondevous: getsockname");
            exit(1);
        }
        controlport = ntohs(adr_inet.sin_port);

        if (open_udp_port(0, &datsock) != 0) {
            exit(1);
        }

        if (getsockname(datsock, (struct sockaddr *)&adr_inet, &len_inet) != 0) {
            perror("Rondevous: getsockname");
            exit(1);
        }
        datport = ntohs(adr_inet.sin_port);

        // start threads

        // save a copy of the client's address so that we don't overwrite it before the threads have a chance to make their own copies
        struct sockaddr_storage * client_addr_2;
        client_addr_2 = (sockaddr_storage*)malloc(addr_len);
        memcpy(client_addr_2, &client_addr, addr_len);

		serverThreads++;
		boost::thread* thread;
		thread = new boost::thread(boost::bind(&GaTACDroneControl::runServer,this,controlsock, client_addr_2, addr_len, serverThreads, i));
		threads[2*i] = thread;
		cout<<"starting thread "<<serverThreads <<endl;

		serverThreads++;
		thread = new boost::thread(boost::bind(&GaTACDroneControl::dataServer,this,datsock, client_addr_2, addr_len, serverThreads, i));
		threads[2*i + 1] = thread;
		cout<<"starting data thread "<<serverThreads<<endl;


        // inform client of ready state
        char sendBuffer[BUFLEN];

        sprintf(sendBuffer, "%u %u %u", i, controlport, datport);
        int numSent;
		if ((numSent = sendto(datsock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
			perror("Server: error sending client initialization.");
			exit(1);
		}

	}

    cout << "All clients connected" << endl;

	close(rondsock);

	for (int i = 0; i < 2 * expectedDrones; i ++) {
		threads[i]->join();
		delete threads[i];
	}

	// kill all started child processes here

    for (int i = 0; i < subProcesses.size(); i ++) {
        kill (subProcesses.at(i), SIGINT);
    }
}
/**
 * This method sets up the data socket for each client and listens for navdata requests. It loops continuously, updating the navdata for each client navdata data members.
 * @param remoteIP The IP supplied for a client data socket
 * @param remotePort The port number supplied for a client data socket, by default 4998, 5998, and 6998
 * @param threadNo The ID of the thread this method is starting
 */
void GaTACDroneControl::dataServer(int datsock_in, struct sockaddr_storage * client_addr_in, socklen_t addr_len_in, int threadNo_in, const int myDroneId_in) {

    int datsock = datsock_in;
    struct sockaddr_storage client_addr;
    memcpy(&client_addr, client_addr_in, addr_len_in);
    socklen_t addr_len = addr_len_in;
    int threadNo = threadNo_in;
    const int myDroneId = myDroneId_in;

	signal(SIGINT, handlesigint);

    struct timeval tv;

    tv.tv_sec = 0;  /* 30 Secs Timeout */
    tv.tv_usec = 500000l;  // Not init'ing this can cause strange errors

    setsockopt(datsock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));

	// Loop forever. Read commands from socket and perform the action specified.
	int bytesReceived = 0;
	while (!isScenarioOver()) {
		char receiveBuffer[BUFLEN];
		char publishMessage[BUFLEN];

		if ((bytesReceived = recvfrom(datsock, receiveBuffer, BUFLEN - 1, 0, (struct sockaddr *) &client_addr, &addr_len)) == -1) {
            if(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                continue;
            } else {
                perror("Error receiving command.");
                exit(1);
			}
		}
		receiveBuffer[bytesReceived] = '\0';

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
		case 'n':
			break;

		default:
			cout << "Error parsing nav command - invalid command character received." << endl;
			break;
		}

		// Sending acknowledgment message to client
		int len = BUFLEN;
		char sendBuffer[len];
		char navBuffer[BUFLEN];
		strcpy(sendBuffer, getData(myDroneId));
		//If command received was to spawn a drone, first char of ACK set to id
		if(rawCommand == 'n'){
		int numSent = 0;
		sleep(5);
		if ((numSent = sendto(datsock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
			perror("Server: error sending acknowledgment.");
			exit(1);
		}
		}
		else{
		int numSent = 0;
		if ((numSent = sendto(datsock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
			perror("Server: error sending acknowledgment.");
			exit(1);
		}
		}
	}
	// Cleaning up socket information
	close(datsock);
	cout << "Data thread " << threadNo << " Exiting" << endl;
}

/**
 * This method sets up the main UDP socket server. It loops continuously, parsing the input
 * received from the UDP socket and launching the correct ROS services on the machine it's running on.
 * The machine running this main server must therefore have all necessary ROS packages installed.
 * @param remoteIP The IP supplied for a client socket
 * @param remotePort The port number supplied for a client socket
 * @param threadNo The ID of the thread this method is starting
 */
void GaTACDroneControl::runServer(int sock_in, struct sockaddr_storage * client_addr_in, socklen_t addr_len_in, int threadNo_in, const int myDroneId_in) {
	const char *publishCommand = "rostopic pub -1 /drone%s/ardrone/%s std_msgs/Empty&";
	const char *serviceCall = "rosservice call /drone%d/%s";
	Locker lock(&dronePositionMtx);
	lock.unlock();

    int sock = sock_in;
    struct sockaddr_storage client_addr;
    memcpy(&client_addr, client_addr_in, addr_len_in);
    socklen_t addr_len = addr_len_in;
    int threadNo = threadNo_in;
    const int myDroneId = myDroneId_in;

	signal(SIGINT, handlesigint);

	// Loop forever. Read commands from socket and perform the action specified.
	int bytesReceived = 0;
	do {
		const char *droneNumber, *x, *y;
		string temp;
		string errorMessage = "none";
		string invalidDroneId = "No drone with ID has been spawned. Please specify a valid drone ID.";
       		string invalidLocation = "Location entered is beyond the grid parameters.";
		char receiveBuffer[BUFLEN];
		char publishMessage[BUFLEN * 5];
		int initialColumn, initialRow;
		int droneInt;
		int sleepCtr; // to ensure drones don't send commands before server can process, and drones begin in sync
		bool allReady = false;
		int xInt, yInt, droneNumberInt = 0;
		char * senseOption;
		int senseInt, maxdist, ignored, nor, noc;
		string droneRole;
		vector<pair<string, int>> senseResult;
		const char *navDataToSend = ""; // Holds string of navdata server will send to client on request
		std::stringstream strX;
		std::stringstream strY;
		std::stringstream strID;

		cout << "Waiting for a command..." << endl;
		if ((bytesReceived = recvfrom(sock, receiveBuffer, BUFLEN - 1, 0, (struct sockaddr *) &client_addr, &addr_len)) == -1) {
            if(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                continue;
            } else {
                perror("Error receiving command.");
                exit(1);
            }
		}
		receiveBuffer[bytesReceived] = '\0';

		cout << "Command Received!----> ";

		// Splitting command input into tokens by whitespace
		string buffer;
		string stringCommand(receiveBuffer);
		stringstream ss(stringCommand);

		std::istream_iterator<std::string> begin(ss);
		std::istream_iterator<std::string> end;

		// Storing tokens in vector
		vector<string> tokens(begin, end);

		char rawCommand = receiveBuffer[0];
		switch (rawCommand) {
		case 's':

			cout << "Spawn drone." << endl;
			droneRole = tokens.at(1);
			initialColumn = atoi((tokens.at(2)).c_str());
			initialRow = atoi((tokens.at(3)).c_str());
			// If grid size hasn't been set
			if (gridSizeCheck() == false) {
                errorMessage = "No grid size set. You must specify a grid size before spawning a drone.";
			}
			else if (maxDrones()) {
                errorMessage = "The requested number of drones has already been spawned";
			}
			// If start position isn't valid
			else if (validLocation(initialColumn, initialRow) == false) {
                errorMessage = "The starting location you specified does not lie within the grid. Please choose a valid starting location.";
			}

			if (errorMessage != "none") {
				printf("Error: couldn't spawn drone. %s\n", errorMessage.c_str());
				exit(1);
			}
			else{
			/* If server passes all checks, client message processed */
                lock.lock();
                cout << droneRoles.size() << " " << myDroneId << endl;
                droneRoles.at(myDroneId) = droneRole;
                dronePositions.at(myDroneId) = make_pair(initialColumn, initialRow);
                lock.unlock();
                printf("Ready to spawn drone at [%d, %d].\n", initialColumn, initialRow);
                numberOfDrones++;
			}
			break;
        case 'X':
            //scenario is over
            if (isScenarioOver())
            {
                errorMessage = "Scenario has already ended";
            } else
            {
                cout << "Ending Scenario" << endl;
                string end_msg = stringCommand.substr(2);
                setScenarioIsOver(end_msg);
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
			else if (dronesReady.at(myDroneId)) {
                printf("Error: Drone ID %s is already flying.\n", droneNumber);
			}
			else{
			/* If server passes all checks, client message processed */
			sprintf(publishMessage, serviceCall, droneInt, "takeoff_thinc_smart");
			ignored = system(publishMessage);

			dronesReady.at(myDroneId) = true;

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
			else if (!dronesReady.at(myDroneId)) {
                printf("Error: Drone ID %s is not flying.\n", droneNumber);
			}
			else{
			/* If server passes all checks, client message processed */
			sprintf(publishMessage, serviceCall, droneInt, "land_at_home");
			ignored = system(publishMessage);

			dronesReady.at(myDroneId) = false;
            }
			break;

		case 'h':
			cout << "Hover." << endl;
			droneNumber = (tokens.at(1)).c_str();
			strID << droneNumber;
			strID >> droneNumberInt;
		/* Now the server checks if the drone ID and location entered are valid */
			// If grid hasn't been started
			if (gridStartCheck() == false) {
                errorMessage = "The grid has not yet been started. Grid must be started before sending commands to a drone.";
			}
			// If drone ID isn't valid
			else if (validDroneId(droneNumberInt) == false) {
                errorMessage = invalidDroneId;
			}
		/* If server passes checks, client message processed */
			else{
                lock.lock();
                int x = dronePositions.at(droneNumberInt).first;
                int y = dronePositions.at(droneNumberInt).second;
                lock.unlock();
                moveAndCheck(x, y, droneNumberInt);
			}
		//end moveAndCheck
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
			sprintf(publishMessage, publishCommand, droneInt, "reset");
			ignored = system(publishMessage);
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
			else if (validDroneId(droneNumberInt) == false) {
			errorMessage = invalidDroneId;
			}
			// If destination isn't valid
			else if (validLocation(xInt, yInt) == false) {
				errorMessage = invalidLocation;
			}
			if (errorMessage != "none") {
			printf("Error: couldn't move drone. %s\n", errorMessage.c_str());
			}
		/* If server passes all checks, client message processed */
			else{
			moveAndCheck(xInt, yInt, droneNumberInt);
			}
		//end moveAndCheck
			break;

		case 'g':
			cout << "Set grid size." << endl;
			noc = atoi(tokens.at(1).c_str());
			nor = atoi(tokens.at(2).c_str());
			// If size has already been set
			if (gridSizeCheck()) {
                cout << "Error: The grid size has already been set to ["<<numberOfColumns<<" x "<<numberOfRows<<"]. Specify grid size only once." << endl;
			} else {
                // If size isn't valid
                if (validGridSize(noc, noc) == false) {
                    cout << "Error: The grid size specified was too large. The maximum grid size is 10x10." << endl;
                    exit(1);
                }
                /* If server passes all checks, client message processed */
                else{
                    gridSizeSet = true;
                    this->numberOfColumns = noc;
                    this->numberOfRows = nor;
                }
			}
			break;

			//When running multi clients, have each use readyUp() to start the grid
		case 'y':

			allReady = true;

            lock.lock();
            clientsReady.at(myDroneId) = true;

			for(int i=0; i < clientsReady.size(); i++)
			{
                if(clientsReady.at(i) == false){
                    allReady = false;
                    break;
                }
			}
			lock.unlock();

			if(allReady == true){
                readyForData = true;
                // If grid size has been set
                if (this->gridSizeCheck() == true && this->gridStartCheck() == false) {
                    cout << "All clients ready. Starting grid!" << endl;
                    cout << "Waiting a few seconds before server will receive commands..." << endl;
                    launchGrid();
                    gridStarted = true;

                }
                // If grid has already been started
                else if(this->gridStartCheck() == true) {
                    cout << "Error: Grid already started." << endl;
                }
                //If grid size has not been set
                else if(this->gridSizeCheck() == false) {
                    cout << "Error: No grid size set. You must specify a grid size before starting the grid." << endl;
                    exit(1);
                }
			}
			break;

			//Kind of deprecated, usable only for single client start
		case 'i':
			/*cout << "Start gazebo." << endl;
			// If grid size has been set
			if (this->gridSizeCheck() == true && this->gridStartCheck() == false) {
			launchGrid();
			gridStarted = true;
			}
			// If grid has already been started
			else if(this->gridStartCheck() == true) {
			cout << "Error: Grid already started." << endl;
			}
			//If grid size has not been set
			else if(this->gridSizeCheck() == false) {
			cout << "Error: No grid size set. You must specify a grid size before starting the grid." << endl;
			exit(1);
			}*/
			break;

		case 'u':
			cout << "Sense North" << endl;
			droneNumber = (tokens.at(1)).c_str();
			maxdist = atoi((tokens.at(2)).c_str());
			droneInt = atoi(droneNumber);
			senseInt = 0;
			senseResult = sense(droneInt, senseInt, maxdist);
  			break;

		case 'd':
			cout << "Sense South" << endl;
			droneNumber = (tokens.at(1)).c_str();
			maxdist = atoi((tokens.at(2)).c_str());
			droneInt = atoi(droneNumber);
			senseInt = 1;
			senseResult = sense(droneInt, senseInt, maxdist);
  			break;

		case 'k':
			cout << "Sense East" << endl;
			droneNumber = (tokens.at(1)).c_str();
			maxdist = atoi((tokens.at(2)).c_str());
			droneInt = atoi(droneNumber);
			senseInt = 2;
			senseResult = sense(droneInt, senseInt, maxdist);
  			break;

		case 'j':
			cout << "Sense West" << endl;
			droneNumber = (tokens.at(1)).c_str();
			maxdist = atoi((tokens.at(2)).c_str());
			droneInt = atoi(droneNumber);
			senseInt = 3;
			senseResult = sense(droneInt, senseInt, maxdist);
  			break;

			//Default case, for command characters that are undefined
		default:
			cout << "Error parsing raw command - invalid command character received." << endl;
			break;
		}
		// Sending acknowledgment message to client
		int len = BUFLEN;
		char sendBuffer[len];
		char navBuffer[BUFLEN];
		strcpy(sendBuffer, receiveBuffer);
		if (isScenarioOver()){
			sprintf(sendBuffer, "X %s", scenarioOverMsg.c_str());
            int numSent = 0;
            if ((numSent = sendto(sock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
                perror("Server: error sending acknowledgment.");
                exit(1);
            }
		}
		//If command received was to spawn a drone, first char of ACK set to id
		else if(rawCommand == 's'){
			sprintf(sendBuffer, "%d %s", myDroneId, receiveBuffer);
            int numSent = 0;
            if ((numSent = sendto(sock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
                perror("Server: error sending acknowledgment.");
                exit(1);
            }
		}
		//If command received was ready up command, server waits for other drones or starts grid
		else if(rawCommand == 'y'){
            cout<<"Waiting for other drones..." <<endl;

            while (! this->gridStartCheck() ) {
                usleep(10000);
            }

			sprintf(publishMessage, serviceCall, myDroneId, "takeoff_thinc_smart");
			ignored = system(publishMessage);
			dronesReady.at(myDroneId) = true;

            while (! this->droneStartCheck() ) {
                usleep(10000);
            }

            int numSent = 0;
            if ((numSent = sendto(sock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
                perror("Server: error sending acknowledgment.");
                exit(1);
            }
            cout<<"All clients ready, sending reply to " << myDroneId << endl;

		}
		//If command received was a sense command, first char of ACK set to sense return integer
		else if(rawCommand == 'u' || rawCommand == 'j' || rawCommand == 'k' || rawCommand == 'd'){
            string tempBuffer = "";
            tempBuffer.append(std::to_string(senseResult.size()));
            tempBuffer.append(" ");
            for(int i = 0; i < senseResult.size(); i++)
            {
                tempBuffer.append(senseResult.at(i).first);
                tempBuffer.append(" ");
                tempBuffer.append(std::to_string(senseResult.at(i).second));
                tempBuffer.append(" ");
            }
            strcpy(sendBuffer, tempBuffer.c_str());
            int numSent = 0;
			if ((numSent = sendto(sock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
                perror("Server: error sending acknowledgment.");
                exit(1);
			}
		}
		else{
            int numSent = 0;
            if ((numSent = sendto(sock, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *) &client_addr, addr_len)) == -1) {
                perror("Server: error sending acknowledgment.");
                exit(1);
            }
		}
	} while (! isScenarioOver());
	// Cleaning up socket information
	close(sock);

    cout << "Control thread " << threadNo << " Exiting" << endl;
}


bool GaTACDroneControl::droneStartCheck() {
    for(int i=0; i < dronesReady.size(); i++)
    {
        if(dronesReady.at(i) == false){
            return false;
        }
    }

    return true;
}


/**
 * This method launches the Gazebo simulator with a grid of whatever size was specified via the setGridSize method,
 * and with any drones that have been set up via the setUpDrone method.
 *
 * *** NOTE: When using real drones, this method instead creates/initializes the ROS nodes for each drone. ***
 */
void GaTACDroneControl::launchGrid() {
	cout << "Launching Grid "<<endl;
	/* simulatorMode == true */
	if(simulatorMode == true){
		const char *gazeboMessage = "roslaunch /tmp/grid_flight.launch";
		const char *thincSmartCommand = "ROS_NAMESPACE=drone%d rosrun ardrone_thinc thinc_smart %d %d %d %d %f %f %f s";
		char thincSmartMessage[strlen(thincSmartCommand) + BUFLEN];

		// Configure launch file and start gazebo
		configureLaunchFile();
		runSubProcess(gazeboMessage);

		// Wait for gazebo to finish loading. This takes a while.
		sleep(20);

        // Starting a thinc_smart ROS node for each drone
		int droneID;
		Locker lock(&dronePositionMtx);
		for (int i = 0; i < numberOfDrones; i++) {
			droneID = i;
			sprintf(thincSmartMessage, thincSmartCommand, droneID, numberOfColumns, numberOfRows, dronePositions.at(droneID).first, dronePositions.at(droneID).second, 2.0, 2.0, (droneID + 1.0) * 0.4);
			cout << "publishing message: " << thincSmartMessage << endl;
			runSubProcess(thincSmartMessage);
		}
		lock.unlock();
		sleep(5);
	}
	/* simulatorMode == false */
	if(simulatorMode == false){

		const char *coreMessage = "xterm -e roscore&";
		const char *launchMessage = "roslaunch /tmp/tagLaunch.launch";
		const char *thincSmartCommand = "ROS_NAMESPACE=drone%d rosrun ardrone_thinc thinc_smart %d %d %d %d %f %f %f r";
		const char *ardroneDriverCommand = "ROS_NAMESPACE=drone%d rosrun ardrone_autonomy ardrone_driver %s&";
		const char *flattenTrim = "ROS_NAMESPACE=drone%d xterm -e rosservice call --wait /drone%d/ardrone/flattrim&";
		const char *toggleCam = "ROS_NAMESPACE=drone%d xterm -e rosservice call /drone%d/ardrone/togglecam&";
		char thincSmartMessage[strlen(thincSmartCommand) + BUFLEN];
		char ardroneDriverMessage[256];
		char flatTrimMessage[256];
		char toggleCamMessage[256];

		// Configure launch file and start core
//		configureLaunchFile();
//		system(coreMessage);
//		runSubProcess(launchMessage);

//		sleep(10);
		Locker lock(&dronePositionMtx);

		// Starting a thinc_smart ROS node for each drone && an ardrone_autonomy ROS node for each drone
		int droneID;
		for (int i = 0; i < numberOfDrones; i++) {
			droneID = i;
			sprintf(thincSmartMessage, thincSmartCommand, droneID, numberOfColumns, numberOfRows, dronePositions.at(droneID).first, dronePositions.at(droneID).second, 1.0, 1.0, droneID + 0.5);
			cout << "publishing message: " << thincSmartMessage << endl;
			runSubProcess(thincSmartMessage);
		}
		lock.unlock();
		sleep(2);

	}
}

/**
 * In simulation, this method modifies the grid_flight.launch file used by Gazebo to start the simulator.
 * It specifies the grid size and number/starting position of all drones.
 *
 * With actual drones, this method uses a modification of ardrone_autonomy's ardrone.launch file, to allow tag spotting and specify other parameters.
 *
 * *** NOTE: Directory in which to create launch file can be specified here. ***
 */
void GaTACDroneControl::configureLaunchFile() {
	cout << "Configuring launch file." << endl;
	/* simulatorMode == true */
	if(simulatorMode == true){
		// The following strings match the formatting of the grid_flight.launch file found in the thinc_sim_gazebo ros package (in the launch folder).
		const char *genTextureText =
			"<?xml version=\"1.0\"?>\n\n"
			"<launch>\n\t"
			"<node\n\t\t"
			"name=\"gen_texture\" pkg=\"cvg_sim_gazebo\" type=\"gen_texture\"\n\t\t"
			"args=\"%d %d 1 1 $(find cvg_sim_gazebo)/meshes/grid.png $(find cvg_sim_gazebo)/worlds/grid.world\"\n\t"
			"/>\n\t";
		const char *genDaeText =
			"<node\n\t\t"
			"name=\"gen_dae\" pkg=\"cvg_sim_gazebo\" type=\"gen_dae.py\"\n\t\t"
			"args=\"%d %d 2 2 $(find cvg_sim_gazebo)/meshes/grid.dae\"\n\t"
			"/>\n\n\t"
			"<!-- Start Gazebo with wg world running in (max) realtime -->\n\t"
			"<include file=\"$(find cvg_sim_gazebo)/launch/grid.launch\"/>\n\n\t"
			"<!-- Spawn simulated quadrotor uav -->\n";
		const char *droneText =
			"\t<group ns=\"drone%d\">\n\t\t"
			"<param name=\"tf_prefix\" value=\"drone%d\"/>\n\t\t"
			"<include file=\"$(find cvg_sim_gazebo)/launch/spawn_quadrotor.launch\" >\n\t\t\t"
			"<arg name=\"robot_namespace\" value=\"drone%d\"/>\n\t\t\t"
			"<arg name=\"x\" value=\"%.2f\"/>\n\t\t\t"
			"<arg name=\"y\" value=\"%.2f\"/>\n\t\t\t"
			"<arg name=\"z\" value=\"0.70\"/>\n\t\t"
			"</include>\n\t"
			"</group>\n\n";
		const char *endingText = "</launch>";

		// Open file stream
		ofstream fileStream("/tmp/grid_flight.launch", ios::trunc);

		// Write gen_texture and gen_dae text to file
		char textureBuffer[strlen(genTextureText) + 256];
		char daeBuffer[strlen(genDaeText) + 256];
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
		char droneBuffer[strlen(droneText) + 256];
		if (fileStream.is_open()) {
			Locker lock(&dronePositionMtx);

			for (int i = 0; i < numberOfDrones; i++) {
				droneID = i;
				droneX = originX + (2 * dronePositions.at(droneID).second);
				droneY = originY - (2 * dronePositions.at(droneID).first);
				sprintf(droneBuffer, droneText, droneID, droneID, droneID, droneID, droneX, droneY);
				fileStream << droneBuffer;
			}
			lock.unlock();
			fileStream << endingText;
		}

		// Close file stream
		fileStream.close();
	}

	/* simulatorMode == false */
	if(simulatorMode == false){
		// The following strings match the formatting of the ardrone.launch file found in the ardrone_autonomy ros package (in the launch folder).
		const char *startingText =
			"<?xml version=\"1.0\"?>\n\n"
			"<launch>\n";
		const char *droneText =
			"\t<group ns=\"drone%d\">\n\t\t"
			"<param name=\"tf_prefix\" value=\"drone%d\"/>\n\t\t"
			"<include file=\"$(find ardrone_autonomy)/launch/vanilla.launch\" >\n\t\t\t"
			"<arg name=\"drone_ip\" value=\"192.168.1.1%d\"/>\n\t\t"
			"<arg name=\"drone_frame_id\" value=\"drone%d_base\"/>\n\t\t"
			"</include>\n\t"
			"<include file=\"$(find tum_ardrone)/launch/tum_ardrone.launch\">\n"
			"<arg name=\"drone_ip\" value=\"192.168.1.1%d\"/>"
			"</include>"
			"</group>\n\n";

		const char *endingText = "</launch>";

		// Open file stream
		ofstream fileStream;
		fileStream.open("/tmp/tagLaunch.launch", ios::out);

		// Write gen_texture and gen_dae text to file
		char startingBuffer[strlen(startingText) + 1];
		strcpy(startingBuffer, startingText);
		if (fileStream.is_open()) {
			fileStream << startingBuffer;
		}

		// Find coordinates of (0,0) in Gazebo terms
		int originX, originY;
		getGazeboOrigin(originX, originY);

		// Write all drone sub-launch text to file
		int droneID;
		float droneX, droneY;
		char droneBuffer1[strlen(droneText) + 256];
		if (fileStream.is_open()) {
			Locker lock(&dronePositionMtx);
			for (int i = 0; i < numberOfDrones; i++) {
				droneID = i;
				droneX = originX + (2 * dronePositions.at(droneID).second);
				droneY = originY - (2 * dronePositions.at(droneID).first);
            			sprintf(droneBuffer1, droneText, droneID, droneID, droneID, droneID, droneID);
            			fileStream << droneBuffer1;
			}
			lock.unlock();
			fileStream << endingText;

		}

		// Close file stream
		fileStream.close();
	}
}

/**
 * Gazebo places the grid at different locations within its own coordinate system depending on the size of the grid.
 * The user will specify a grid size (A x B), and this method will find the Gazebo coordinates of (0, 0) on the user's grid.
 * @param x User grid's X-axis origin
 * @param y User grid's Y-axis origin
 */
void GaTACDroneControl::getGazeboOrigin(int& x, int& y) {
	x = (-1) * (numberOfRows - 1);
	y = numberOfColumns - 1;
}


/**
 * This method takes movement parameters and sends waypoint messages one cell at a time.
 * As movements are made, it updates the position of moving drone and checks for shared cells
 * using sharedSpace() method. On completion of desired movement, drone's final destination is printed on the server terminal.
 * @param x Desired X-axis destination
 * @param y Desired Y-axis destination
 * @param Id ID of the drone being moved
 */
void GaTACDroneControl::moveAndCheck(int x, int y, int Id)
{
	const char *moveCommand = "rosservice call /drone%d/waypoint -- %d %d -1"; //id...  x y z id
	char publishMessage[strlen(moveCommand) + BUFLEN];
	int droneId = Id;
	Locker lock(&dronePositionMtx);
	int dx = dronePositions.at(droneId).first - x;
	int dy = dronePositions.at(droneId).second - y;
	//Using same logic as ardrone_thinc.cpp file, send messages for movement one cell at a time
	/* simulatorMode == true and false */
	if(!(dx == 0 && dy == 0)){
        do
        {
            lock.lock();
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
            lock.unlock();
            sprintf(publishMessage, moveCommand, droneId, xSend, ySend);
            int ignored = system(publishMessage);
            vector<bool> dronesSharingSpace;

            if(sharedSpace(&dronesSharingSpace) == true){
                lock.lock();
                cout<<"Drones sharing a cell: " << endl;
                for(int i = 0; i < dronesSharingSpace.size(); i++)
                {
                    if(dronesSharingSpace.at(i) == true)
                    cout<<"==> drone "<<i<<" @ ("<<dronePositions.at(i).first<<", "<<dronePositions.at(i).second<<")"<<endl;
                }
                lock.unlock();
            }
        }while ((dx != 0) || (dy != 0));
        cout << this->getGridPosition(droneId) << endl;
	}
	else if(dx == 0 && dy == 0)
	{
        sprintf(publishMessage, moveCommand, droneId, x, y);
        int ignored = system(publishMessage);
        cout << "Drone " << droneId << "hovering." << endl;
	}
}

/**
 * This method compares current locations of drones and returns true if a cell is being shared.
 * Additionally, it updates a vector of which drones are sharing a cell.
 * If a shared cell is detected, the coordinates and drone ID's are printed to server terminal.
 * @return Boolean indicating whether a cell on the grid is occupied by two or more drones; true indicates a shared cell is detected
 */
bool GaTACDroneControl::sharedSpace(vector<bool> * dronesSharingSpace)
{
	Locker lock(&dronePositionMtx);
	/* simulatorMode == true and false */
	bool sharing = false;
	while (dronesSharingSpace->size() < dronePositions.size()) {
		dronesSharingSpace->push_back(false);
	}
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
				dronesSharingSpace->at(i) = true;
				dronesSharingSpace->at(k) = true;
			}
			}
		}
	}
	return sharing;
}

/**
 * This method returns true if the maximum number of drones has already been spawned.
 * @return Boolean indicating whether 3 drones are already on the grid, true if this is the case
 */
bool GaTACDroneControl::maxDrones()
{
	if(numberOfDrones >= expectedDrones)
	return true;
	else
	return false;
}

/**
 * This method returns true if a drone id sent by the client is a valid drone id that has previously been spawned.
 * @param id Drone ID to be verified
 * @return Boolean indicating whether the given drone ID is currently on the grid, true if the ID is in use and drone is present
 */
bool GaTACDroneControl::validDroneId(int id)
{
	if(id < 0 || id > (numberOfDrones))
	return false;
	else
	return true;
}

/**
 * This method returns the value of boolean gridSizeSet, which lets the server check if the grid size has been set.
 * @return Boolean indicating whether the grid size has been set, true if the size has already been set
 */
bool GaTACDroneControl::gridSizeCheck()
{
	return gridSizeSet;
}

/**
 * This method returns the value of boolean gridStarted, which lets the server know if the grid has been started and the drones' ROS nodes have been initialized
 * @return Boolean indicating whether the grid has started and ROS nodes have been initialized, true if the size has already been set
 */
bool GaTACDroneControl::gridStartCheck()
{
	return gridStarted;
}

/**
 * This method returns true if a location sent by the client is within the bounds of the grid.
 * @param x X-axis value to be verified
 * @param y Y-axis value to be verified
 * @return Boolean indicating the given location is on the grid, true if the location is valid
 */
bool GaTACDroneControl::validLocation(int x, int y)
{
	if(x < 0 || y < 0 || x >= numberOfColumns || y >= numberOfRows)
	return false;
	else
	return true;
}

/**
 * This method returns true if a client-set grid is between 0x0 and 10x10.
 * @param x X-axis dimension to be verified
 * @param y Y-axis dimension to be verified
 * @return Boolean indicating whether the specified grid dimensions are valid, true if valid
 */
bool GaTACDroneControl::validGridSize(int x, int y)
{
	if(x < 1 || y < 1 || x > 10 || y > 10)
	return false;
	else
	return true;
}

/**
 * This method will return and print the current position of a given drone on the grid.
 * @param droneId ID of drone to return navdata from
 * @return Human-readable string denoting the drone's current location on the grid
 */
string GaTACDroneControl::getGridPosition(int droneId)
{
	std::stringstream strID;
	std::stringstream strX;
	std::stringstream strY;
	Locker lock(&dronePositionMtx);
	int xPos = dronePositions.at(droneId).first;
	int yPos = dronePositions.at(droneId).second;
	lock.unlock();
	strID << droneId;
	strX << xPos;
	strY << yPos;
	string id_str = strID.str();
	string x_str = strX.str();
	string y_str = strY.str();
 	string toReturn = "Drone " + id_str + " position: (" + x_str + ", " + y_str + ")";
	return toReturn;
}

/**
 * This method will call the PrintNavdata service to set the drone's data members to the correct values and return the requested data to the client.
 * @param droneId ID of drone to return navdata from
 * @return Character array of all navdata values, to be sent over a socket to the client, broken up into strings of thirty characters, and used to set navdata members
 */
const char* GaTACDroneControl::getData(int droneId)
{
	const char *printNavdataCommand = "rosservice call /drone%d/printnavdata"; //id...option id
	char printNavMessage[strlen(printNavdataCommand) + BUFLEN];
	sprintf(printNavMessage, printNavdataCommand, droneId);
    FILE *lsofFile_p = popen(printNavMessage, "r");

    if (!lsofFile_p)
    {
    return "";
    }

	string line1 = "";
	string line2 = "";
	string line3 = "";
	string line4 = "";
	string line5 = "";
	string line6 = "";

    string fullbuffer = "";

    char buffer[1024];
    while (fgets(buffer, sizeof(buffer), lsofFile_p) != NULL)
    {
        fullbuffer.append(buffer);
    }


    stringstream stream (fullbuffer);
	string allDataString = "";

	//battery
    getline(stream, line1);
    if(line1.size() < 30){
        int whiteSpace = 30 - line1.size();
        for(int w = 0 ; w < whiteSpace; w++)
         {
            line1 += " ";
         }
    }
    line1 = line1.substr (5);
    //forward velocity
    getline(stream, line2);
    if(line2.size() < 30){
        int whiteSpace = 30 - line2.size();
        for(int w = 0 ; w < whiteSpace; w++)
         {
            line2 += " ";
         }
    }
    //sideways velocity
    getline(stream, line3);
    if(line3.size() < 30){
        int whiteSpace = 30 - line3.size();
        for(int w = 0 ; w < whiteSpace; w++)
         {
            line3 += " ";
         }
    }
    //vert velocity
    getline(stream, line4);
    if(line4.size() < 30){
        int whiteSpace = 30 - line4.size();
        for(int w = 0 ; w < whiteSpace; w++)
         {
            line4 += " ";
         }
    }
    //sonar
    getline(stream, line5);
    if(line5.size() < 30){
        int whiteSpace = 30 - line5.size();
        for(int w = 0 ; w < whiteSpace; w++)
         {
            line5 += " ";
         }
    }
    //tags spotted
    getline(stream, line6);
    if(line6.size() < 30){
        int whiteSpace = 30 - line6.size();
        for(int w = 0 ; w < whiteSpace; w++)
         {
            line6 += " ";
         }
    }
    allDataString = line1 + line2 + line3 + line4 + line5 + line6;

    pclose(lsofFile_p);

	return allDataString.c_str();
}

/**
 * This method allows a client to query the server whether another drone is north, south, east, or west of the client's drone on the grid.
 * @param droneId The drone ID of the client sending sense request.
 */
vector<pair<string, int>> GaTACDroneControl::sense(int droneId, int option, int maxDist)
{
	Locker lock(&dronePositionMtx);

	// instead of looking through squares, go through each drone's position and classify it


	int xCurrent = dronePositions.at(droneId).first;
	int yCurrent = dronePositions.at(droneId).second;

    int maxdist = (int)sqrt(maxDist * maxDist);

	vector<pair<string, int>> returnval;

    //cycles through current drone positions and tests them against querying client position
    for(int k = 0; k <dronePositions.size(); k++)
    {
        int dist = ceil(sqrt((dronePositions.at(k).first-xCurrent)*(dronePositions.at(k).first-xCurrent)
                              + (dronePositions.at(k).second-yCurrent)*(dronePositions.at(k).second-yCurrent)));

        if(droneId != k && dist <= maxdist){

            int whichclass = -1;

            //case: another drone is not North of subject drone
            if (dronePositions.at(k).second > yCurrent) {
                whichclass = 0;
            } else if (dronePositions.at(k).second < yCurrent) {
                whichclass = 1;
            } else if(dronePositions.at(k).first >= xCurrent) {
                whichclass = 2;
            } else if(dronePositions.at(k).first <= xCurrent) {
                whichclass = 3;
            }

            if (whichclass == option) {
                returnval.push_back(make_pair(droneRoles.at(k), dist));
            }

        }
    }

    return returnval;
}


void GaTACDroneControl::setScenarioIsOver(string msg)
{
    Locker scenarioOverLock(&scenarioOverMtx);

    scenarioOver = true;
    scenarioOverMsg = msg;

    scenarioOverLock.unlock();
}

bool GaTACDroneControl::isScenarioOver()
{
    bool returnval;

    Locker scenarioOverLock(&scenarioOverMtx);

    returnval = scenarioOver;

    scenarioOverLock.unlock();

    return returnval;
}

void GaTACDroneControl::runSubProcess(const char * command) {

    pid_t pid = system2(command);

    if (pid > 0) {
        subProcesses.push_back(pid);
    } else {
        perror("Error creating child process. ");
        exit(1);
    }
}
