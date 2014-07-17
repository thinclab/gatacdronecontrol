
all: simServerThree simServer tagServer tagClientA tagClientB simThreeA simThreeB simThreeC SWARMSERVER SWARMCLIENT

# servers

simServerThree: GaTACDroneControl.cpp serverTestThreeDrones.cpp  GaTACDroneControl.hpp
	g++ GaTACDroneControl.cpp serverTestThreeDrones.cpp -lboost_thread -std=c++0x -o simServerThree


SWARMSERVER: GaTACDroneControl.cpp SWARMSERVER.cpp  GaTACDroneControl.hpp
	g++ GaTACDroneControl.cpp SWARMSERVER.cpp -lboost_thread -std=c++0x -o SWARMSERVER


simServer: GaTACDroneControl.hpp GaTACDroneControl.cpp serverTest.cpp
	g++ GaTACDroneControl.cpp serverTest.cpp -lboost_thread -std=c++0x -o simServer

tagServer:  GaTACDroneControl.hpp GaTACDroneControl.cpp  realServerTest.cpp
	g++ GaTACDroneControl.cpp realServerTest.cpp -lboost_thread -std=c++0x -o tagServer



# clients

SWARMCLIENT: GaTACDroneControl.cpp SWARMCLIENT.cpp  GaTACDroneControl.hpp
	g++ GaTACDroneControl.cpp SWARMCLIENT.cpp -lboost_thread -std=c++0x -o SWARMCLIENT

tagClientA: GaTACDroneControl.hpp GaTACDroneControl.cpp tagTestClient.cpp
	g++ GaTACDroneControl.cpp tagTestClient.cpp -lboost_thread -std=c++0x -o tagClientA

tagClientB: GaTACDroneControl.hpp GaTACDroneControl.cpp tagTestClient2.cpp
	g++ GaTACDroneControl.cpp tagTestClient.cpp -lboost_thread -std=c++0x -o tagClientB

simThreeA: GaTACDroneControl.hpp GaTACDroneControl.cpp SimThreeDronesClientA.cpp
	g++ GaTACDroneControl.cpp SimThreeDronesClientA.cpp -lboost_thread -std=c++0x -o simThreeA

simThreeB: GaTACDroneControl.hpp GaTACDroneControl.cpp SimThreeDronesClientB.cpp
	g++ GaTACDroneControl.cpp SimThreeDronesClientB.cpp -lboost_thread -std=c++0x -o simThreeB

simThreeC: GaTACDroneControl.hpp GaTACDroneControl.cpp SimThreeDronesClientC.cpp
	g++ GaTACDroneControl.cpp SimThreeDronesClientC.cpp -lboost_thread -std=c++0x -o simThreeC
