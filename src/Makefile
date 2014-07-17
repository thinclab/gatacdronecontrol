
all: simServerThree simServer tagServer tagClientA tagClientB simThreeA simThreeB simThreeC SWARMSERVER SWARMCLIENT

# servers

simServerThree: GaTACDroneControl.cpp serverTestThreeDrones.cpp  GaTACDroneControl.hpp
	g++ GaTACDroneControl.cpp serverTestThreeDrones.cpp -lboost_thread -o simServerThree


SWARMSERVER: GaTACDroneControl.cpp SWARMSERVER.cpp  GaTACDroneControl.hpp
	g++ GaTACDroneControl.cpp SWARMSERVER.cpp -lboost_thread -o SWARMSERVER


simServer: GaTACDroneControl.hpp GaTACDroneControl.cpp serverTest.cpp
	g++ GaTACDroneControl.cpp serverTest.cpp -lboost_thread -o simServer

tagServer:  GaTACDroneControl.hpp GaTACDroneControl.cpp  realServerTest.cpp
	g++ GaTACDroneControl.cpp realServerTest.cpp -lboost_thread -o tagServer



# clients

SWARMCLIENT: GaTACDroneControl.cpp SWARMCLIENT.cpp  GaTACDroneControl.hpp
	g++ GaTACDroneControl.cpp SWARMCLIENT.cpp -lboost_thread -o SWARMCLIENT

tagClientA: GaTACDroneControl.hpp GaTACDroneControl.cpp tagTestClient.cpp
	g++ GaTACDroneControl.cpp tagTestClient.cpp -lboost_thread -o tagClientA

tagClientB: GaTACDroneControl.hpp GaTACDroneControl.cpp tagTestClient2.cpp
	g++ GaTACDroneControl.cpp tagTestClient.cpp -lboost_thread -o tagClientB

simThreeA: GaTACDroneControl.hpp GaTACDroneControl.cpp SimThreeDronesClientA.cpp
	g++ GaTACDroneControl.cpp SimThreeDronesClientA.cpp -lboost_thread -o simThreeA

simThreeB: GaTACDroneControl.hpp GaTACDroneControl.cpp SimThreeDronesClientB.cpp
	g++ GaTACDroneControl.cpp SimThreeDronesClientB.cpp -lboost_thread -o simThreeB

simThreeC: GaTACDroneControl.hpp GaTACDroneControl.cpp SimThreeDronesClientC.cpp
	g++ GaTACDroneControl.cpp SimThreeDronesClientC.cpp -lboost_thread -o simThreeC
