
all: simServerThree simServer tagServer tagClientA simThreeA simThreeB simThreeC

# servers

simServerThree: GaTACDroneControl.cpp serverTestThreeDrones.cpp  GaTACDroneControl.hpp
	g++ GaTACDroneControl.cpp serverTestThreeDrones.cpp -lboost_thread -o simServerThree


simServer: GaTACDroneControl.hpp GaTACDroneControl.cpp serverTest.cpp
	g++ GaTACDroneControl.cpp serverTest.cpp -lboost_thread -o simServer

tagServer:  GaTACDroneControl.hpp GaTACDroneControl.cpp  realServerTest.cpp
	g++ GaTACDroneControl.cpp realServerTest.cpp -lboost_thread -o tagServer



# clients

tagClientA: GaTACDroneControl.hpp GaTACDroneControl.cpp tagTestClient.cpp
	g++ GaTACDroneControl.cpp tagTestClient.cpp -lboost_thread -o tagClientA

simThreeA: GaTACDroneControl.hpp GaTACDroneControl.cpp SimThreeDronesClientA.cpp
	g++ GaTACDroneControl.cpp SimThreeDronesClientA.cpp -lboost_thread -o simThreeA

simThreeB: GaTACDroneControl.hpp GaTACDroneControl.cpp SimThreeDronesClientB.cpp
	g++ GaTACDroneControl.cpp SimThreeDronesClientB.cpp -lboost_thread -o simThreeB

simThreeC: GaTACDroneControl.hpp GaTACDroneControl.cpp SimThreeDronesClientC.cpp
	g++ GaTACDroneControl.cpp SimThreeDronesClientC.cpp -lboost_thread -o simThreeC
