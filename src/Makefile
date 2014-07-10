
all: simServerThree simServer tagServer tagClientA

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
