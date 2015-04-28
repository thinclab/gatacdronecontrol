all: bin/SWARMCLIENT bin/SWARMSERVER bin/simServerThree bin/simServer bin/tagServer bin/tagClientA bin/simThreeA bin/simThreeB bin/simThreeC bin/keyboard bin/random bin/server bin/fugitiveClient bin/anticoordClient


lib/GaTACDroneControlServer.a: include/GaTACDroneControlServer.hpp src/GaTACDroneControlServer.cpp
	g++ -O3 -std=c++0x -Iinclude/ -c src/GaTACDroneControlServer.cpp -o obj/GaTACDroneControlServer.o
	ar rvs lib/GaTACDroneControlServer.a obj/GaTACDroneControlServer.o

lib/GaTACDroneControlClient.a: include/GaTACDroneControlClient.hpp src/GaTACDroneControlClient.cpp
	g++ -O3 -std=c++0x -Iinclude/ -c src/GaTACDroneControlClient.cpp -o obj/GaTACDroneControlClient.o
	ar rvs lib/GaTACDroneControlClient.a obj/GaTACDroneControlClient.o

lib/policytree.a: include/policytree.h src/PolicyTree.cpp
	g++ -O3 -std=c++0x -Iinclude/ -c src/PolicyTree.cpp -o obj/policytree.o
	ar rvs lib/policytree.a obj/policytree.o


# servers

bin/SWARMSERVER: src/SWARMSERVER.cpp lib/GaTACDroneControlServer.a
	g++ -o bin/SWARMSERVER -O3 -std=c++0x -Iinclude/  src/SWARMSERVER.cpp lib/GaTACDroneControlServer.a -lboost_system -lboost_thread

bin/simServerThree: src/serverTestThreeDrones.cpp lib/GaTACDroneControlServer.a
	g++ -o bin/simServerThree -O3 -std=c++0x -Iinclude/ src/serverTestThreeDrones.cpp  lib/GaTACDroneControlServer.a -lboost_system -lboost_thread

bin/simServer: src/serverTest.cpp lib/GaTACDroneControlServer.a
	g++ -o bin/simServer -O3 -std=c++0x -Iinclude/ src/serverTest.cpp  lib/GaTACDroneControlServer.a -lboost_system -lboost_thread

bin/tagServer: src/realServerTest.cpp lib/GaTACDroneControlServer.a
	g++ -o bin/tagServer -O3 -std=c++0x -Iinclude/ src/realServerTest.cpp  lib/GaTACDroneControlServer.a -lboost_system -lboost_thread

bin/server: src/server.cpp lib/GaTACDroneControlServer.a
	g++ -o bin/server -O3 -std=c++0x -Iinclude/ src/server.cpp  lib/GaTACDroneControlServer.a -lboost_system -lboost_thread



# clients

bin/SWARMCLIENT: src/SWARMCLIENT.cpp lib/GaTACDroneControlClient.a
	g++ -o bin/SWARMCLIENT -O3 -std=c++0x -Iinclude/  src/SWARMCLIENT.cpp lib/GaTACDroneControlClient.a -lboost_system

bin/tagClientA: src/tagTestClient.cpp lib/GaTACDroneControlClient.a
	g++ -o bin/tagClientA -O3 -std=c++0x -Iinclude/ src/tagTestClient.cpp lib/GaTACDroneControlClient.a -lboost_system

bin/simThreeA: src/SimThreeDronesClientA.cpp lib/GaTACDroneControlClient.a
	g++ -o bin/simThreeA -O3 -std=c++0x -Iinclude/ src/SimThreeDronesClientA.cpp lib/GaTACDroneControlClient.a -lboost_system -lboost_thread

bin/simThreeB: src/SimThreeDronesClientB.cpp lib/GaTACDroneControlClient.a
	g++ -o bin/simThreeB -O3 -std=c++0x -Iinclude/ src/SimThreeDronesClientB.cpp lib/GaTACDroneControlClient.a -lboost_system -lboost_thread

bin/simThreeC: src/SimThreeDronesClientC.cpp lib/GaTACDroneControlClient.a
	g++ -o bin/simThreeC -O3 -std=c++0x -Iinclude/ src/SimThreeDronesClientC.cpp lib/GaTACDroneControlClient.a -lboost_system -lboost_thread

bin/keyboard: src/keyboardClient.cpp lib/GaTACDroneControlClient.a
	g++ -o bin/keyboard -O3 -std=c++0x -Iinclude/ src/keyboardClient.cpp lib/GaTACDroneControlClient.a -lboost_system -lboost_thread

bin/random: src/randomClient.cpp lib/GaTACDroneControlClient.a
	g++ -o bin/random -O3 -std=c++0x -Iinclude/ src/randomClient.cpp lib/GaTACDroneControlClient.a -lboost_system -lboost_thread

bin/fugitiveClient: src/policyClient.cpp lib/GaTACDroneControlClient.a lib/policytree.a
	g++ -o bin/fugitiveClient -O3 -std=c++0x -Iinclude/ src/policyClient.cpp lib/GaTACDroneControlClient.a lib/policytree.a -lboost_system -lboost_thread

bin/anticoordClient: src/anticoordClient.cpp lib/GaTACDroneControlClient.a lib/policytree.a
	g++ -o bin/anticoordClient -O3 -std=c++0x -Iinclude/ src/anticoordClient.cpp lib/GaTACDroneControlClient.a lib/policytree.a -lboost_system -lboost_thread

clean:
	rm lib/*
	rm bin/*
	rm obj/*
