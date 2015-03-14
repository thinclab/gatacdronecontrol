#!/bin/bash

for i in {0..12};
do

	./bin/SWARMCLIENT $i &
	sleep 1

done
