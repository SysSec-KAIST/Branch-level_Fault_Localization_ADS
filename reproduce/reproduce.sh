#!/bin/bash

### MAP CONFIG ###
#test_map='Straight1LaneSame'
#test_map='Borregas Ave'`
#test_map='SingleLaneRoad'
# test_map="SanFrancisco Correct Adjusted"
test_map="SanFrancisco Correct"

#test_vehicle='Lincoln2017MKZ'
test_vehicle='Lincoln2017MKZ LGSVL'
### MODULE CONFIG ###
## NO RECORDER ##
modules="['Localization', 'Perception', 'Transform', 'Routing', 'Prediction', 'Planning', 'Traffic Light', 'Control', 'Recorder']"

export PYTHONPATH="$PYTHONPATH:.."

# SanFrancisco scenario
for ((i=0; i<5; i++))
do
	python3 input_replay.py --map="$test_map" --vehicle="$test_vehicle" --modules="$modules" --chromosome="[-426, 10.2, 414.18, 95, -20, 1.0, -422.2, 403.37, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" --scenario='["LANE_FOLLOW", "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN"]'
done
