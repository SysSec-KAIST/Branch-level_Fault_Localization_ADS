#!/bin/bash

start_line=${1:-1}
end_line=${2:--1}

chromosome_file="./Data/solution_failures.json"

test_map="SanFrancisco Correct"
test_vehicle="Lincoln2017MKZ LGSVL"
modules="['Localization', 'Perception', 'Transform', 'Routing', 'Prediction', 'Planning', 'Traffic Light', 'Control', 'Recorder']"

export PYTHONPATH="$PYTHONPATH:.."

current_line=1

if [ $end_line -eq -1 ]; then
   echo "Starting from line $start_line until the end"
else
   echo "Running from line $start_line to line $end_line"
fi

while IFS= read -r line; do
   if [ $current_line -ge $start_line ] && { [ $end_line -eq -1 ] || [ $current_line -le $end_line ]; }; then
       chromosome=$(echo "$line" | jq -r '.chromosome' | tr -d '\n' | sed 's/\s\+/ /g' | sed 's/^ *//;s/ *$//')

       scenario=$(echo "$line" | jq -r '.stop_reason.scenario | "[\"\(.start)\", \"\(.end)\"]"')
       
       echo "# Processing line: $line"
       echo "# Processing case $current_line"
       echo "# Running chromosome: \"$chromosome\""
       echo "# Scenario: $scenario"
       

       python3 input_replay.py \
        --map="$test_map" \
        --vehicle="$test_vehicle" \
        --modules="$modules" \
        --chromosome="$chromosome" \
        --scenario="$scenario"

       echo ""
   fi
   ((current_line++))
   
   if [ $end_line -ne -1 ] && [ $current_line -gt $end_line ]; then
       break
   fi
done < <(jq -c '.[]' "$chromosome_file")

echo "All replay commands executed."
