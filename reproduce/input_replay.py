from environs import Env
import lgsvl
import argparse
import time
import ast
import copy
import sys
import json
import datetime
import os
from os.path import dirname, realpath
parent_dir = dirname(dirname(realpath(__file__)))
sys.path.append(parent_dir)

from google.protobuf.json_format import ParseDict
from map_utils.map_object_table import LaneTable, JunctionTable
from modules.map.proto.map_pb2 import Map
import math
from enum import Enum
from operator import itemgetter

# PARAMETERS
LGSVL__SIMULATOR_HOST = "127.0.0.1"
LGSVL__SIMULATOR_PORT = 8181
TIME_LIMIT = 150  # seconds
lane_width = 3.7
speed_buffer = 3e-1
distance_to_dest_buffer = 6
lane_speed_limit = 11.176
apollo_modules = []


class Motion(Enum):
    Cut_in_Follow = 1
    Cut_in_Accel = 2
    Cut_in_Decel = 3
    Cut_out_Follow = 4
    Cut_out_Accel = 5
    Cut_out_Decel = 6
    Lane_keep_Follow = 7
    Lane_keep_Accel = 8
    Lane_keep_Decel = 9


class Emergency(Enum):
    NORMAL = 1
    EMER_STOP = 2
    EMER_PULL_OVER = 3
    RESUME = 4

def cal_euc(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def transform_to_forward(tr):
    ax = tr.rotation.x * math.pi / 180.0
    sx, cx = math.sin(ax), math.cos(ax)

    ay = tr.rotation.y * math.pi / 180.0
    sy, cy = math.sin(ay), math.cos(ay)

    return lgsvl.Vector(cx * sy, -sx, cx * cy)

def _is_json_key(json, key):
    try:
        _ = json[key]
    except KeyError:
        return False
    return True

def get_routing_lane_list(routing_response):
    routing_lane_list = []
    for road in routing_response["road"]:
        for passage in road["passage"]:
            for lane in passage["segment"]:
                routing_lane_list.append(lane["id"])
    return routing_lane_list

def get_object_location_list(sim, position):
    # Creating state object to define state for Ego and NPC
    transform = sim.map_point_on_lane(position)
    forward, right, _ = get_unit_vector(sim, transform.position)
    adc_position = transform.position

    ####  0     1     2  ####
    ####  3    ADC    4  ####
    ####  5     6     7  ####
    object_location_list = [None] * 8
    object_location_list[0] = sim.map_point_on_lane(
        adc_position - lane_width * right + 13 * forward
    ).position
    object_location_list[1] = sim.map_point_on_lane(
        adc_position + 13 * forward
    ).position
    object_location_list[2] = sim.map_point_on_lane(
        adc_position + lane_width * right + 13 * forward
    ).position
    object_location_list[3] = sim.map_point_on_lane(adc_position - lane_width * right).position
    object_location_list[4] = sim.map_point_on_lane(adc_position + lane_width * right).position
    object_location_list[5] = sim.map_point_on_lane(
        adc_position - lane_width * right - 13 * forward
    ).position
    object_location_list[6] = sim.map_point_on_lane(
        adc_position - 13 * forward
    ).position
    object_location_list[7] = sim.map_point_on_lane(
        adc_position + lane_width * right - 13 * forward
    ).position

    return object_location_list

def on_collision(agent1, agent2, contact):
    print("Ego collided with {}".format(agent2))
    time.sleep(2)
    raise lgsvl.evaluator.TestException("Ego collided with {}".format(agent2))


def on_destination(agent):
    print("Ego reached destination")
    raise lgsvl.evaluator.TestException("Ego reached destination")

def transform_to_right(tr):
    ax = tr.rotation.x * math.pi / 180.0
    ay = tr.rotation.y * math.pi / 180.0
    az = tr.rotation.z * math.pi / 180.0

    sx, cx = math.sin(ax), math.cos(ax)
    sy, cy = math.sin(ay), math.cos(ay)
    sz, cz = math.sin(az), math.cos(az)

    return lgsvl.Vector(sx * sy * sz + cy * cz, cx * sz, sx * cy * sz - sy * cz)


def transform_to_up(tr):
    ax = tr.rotation.x * math.pi / 180.0
    ay = tr.rotation.y * math.pi / 180.0
    az = tr.rotation.z * math.pi / 180.0

    sx, cx = math.sin(ax), math.cos(ax)
    sy, cy = math.sin(ay), math.cos(ay)
    sz, cz = math.sin(az), math.cos(az)

    return lgsvl.Vector(sx * sy * cz - cy * sz, cx * cz, sy * sz + sx * cy * cz)

def get_unit_vector(sim, position):
    transform = sim.map_point_on_lane(position)

    return (
        transform_to_forward(transform),
        transform_to_right(transform),
        transform_to_up(transform),
    )

def bug_oracle_tag(chromosome, oracle, sim, ego, object_list, object_location_list):
    global scenario
    bug_oracle = []
    npc_data = []

    forward, _, _ = get_unit_vector(sim, ego.state.position)

    ### NPC BEHAVIOR DECISION ###
    min_dist = 100000
    min_dist_idx = -1
    for i in range(len(object_location_list)):
        dist = cal_euc(
            chromosome[6],
            chromosome[7],
            object_location_list[i].x,
            object_location_list[i].z,
        )
        if dist < min_dist:
            min_dist = dist
            min_dist_idx = i
    npc_data.append(
        "NPCLocation: L%d(%f, %f)" % (min_dist_idx + 1, chromosome[6], chromosome[7])
    )
    npc_data.append(", NPCManeuver: " + Motion(chromosome[8]).name)
    npc_behavior = "".join(npc_data)

    ### 0. REMOVE OUT-OF-SCOPE BUGS ###
    if oracle == "collision" and ego.state.speed < speed_buffer:
        bug_oracle.append("BugOracle: COLLISION")
        root_cause = "OUT_OF_SCOPE ERROR: Collision occurred while stopped"
        bug_oracle.append(root_cause)
        return ", ".join(bug_oracle), npc_behavior

    # if oracle == "collision" and 0 < lgsvl.evaluator.separation(ego.state.position, object_list[0].state.position) < 5:
    if oracle == "collision" and 0 < (ego.state.position - object_list[0].state.position).dot(forward) < 5:
        bug_oracle.append("BugOracle: COLLISION")
        root_cause = "OUT_OF_SCOPE ERROR: Collision occurred by following NPC vehicle"
        bug_oracle.append(root_cause)
        return ", ".join(bug_oracle), npc_behavior
    
    if oracle == "collision" and cal_euc(ego.state.position.x, ego.state.position.z, chromosome[0], chromosome[2]) < 5:
        bug_oracle.append("BugOracle: COLLISION")
        root_cause = "OUT_OF_SCOPE ERROR: NPC vehicle initial position is too close to ego vehicle"
        bug_oracle.append(root_cause)
        return ", ".join(bug_oracle), npc_behavior

    return ", ".join(bug_oracle), npc_behavior

if __name__ == "__main__":
    global forward, right, available_object_location
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--adc_ip", type=str, default="0.0.0.0", help="Specify IP of autopilot"
    )
    parser.add_argument(
        "--adc_port", type=int, default=9090, help="Specify PORT number of autopilot"
    )
    parser.add_argument(
        "--map",
        type=str,
        default="Straight1LaneSame",
        help="Specify testing map (Dreamview map name)",
    )
    parser.add_argument(
        "--vehicle",
        type=str,
        default="Lincoln2017MKZ",
        help="Specify testing vehicle (Dreamview vehicle name)",
    )
    parser.add_argument(
        "--modules", type=str, default="", help="Specify Apollo modules you want to use"
    )
    parser.add_argument(
        "--dvport", type=str, default="8888", help="Specify Dreamview port number"
    )
    parser.add_argument(
        "--chromosome",
        type=str,
        default="[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]",
        help="Specify input to replay",
    )
    parser.add_argument(
        "--dest_xyz", action="store_true", help="Give destination x, y, z"
    )
    parser.add_argument(
        "--emergency_tick_start",
        type=int,
        default=8,
        help="Specify the start tick of the emergency action",
    )
    parser.add_argument(
        "--emergency_tick_end",
        type=int,
        default=50,
        help="Specify the end tick of the emergency action",
    )
    parser.add_argument(
        "--scenario",
        type=str,
        default="",
        help="Specify the scenario of the seed",
    )

    # parse arguments
    FLAGS, unparsed = parser.parse_known_args()
    ADC_IP = FLAGS.adc_ip
    ADC_PORT = FLAGS.adc_port
    test_map = FLAGS.map
    test_vehicle = FLAGS.vehicle
    modules = ast.literal_eval(FLAGS.modules)
    DREAMVIEW_PORT = FLAGS.dvport
    chromosome = ast.literal_eval(FLAGS.chromosome)
    destination_coordinate = FLAGS.dest_xyz
    emergency_tick_start = FLAGS.emergency_tick_start
    emergency_tick_end = FLAGS.emergency_tick_end
    scenario = ast.literal_eval(FLAGS.scenario)

    #### APOLLO MODULE LIST INITIALIZE ####
    env = Env()
    try:
        apollo_modules = env.list("LGSVL__AUTOPILOT_0_VEHICLE_MODULES", modules)
        if len(apollo_modules) == 0:
            log.warning(
                "LGSVL__AUTOPILOT_0_VEHICLE_MODULES is empty, using default list: {0}".format(
                    apollo_modules
                )
            )
            modules = [
                "Recorder",
                "Localization",
                "Perception",
                "Transform",
                "Routing",
                "Prediction",
                "Planning",
                "Traffic Light",
                "Control",
            ]
    except Exception:
        log.warning(
            "LGSVL__AUTOPILOT_0_VEHICLE_MODULES is not set, using default list: {0}".format(
                apollo_modules
            )
        )
        modules = [
            "Localization",
            "Perception",
            "Transform",
            "Routing",
            "Prediction",
            "Planning",
            "Traffic Light",
            "Control",
        ]

    # Set test map (map id is in web UI)
    sim = lgsvl.Simulator(LGSVL__SIMULATOR_HOST, LGSVL__SIMULATOR_PORT)
    if test_map == "Straight1LaneSame":
        print("Straight1LaneSame")
        scene_name = "1e2287cf-c590-4804-bcb1-18b2fd3752d1"
    elif test_map == "SanFrancisco":
        print("SanFrancisco")
        scene_name = "12da60a7-2fc9-474d-a62a-5cc08cb97fe8"
    elif test_map == "SanFrancisco Correct":
        print("SanFrancisco Correct")
        scene_name = "12da60a7-2fc9-474d-a62a-5cc08cb97fe8"
    elif test_map == "SanFrancisco Correct Adjusted":
        print("SanFrancisco Correct Adjusted")
        scene_name = "12da60a7-2fc9-474d-a62a-5cc08cb97fe8"
    elif test_map == "Borregas Ave":
        print("Borregas Ave")
        scene_name = "aae03d2a-b7ca-4a88-9e41-9035287a12cc"
    elif test_map == "SingleLaneRoad":
        print("Single Lane Road")
        scene_name = "a6e2d149-6a18-4b83-9029-4411d7b2e69a"
    elif test_map == "SingleLaneRoad_long":
        print("Single Lane Road Long")
        scene_name = "edc91085-948d-4127-a1f2-a0084ed80899"
    elif test_map == "Tartu":
        print("Tartu Beta Release 3")
        scene_name = "bd77ac3b-fbc3-41c3-a806-25915c777022"

    if sim.current_scene == scene_name:
        sim.reset()
    else:
        sim.load(scene_name)

    # Set spawn position of EGO vehicle
    ego_start_pos = lgsvl.Vector(
        chromosome[0], chromosome[1], chromosome[2]
    )

    # Creating state object to define state for Ego and NPC
    transform = sim.map_point_on_lane(ego_start_pos)
    forward = transform_to_forward(transform)
    right = transform_to_right(transform)
    up = transform_to_up(transform)
    adc_position = transform.position

    sim.reset()

    #### DATE & WEATHER SETTING ####
    sim.set_time_of_day(chromosome[-6])

    rain = chromosome[-5]
    fog = chromosome[-4]
    wetness = chromosome[-3]
    cloudiness = chromosome[-2]
    damage = chromosome[-1]
    sim.weather = lgsvl.WeatherState(rain, fog, wetness, cloudiness, damage)

    #### EGO VEHICLE SETTING ####
    egoState = lgsvl.AgentState()
    # Set start position of ego vehicle
    egoState.transform = sim.map_point_on_lane(ego_start_pos)
    # Set sensor configuration
    # DEFAULT
    ego = sim.add_agent(
        env.str("LGSVL__VEHICLE_0", "8fe5891d-0aa8-49da-a9bf-2c25666641a6"),
#        env.str("LGSVL__VEHICLE_0", "22656c7b-104b-4e6a-9c70-9955b6582220"),
        lgsvl.AgentType.EGO,
        egoState,
    )
    # CAMERA TRANSFORM#
    #  ego = sim.add_agent("17e82eda-661f-449f-add7-a239d1677bc7", lgsvl.AgentType.EGO, egoState)
    # Connect ego vehicle to Baidu Apollo
    ego.connect_bridge(ADC_IP, ADC_PORT)

    # If reached to destination, end simulation
    ego.on_collision(on_collision)
    ego.on_destination_reached(on_destination)

    # Dreamview car & map setting
    dv = lgsvl.dreamview.Connection(sim, ego, ADC_IP, DREAMVIEW_PORT)
    dv.reset_all_modules()
    dv.set_hd_map(test_map)
    dv.set_vehicle(test_vehicle)

    # Send module setup & routing message to Dreamview
    if destination_coordinate:
        destination = lgsvl.Vector(chromosome[3], 10.2, chromosome[4])
    else:
        destination = egoState.position + chromosome[3] * forward + chromosome[4] * right
    print(destination)
    hit_down = sim.raycast(destination, -up, 1)
    hit_up = sim.raycast(destination, up, 1)
    if hit_down:
        #  print("raycast downward")
        destination = hit_down.point
    elif hit_up:
        #  print("raycast upward")
        destination = hit_up.point

    current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    status = dv.setup_apollo(destination.x, destination.z, apollo_modules, filename=str(current_time) + ".mp4")
    if not status:
        sys.exit("SETUP APOLLO FAILED")

    # os.mkdir(os.path.join(data_path, current_time))
    file_name = current_time + "_" + FLAGS.chromosome + '.txt'
    dv.start_case(file_name)

    #### OBSTACLE SETTING ####
    object_list = []
    object_num = 1
    for i in range(object_num):
        object_state = copy.deepcopy(egoState)
        object_state.transform = sim.map_point_on_lane(
            lgsvl.Vector(chromosome[6], 10.2, chromosome[7])
        )
        object_state.transform.position = lgsvl.Vector(
            chromosome[6], 10.2, chromosome[7]
        )
        #  print(object_state.position)
        obj = sim.add_agent("Sedan", lgsvl.AgentType.NPC, object_state)
        # obj = sim.add_agent("Bicycle", lgsvl.AgentType.NPC, object_state)
        if chromosome[8] == Motion.Cut_in_Follow.value:
            obj.follow_closest_lane(True, chromosome[9], lane_speed_limit, False)
            obj_direction = object_state.position - egoState.position
            isLeftChange = True if obj_direction.dot(right) > 0 else False
            obj.change_lane(isLeftChange)
        elif chromosome[8] == Motion.Cut_in_Accel.value:
            obj.follow_closest_lane(True, chromosome[9], lane_speed_limit + 1, False)
            obj_direction = object_state.position - egoState.position
            isLeftChange = True if obj_direction.dot(right) > 0 else False
            obj.change_lane(isLeftChange)
        elif chromosome[8] == Motion.Cut_in_Decel.value:
            obj.follow_closest_lane(True, chromosome[9], 0, False)
            obj_direction = object_state.position - egoState.position
            isLeftChange = True if obj_direction.dot(right) > 0 else False
            obj.change_lane(isLeftChange)
        elif chromosome[8] == Motion.Cut_out_Follow.value:
            obj.follow_closest_lane(True, chromosome[9], lane_speed_limit, False)
            isLeftChange = False
            obj.change_lane(isLeftChange)
        elif chromosome[8] == Motion.Cut_out_Accel.value:
            obj.follow_closest_lane(True, chromosome[9], lane_speed_limit + 1, False)
            isLeftChange = False
            obj.change_lane(isLeftChange)
        elif chromosome[8] == Motion.Cut_out_Decel.value:
            obj.follow_closest_lane(True, chromosome[9], 0, False)
            isLeftChange = False
            obj.change_lane(isLeftChange)
        elif chromosome[8] == Motion.Lane_keep_Follow.value:
            obj.follow_closest_lane(True, chromosome[9], chromosome[9], False)
        elif chromosome[8] == Motion.Lane_keep_Accel.value:
            obj.follow_closest_lane(True, chromosome[9], lane_speed_limit + 1, False)
        elif chromosome[8] == Motion.Lane_keep_Decel.value:
            obj.follow_closest_lane(True, chromosome[9], 0, False)
        else:
            sys.exit("OBJECT MOTION ERROR")

        object_list.append(obj)

    mapdata = {}
    map_data = Map()
    mapdata = dv.get_hdmap_data()["data"]
    ParseDict(mapdata, map_data)
    lane_table = LaneTable(map_data)
    junction_table = JunctionTable(map_data)
    # import pdb; pdb.set_trace()

    cnt = 0
    tick = 0
    stop = False
    first = True
    stop_reason_code = ""
    error_code = ""
    error_msg = ""
    oracle = ""
    emergency_failure_resason = ""
    reference_line_end = ""
    current_scenario = "UNKNOWN"
    routing_response_start = ""
    routing_response_end = ""
    lane_change_decision = False
    lane_borrow_decision = False
    nudge_decision = False
    pull_over_position_change = False
    previous_pull_over_position = None
    first_pull_over_position = None
    try:
        t0 = time.time()

        frame_cnt = 0
        while True:
            frame_cnt += 1
            sim.run(0.5)
            if frame_cnt == 1:

                # controllables = sim.get_controllables("signal")
                # for c in controllables:
                #     print(c)

                signal = sim.get_controllable(lgsvl.Vector(-55.39, 10.2, 285.87), 'signal')
                print(signal)

                control_policy = "green=100;red=10;loop"
                signal.control(control_policy)
                print(signal.control_policy)

            # reason_code initialization
            stop_reason_code = ""
            error_code = ""

            data = dv.get_instrumentation_data()
            trajectory = data["data"]["trajectory"]

            # Receive routing data
            if _is_json_key(data["data"], "routingResponse"):
                if first:
                    routing_response_start = get_routing_lane_list(data["data"]["routingResponse"])
                else:
                    routing_response_end = get_routing_lane_list(data["data"]["routingResponse"])

            # Receive planning internal data
            planning_internal_data = data["data"]["planningDebugMessage"]
            if planning_internal_data["laneBorrowDecision"]:
                lane_borrow_decision = True
            if planning_internal_data["laneChangeDecision"]:
                lane_change_decision = True

            if _is_json_key(planning_internal_data, "pullOverPosition"):
                current_pull_over_position = planning_internal_data["pullOverPosition"]

                if first_pull_over_position is None:
                    first_pull_over_position = current_pull_over_position
                    
                if previous_pull_over_position is not None and current_pull_over_position != previous_pull_over_position:
                    pull_over_position_change = True

                previous_pull_over_position = current_pull_over_position

            if len(planning_internal_data["referenceLinesString"]) > 0:
                if first:
                    reference_line_start = planning_internal_data["referenceLinesString"][0]
                    first = False
                else:
                    reference_line_end = planning_internal_data["referenceLinesString"][0]

            # check whether error has occurred in current frame
            current_status = trajectory["header"].get("status", None)
            if current_status:
                # check error codes at `https://github.com/ApolloAuto/apollo/blob/v7.0.0/modules/common/proto/error_code.proto`
                error_code = current_status["errorCode"]
                error_msg = current_status["msg"]
                is_error_occurred = is_error_occurred if error_code == 0 else 1

            #### LANE & TARGET LANE ID ####
            laneId = trajectory["laneId"]
            current_lane_id = sim.get_closest_lane(ego.state.position)

            #### TRAJECTORY POINT ####
            trajectoryPoint = trajectory["trajectoryPoint"]
            if len(trajectoryPoint) > 0:
                v_last = trajectoryPoint[-1]["v"]

            try:
                heading = trajectory["debug"]["planningData"]["adcPosition"]["pose"][
                    "heading"
                ]
                position = trajectory["debug"]["planningData"]["adcPosition"]["pose"][
                    "position"
                ]
                (adc_x, adc_y) = itemgetter("x", "y")(position)

                lanes = mapdata["lane"]
                for lane in lanes:
                    if lane["id"] == laneId[0] and lane["turn"] == "NO_TURN":
                        lane_points = lane["centralCurve"]["segment"][0]["lineSegment"][
                            "point"
                        ]
                        (x_0, y_0) = itemgetter("x", "y")(lane_points[0])
                        (x_1, y_1) = itemgetter("x", "y")(lane_points[1])

                    #### CURRENT SCENARIO CHECK ####
                    current_scenario = trajectory["debug"]["planningData"]["scenario"]["scenarioType"]

                    #### OBSTACLE INFORMATION ####
                    obstacles = trajectory["debug"]["planningData"]["obstacle"]
                    for obstacle in obstacles:
                        obs_id = obstacle["id"]
                        # if not obs_id == "DEST":
                        #     print(obs_id)
            except Exception as e:
                print("Exception " + str(e))
            
            #### OBSTACLE INFORMATION ####
            try:
                #### decision list of the first object ####
                obstacles = trajectory["debug"]["planningData"]["obstacle"]
                for obstacle in obstacles:
                    for obj_decision in obstacle["decisionTag"]:
                        if _is_json_key(obj_decision["decision"], "nudge"):
                            print("nudge EXISTS")
                            nudge_decision = True

                        # if _is_json_key(obj_decision["decision"], "stop"):
                        #     print("STOP EXISTS")
            except Exception as e:
                pass


            #### CRITICAL REGION ####
            if _is_json_key(trajectory, "criticalRegion"):
                criticalRegion = trajectory["criticalRegion"]
                #  print(criticalRegion)

            #### TRAJECTORY TYPE ####
            if _is_json_key(trajectory, "trajectoryType"):
                trajectoryType = trajectory["trajectoryType"]

            try:
                mainDecision = trajectory["decision"]["mainDecision"]
                stop_reason_code = mainDecision["stop"]["reasonCode"]
                changeLaneType = mainDecision["stop"]["changeLaneType"]
                vehicleSignal = trajectory["decision"]["vehicleSignal"]
            except Exception as e:
                pass

            #### BUG ORACLE CHECK ####
            #### DESTINATION REACHED CHECK ####
            dest_separation = lgsvl.evaluator.separation(
                ego.state.position, sim.map_point_on_lane(destination).position
            )
            if dest_separation < distance_to_dest_buffer:
                time.sleep(1)
                break

            #### ADC STOP ####
            if ego.state.speed < speed_buffer and not chromosome[5] == Emergency.EMER_STOP.value and not chromosome[5] == Emergency.EMER_PULL_OVER.value:
                cnt += 1
                if cnt % 5 == 0 and cnt != 0:
                    print("cnt: " + str(cnt))
            else:
                cnt = 0

            if cnt >= 40:
                if cal_euc(ego.state.position.x, ego.state.position.z, chromosome[0], chromosome[2]) > 5 and chromosome[5] == Emergency.NORMAL.value:
                    stop = True
                    oracle = "immobile"
                else:
                    dv.reset_planning_module()
                break

            if tick == emergency_tick_start:
                if chromosome[5] == Emergency.EMER_STOP.value:
                    dv.emergency_stop()
                elif chromosome[5] == Emergency.EMER_PULL_OVER.value:
                    dv.pull_over()

            if tick == emergency_tick_end:
                if (
                    chromosome[5] == Emergency.EMER_STOP.value
                    or chromosome[5] == Emergency.EMER_PULL_OVER.value
                ):
                    # check if the vehicle is stopped
                    if ego.state.speed > speed_buffer:
                        stop = True
                        oracle = "mission_failure"
                        emergency_failure_resason = "ADC moving during EMERGENCY MISSION"

                    # check if the vehicle is pulled over
                    right_neighbor_lanes = lane_table.table[
                        current_lane_id
                    ].lane.right_neighbor_forward_lane_id
                    if chromosome[5] == Emergency.EMER_PULL_OVER.value and len(right_neighbor_lanes) > 0:
                        stop = True
                        oracle = "mission_failure"
                        emergency_failure_resason = "EMERGENCY_PULL_OVER_MISSION_FAILED"

                    dv.resume_cruise()
                    break

            #### TIME LIMIT EXCEEDED ####
            if time.time() - t0 > TIME_LIMIT:
                if (
                    chromosome[5] != Emergency.EMER_PULL_OVER.value
                    and chromosome[5] != Emergency.EMER_STOP.value
                ):
                    stop = True
                    oracle = "mission_failed"
                break

            tick += 1

    except lgsvl.evaluator.TestException as e:
        if f"{e}".startswith("Ego collided with "):
            stop = True
            oracle = "collision"

    if stop:
        routing_response_consistency = False if routing_response_start != routing_response_end else True
        ego_gps = sim.map_to_gps(ego.state.transform)
        start_scene, end_scene = scenario[0], scenario[1]
        if not current_scenario == (start_scene and end_scene):
            scene_decision = False
        else:
            scene_decision = True
        ego_position = lgsvl.Vector(chromosome[0], chromosome[1], chromosome[2])
        object_location_list = get_object_location_list(sim, ego_position)
        terminate_reason, npc_behavior = bug_oracle_tag(chromosome, oracle, sim, ego, object_list, object_location_list)
        if (
            chromosome[5] == Emergency.EMER_STOP.value
            or chromosome[5] == Emergency.EMER_PULL_OVER.value
        ):
            dv.resume_cruise()
        
        try:
            with open('solution_replay.json', 'r') as json_file:
                data = json.load(json_file)
        except FileNotFoundError:
            data = []

        result = {
            "timestamp": current_time,
            "npc_behavior": npc_behavior,
            "out_of_scope": terminate_reason,
            "stop_reason": {
                "oracle": oracle,
                "routing_consistency": routing_response_consistency,
                "scene_decision": scene_decision,
                "stop_reason_code": stop_reason_code,
                "error_msg": error_msg,
                "lane_change_decision": lane_change_decision,
                "lane_borrow_decision": lane_borrow_decision,
                "nudge_decision": nudge_decision,
                "pull_over_position_change": pull_over_position_change,
                "EMERGENCY_FAILURE_REASON": emergency_failure_resason,
                "scenario": {
                    "start": start_scene,
                    "end": end_scene
                    },
            },
            "chromosome": str(chromosome)
        }

        data.append(result)
        with open('solution_replay.json', 'w') as json_file:
            json.dump(data, json_file, indent=4)

    dv.resume_cruise()
    time.sleep(0.5)

    dv.end_case()
    dv.reset_planning_module()
    dv.reset_all_modules()
