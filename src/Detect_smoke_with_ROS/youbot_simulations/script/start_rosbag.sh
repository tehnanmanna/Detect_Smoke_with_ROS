#!/bin/bash

rosbag record /tf /tf_static /statistics /monitoring /monitoring/errors /rosout_agg /B_CONTROLLER/id0/E10_out_vehicle_motion_cmd /B_CONTROLLER/id0/E10_out_vehicle_motion_cmd_debug /A_SELFLOCALIZATION/id5/A10_out_localization_data /S_LEADERTRACKINGBOX/id6/A10_out_localization_data /B_PATHPLANNER/id0/B10_out_planner_data /diagnostics /A_TRACKING/id0/C00_ObjectData /B_PATHPLANNER/id1/B10_out_planner_data /B_PATHPLANNER/id2/B10_out_planner_data /test /E_VEHICLE/id0/E10_driver_intervention /A_SELFLOCALIZATION/id2/A10_out_localization_data /B_WAYPOINTFOLLOW/id0/E10_vehicle_motion_cmd /NGVA/id0/E10_vehicle_motion_cmd
