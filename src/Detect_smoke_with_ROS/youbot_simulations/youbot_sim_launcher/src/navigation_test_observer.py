#!/usr/bin/env python
import rospy
from observer.observer_api import ObserverApi
from observer.observer_common import *
from observer.conditions.nav_condition import NavCondition
#from observer.conditions.bool_condition import BoolCondition
#from observer.conditions.link_condition import LinkCondition
from observer.observables.collision_observable import CollisionObservable
from observer.observables.map_observable import MapObservable
from observer.observables.link_pose_observable import LinkPoseObservable
from observer.observables.pose_reached_observable_handler import PoseReachedObservableHandler
from observer.sim_state import SimState
from logger.logger_api import LoggerAPI

'''
roslaunch simple scenario (sim+base_bumber, controller, nav+amcl)
start this script 
in other terminal listen to /observer topic
drive around a bit
force a collision
drive to position 1 1 0 and turn in place

'''


def main():
    ## commands
    cs = "rosrun youbot_sim_launcher move_base_client.py 0.799 -4.4 0 0 0 0 1.0"
    c1 = "rosrun youbot_sim_launcher move_base_client.py 6.39 -3.96 0 0 0 0.73 0.68"
    c2 = "rosrun youbot_sim_launcher move_base_client.py 2.66 -1.55 0 0 0 -0.71 0.7"
    c3 = "rosrun youbot_sim_launcher move_base_client.py 2.65 -3.22 0 0 0 0.71 0.7"
    c4 = "rosrun youbot_sim_launcher move_base_client.py 5.76 -0.526 0 0 0 1.0 0"
    c5 = "rosrun youbot_sim_launcher move_base_client.py 1.41 -0.51 0 0 0 1.0 0"



    rospy.init_node("observer_nav_test")
    Sim_state = SimState()
    link1 = "youbot::base_footprint"
    Sim_state.register_link(link1)

    nav_cond1 = NavCondition(ID=1, name="S_To_1", command=c1)
    nav_cond1.set(sim_state=Sim_state, link_name=link1,
                  pose=[0.799, -4.4, 0, 0, 0, 0, 1.0])
    nav_cond2 = NavCondition(ID=2, name="1_To_2", command=c2)
    nav_cond2.set(sim_state=Sim_state, link_name=link1,
                  pose=[6.39, -3.96, 0, 0, 0, 0.73, 0.68])
    nav_cond3 = NavCondition(ID=3, name="2_To_3", command=c3)
    nav_cond3.set(sim_state=Sim_state, link_name=link1,
                  pose=[2.66, -1.55, 0, 0, 0, -0.71, 0.7])
    nav_cond4 = NavCondition(ID=4, name="3_To_4", command=c4)
    nav_cond4.set(sim_state=Sim_state, link_name=link1,
                  pose=[2.65, -3.22, 0, 0, 0, 0.71, 0.7])
    nav_cond5 = NavCondition(ID=5, name="4_To_5", command=c5)
    nav_cond5.set(sim_state=Sim_state, link_name=link1,
                  pose=[5.76, -0.526, 0, 0, 0, 1.0, 0])
    nav_cond6 = NavCondition(ID=6, name="5_To_E")
    nav_cond6.set(sim_state=Sim_state, link_name=link1,
                  pose=[1.41, -0.51, 0, 0, 0, 1.0, 0])

    collision_o1 = CollisionObservable()
    linkpose_o1 = LinkPoseObservable(sim_state=Sim_state, link_name=link1)
    map_o = MapObservable()

    path = get_ros_sourcefolder_path()+"/automatic_simulation/tests"
    logger = LoggerAPI()
    observable_handler = PoseReachedObservableHandler(
        logger, nav_cond1, [linkpose_o1, map_o], 0.5,
        path)

    observer = ObserverApi()
    observer.set(max_time=30)
    observer.add_start_command(command=c5)
    observer.add_condition(nav_cond1)
    observer.add_condition(nav_cond2)
    observer.add_condition(nav_cond3)
    observer.add_condition(nav_cond4)
    observer.add_condition(nav_cond5)
    observer.add_condition(nav_cond6)
    observer.use_standard_start_topic()
    #observer.use_costum_start_topic(topic_name="/start")

    rosbag_started = False
    count = 0
    topics = ["/map", "/tf", "/cmd_vel", "/gazebo/cmd_vel", "/scan",
              "/scan_back"]

    while not rospy.is_shutdown():
        is_running = observer.run_once()
        if is_running:
            if not rosbag_started:
                rosbag_started = True
                logger.start_rosbag(path, "bag_"+str(count), topics)
            if observable_handler.get_state():
                observable_handler.clear()
        elif not is_running and rosbag_started:
            rosbag_started = False
            count = count + 1
            logger.stop_rosbag()

if __name__=="__main__":
    main()

