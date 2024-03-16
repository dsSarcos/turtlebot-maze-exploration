#!/usr/bin/env python
'''
Manages the currentn task for the turtlebot the general idea is
to control when the robot explores vs solves the maze
'''

import rospy
from std_srvs.srv import Empty, Trigger
from std_msgs.msg import Float64

def explore():
    rospy.wait_for_service("exploration/explore_maze")
    try:
        explore_maze = rospy.ServiceProxy("exploration/explore_maze", Trigger)
        explore_maze()
        return
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

def exit():
    rospy.wait_for_service("maze_manager/solve_maze")
    try:
        solve_maze = rospy.ServiceProxy("maze_manager/sovle_maze", Empty)
        solve_maze()
        return
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

def main():
    rospy.init_node("task_manager")
    rospy.loginfo("Task manager started...")

    rospy.loginfo("Exploring maze")
    explore()
    rospy.loginfo("Maze explored; exiting")
    exit()

    rospy.loginfo("Maze escaped")

    rospy.loginfo(rospy.wait_for_message("map_coverage_percentage", Float64, timeout=20))

if __name__ == "__main__":
    try:
        main() 
    except rospy.ROSInterruptException as e:
        pass