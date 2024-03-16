#!/usr/bin/env python
# Author : Diego Santamaria-sarcos
# Author : Marius Baden
#
# Encapsulates the logic for going to a chosen fontier point.
# Makes the robot go to a goal point using move_base. Determines when the
# robot has reached the goal point.
# It takes into account that given goal points may not be reachable and 
# that other frontier points may be passed on the way to the goal point.

import copy
import math
import rospy
from actionlib_msgs.msg import GoalStatus
from utility.FrontierPoint import FrontierPoint
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib
from maze_simulation_sarcos_baden.srv import MotionPose

class ExplorationMotionStrategy:

    FEEDBACK_MONITORING_CYCLE_TIME = 0.05
    FEEDBACK_MONITORING_BREAKOUT_SLEEP_TIME = 0.1
    STATUS_ACTIVE = GoalStatus().ACTIVE
    STATUS_PENDING = GoalStatus().PENDING
    STATUS_PREEMPTED = GoalStatus().PREEMPTED


    def __init__(self, marker_publisher, visiting_distance_threshold):
        self.marker_publisher = marker_publisher
        self.robot_pose = None
        self.goal = None
        self.unvisited_points = None
        self.visiting_distance_threshold = visiting_distance_threshold
        self.visited_points_in_current_iteration = None
        self.enable_move_base_feedback_monitoring = False

        # True if the robot has reached the goal point, False otherwise.
        self.result_status = None 

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Start monitoring move_base feedback in a separate thread
        self.monitoring_thread = rospy.Timer(rospy.Duration(ExplorationMotionStrategy.FEEDBACK_MONITORING_CYCLE_TIME), self.__move_base_status_monitoring)

    #  Makes the robot go to a goal point using move_base.
    #
    #  Takes the goal point and a list of unvisited frontier point (usually incl. goal) as input. 
    #  The list of unvisited frontier points is not modified by this method!
    #
    #  Subscribes to /move_base/feedback to
    #  - check if robot passes by closely to a frontier point on the unexplored list.
    #    In that case, set point.visited = true
    #  - check if robot comes close enough (according to VISITING_DISTANCE_THESHOLD) to goal point.
    #    In that case, complete this iteration and go to the next one
    #  - check if move_bases fails to go to the goal (/move_base/feedback.status == 4).
    #    In that case, abort the movement, remove the point from unexplored list,
    #    go to next iteration and mark the point with
    #    an aborted flag or smth.
    def start_going_to_goal(self, goal_frontier_point, unvisited_points):
        rospy.loginfo("Try to explore {}".format(goal_frontier_point))
        self.goal = goal_frontier_point
        self.unvisited_points = copy.copy(unvisited_points)
        self.result_status = None
        self.visited_points_in_current_iteration = []

        # Instruct move_base to go to the goal point
        goal_pose = self.goal.to_pose(self.robot_pose.orientation)
        self.__move_to_nonblocking(goal_pose)

        # Start monitoring move_base feedback
        self.enable_move_base_feedback_monitoring = True



    # Blocks until the robot has reached the goal point or the node decides that is not possible to reach the goal.
    # Returns True if the robot has reached the goal point, False otherwise.
    def wait_until_reached_or_failed(self):
        rospy.logdebug("Move_base monitoring: Waiting to finish...")
        while self.enable_move_base_feedback_monitoring and not rospy.is_shutdown():
            rospy.sleep(self.FEEDBACK_MONITORING_BREAKOUT_SLEEP_TIME)
        rospy.logdebug("Move_base monitoring: Finished")

        return self.result_status, self.visited_points_in_current_iteration
        

    def set_robot_pose(self, pose):
        self.robot_pose = pose


    def __move_base_status_monitoring(self, event=None):
        if self.enable_move_base_feedback_monitoring:
            status_id = self.move_base_client.get_state()
            rospy.logdebug("move_base feedback received: ID {}, Text '{}'".format(status_id, self.move_base_client.get_goal_status_text()))
                
            # Check if robot passes by closely to a frontier point (incl. the goal!) on the unexplored list.
            # In that case, set point.visited = true.
            for point in self.unvisited_points:
                dist = self.__calc_distance_to_robot(point)
                if dist < self.visiting_distance_threshold:
                    point.visited = True
                    self.unvisited_points.remove(point)
                    self.visited_points_in_current_iteration.append(point)
                    rospy.loginfo("Frontier point {} has been passed in distance {:0.2f}".format(repr(point), dist))

            # Redraw the markers in RViz
            self.marker_publisher.refresh()

            # Check if robot comes close enough to goal point.
            # In that case, complete current iteration and go to the next one.
            # This does NOT include setting the goal point to visited. Implemented this way to allow
            # goal points that are not frontier points.
            if self.__calc_distance_to_robot(self.goal) < self.visiting_distance_threshold:
                rospy.loginfo("Exploration goal {} point reached".format(repr(self.goal))) 
                self.__finish_monitoring(True)
                return
            
            # Check if move_bases fails to go to the goal.
            # In that case, abort the movement, remove the point from unexplored list,
            # go to next iteration and mark the point with an aborted flag or smth.
            #
            # Note that status 3 ("Goal reached") is all considered a failure 
            # if the robot has not made it closer to the goal then VISTING_DISTANCE_THRESHOLD.
            if not (status_id in [self.STATUS_ACTIVE, self.STATUS_PENDING, self.STATUS_PREEMPTED]):
                rospy.loginfo("move_base failed to go to goal point. Status (id: {}, description: '{}'".format(status_id, self.move_base_client.get_goal_status_text()))
                self.__finish_monitoring(False)
                return

            

    def __finish_monitoring(self, navigation_succeeded):
        self.enable_move_base_feedback_monitoring = False
        self.result_status = navigation_succeeded

        # For better early detection of bugs
        self.goal = None
        self.unvisited_points = None


    def __calc_distance_to_robot(self, point):
        return math.sqrt((self.robot_pose.position.x - point.x)**2 + (self.robot_pose.position.y - point.y)**2)
    

    def __move_to_nonblocking(self, target_pose):
        new_goal = MoveBaseGoal()
        new_goal.target_pose.header.frame_id = "map"
        new_goal.target_pose.header.stamp = rospy.Time.now()
        new_goal.target_pose.pose = target_pose
        self.move_base_client.send_goal(new_goal)
