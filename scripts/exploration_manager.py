#!/usr/bin/env python
# Author : Diego Santamaria-sarcos
# Author : Marius Baden
#
# Offers a service to fully explore the maze by a frontier based exploration algorithm.
# 
# We assume that the dimensions of the maze are known. Thus, we can spread a grid of 
# frontier points over the maze and explore the maze by visiting all of these points.
#
# Furthermore, we assume the center of the maze being (0,0).
#

import copy
import math
import rospy
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from maze_simulation_sarcos_baden.srv import MotionPose
from utility.RvizMarkerArrayPublisher import RvizMarkerArrayPublisher  
from utility.FrontierPoint import FrontierPoint
from utility.ExplorationMotionStrategy import ExplorationMotionStrategy

# These values are determined from Gazebo.
MAZE_LIMITER_START = (-3.5, -3) # Given in class: (-4, -3.5)
MAZE_LIMITER_END = (3.5, 3) # Given in class: (4, 3.5)
FRONTIER_STE_SIZE = 0.75
BACK_TO_BACK_FAILURE_LIMIT = 5
VISITING_DISTANCE_THRESH_UNATTEMPTED = 0.25
VISITING_DISTANCE_THRESH_FAILED = 0.5

robot_pose = None
occupancy_grid = None
goal_point = None
marker_publisher = None
exploration_motion_strategy = None
frontier_points_grid = None
frontier_size = None
unattempted_points_flat = None


def main():
    global exploration_motion_strategy
    # TODO Remove debug level for production
    rospy.init_node("exploration_manager", log_level=rospy.DEBUG) 

    generate_frontier_points()

    exploration_motion_strategy = ExplorationMotionStrategy(marker_publisher, VISITING_DISTANCE_THRESH_UNATTEMPTED)

    rospy.loginfo("Exploration manager started")
    rospy.Subscriber("/map", OccupancyGrid, __occupancy_grid_callback)
    rospy.Subscriber("/motion_control/map_pose", PoseStamped, __robot_pose_callback)

    # Wait until we have received the first occupancy grid and robot pose
    rospy.logdebug("Waiting for occupancy grid and robot pose...")
    while occupancy_grid is None or robot_pose is None:
        rospy.sleep(0.5)
    rospy.logdebug("Occupancy grid and robot pose received")

    rospy.Service('/exploration/explore_maze', Trigger, explore_maze)
    
    rospy.spin()


def generate_frontier_points():
    global marker_publisher, frontier_points_grid, unattempted_points_flat, frontier_size 
    x_range = np.arange(MAZE_LIMITER_START[0], MAZE_LIMITER_END[0], FRONTIER_STE_SIZE)
    y_range = np.arange(MAZE_LIMITER_START[1], MAZE_LIMITER_END[1], FRONTIER_STE_SIZE)

    frontier_size = (len(x_range), len(y_range))
    frontier_points_grid = [[None]*frontier_size[1] for _ in x_range]

    for i,x in enumerate(x_range):
        for j,y in enumerate(y_range):
            frontier_points_grid[i][j] = FrontierPoint(x,y)

    unattempted_points_flat = [point for sublist in frontier_points_grid for point in sublist]
    
    marker_publisher = RvizMarkerArrayPublisher(unattempted_points_flat)

    # Publish a few times to make them show up in rviz
    for i in range(10):
        marker_publisher.refresh()
        rospy.sleep(0.1)


def explore_maze(req):
    global unattempted_points_flat
    rospy.loginfo("Start exploring maze maze...")

    back_to_back_failures = 0


    #TODO Subscribe to progress node and include in check

    while len(unattempted_points_flat):

        failed_queue = []
        all_points_failed = True

        # Try to visit all frontier points. Abort if move_base fails to go there.
        while len(unattempted_points_flat) > 0:
            #
            # Pick the closest point to the robot pose that is within the global cost map
            #
            possible_points = copy.copy(unattempted_points_flat)
            point_to_explore = __get_closest_point(possible_points)

            # Skip points that are not within the global cost map 
            while (point_occupancy_cost < 0 or point_occupancy_cost == 100):
                possible_points.remove(point_to_explore)
                if point_occupancy_cost == 0:
                    rospy.logdebug("Skipped {} as it is not within explored part of the map".format(point_to_explore))
                else:
                    rospy.logdebug("Dropped {} as it is inside an obstacle".format(point_to_explore))
                    unattempted_points_flat.remove(point_to_explore)
                    point_to_explore.dropped = True

                if len(possible_points) > 0: 
                    point_to_explore = __get_closest_point(possible_points)
                    point_occupancy_cost = __get_cost_at_map_position(point_to_explore.x, point_to_explore.y)
                else: 
                    point_to_explore = __get_closest_point(unattempted_points_flat)
                    rospy.logwarn("All frontier points left were in unexplored terrain or occupied. Fell back to closest point in unexplored area")
                    break
            

            #
            # Explore the choosen point
            #
            exploration_motion_strategy.start_going_to_goal(point_to_explore, unattempted_points_flat + failed_queue)
            success, visited_points = exploration_motion_strategy.wait_until_reached_or_failed()
            unattempted_points_flat =   [point for point in unattempted_points_flat if (point not in visited_points)]
            failed_queue =              [point for point in visited_points if (point not in visited_points)]

            if not success:
                unattempted_points_flat.remove(point_to_explore)
                failed_queue.append(point_to_explore)
                point_to_explore.failed = True
            else:
                all_points_failed = False


            rospy.logdebug("Points left to explore: {}".format(len(unattempted_points_flat) + len(failed_queue)))


        # Retry all points that failed upon first attempt
        rospy.loginfo("Retrying total of {} failed points".format(len(failed_queue)))
        exploration_motion_strategy.visiting_distance_thresh = VISITING_DISTANCE_THRESH_FAILED
        unattempted_points_flat = failed_queue # unattempted_points_flat is empty at this point
        for point in unattempted_points_flat:
            point.failed = False
        marker_publisher.refresh()

        if not all_points_failed:
            back_to_back_failures = 0

        elif back_to_back_failures >= BACK_TO_BACK_FAILURE_LIMIT:
            exception_message = "Too many back to back failures ({}). Aborting exploration".format(back_to_back_failures)
            rospy.logerr(exception_message)
            return TriggerResponse(False, exception_message)

        else:
            # TODO randomize the order of the points to explore 
            rospy.logwarn("All points failed. This occured back to back now for {} time(s)".format(back_to_back_failures))
            back_to_back_failures += 1




    return TriggerResponse(True, "Maze explored")

def __get_closest_point(point_list):
    min_dist = float("inf")
    min_dist_point = None

    for point in point_list:
        dist = __calc_distance_to_robot(point)
        if dist < min_dist:
            min_dist = dist
            min_dist_point = point

    rospy.logdebug("Calculated closest point {} to robot in {:.2f}m".format(repr(min_dist_point), min_dist))
    return min_dist_point

def __calc_distance_to_robot(point):
    return math.sqrt((robot_pose.position.x - point.x)**2 + (robot_pose.position.y - point.y)**2)

# Returns the cost of the cell at the given position in the global cost map
# Occupancy probabilities are in the range [0,100].
# If the grid has not been mapped yet, the value is -1.
def __get_cost_at_map_position(map_x, map_y):
    grid_coord_x = int(round((map_x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution))
    grid_coord_y = int(round((map_y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution))

    # Check if the map pose is within the bounds of the occupancy grid
    if     grid_coord_x < 0 or grid_coord_x >= occupancy_grid.info.width \
        or grid_coord_y < 0 or grid_coord_y >= occupancy_grid.info.height:
            rospy.logerr("Map pose is out of bounds of the occupancy grid")
            return None

    return occupancy_grid.data[grid_coord_y * occupancy_grid.info.width + grid_coord_x] 

def __occupancy_grid_callback(msg):
    global occupancy_grid
    occupancy_grid = msg

def __robot_pose_callback(msg):
    global robot_pose
    robot_pose = msg.pose
    exploration_motion_strategy.set_robot_pose(msg.pose)

if __name__ == "__main__":
    try:
        main() 
    except rospy.ROSInterruptException as e:
        pass