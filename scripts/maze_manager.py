#!/usr/bin/env python3
# Author: Diego Santamaria-sarcos

# Sources:
# [1] http://wiki.ros.org/hector_compressed_map_transport 
# [2] http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython 
# [3] https://learnopencv.com/convex-hull-using-opencv-in-python-and-c/ 

'''
This node manages the turtle's solving of the maze
'''

import roslib
roslib.load_manifest('maze_simulation_sarcos_baden')

import rospy
import cv2 as cv
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, PoseStamped, Pose
from maze_simulation_sarcos_baden.srv import MotionPose
import math
import numpy as np

from cv_bridge import CvBridge, CvBridgeError

def send_goal(goal):
    rospy.wait_for_service("motion_control/move_to_blocking")
    try:
        motion_client = rospy.ServiceProxy("motion_control/move_to_blocking", MotionPose)
        motion_client(goal)
        return True
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)
        return False


def image_to_occupancy_grid(point):
    return [point[0], SIZE - point[1]]

def occupancy_grid_to_map(point):
    return [point[0] * RESOLUTION + MAP_ORIGIN[0], point[1] * RESOLUTION + MAP_ORIGIN[1]]

def image_to_map(point):
    return occupancy_grid_to_map(image_to_occupancy_grid(point))

def dist_to_initial(point):
    return math.dist(point, INITIAL_POSE)

def hull_comparator(shell):
    output = 0
    for i in range(len(shell)):
        p1 = shell[i][0]
        p2 = shell[(i + 1) % len(shell)][0]
        output += math.dist(p1,p2)
    return output

def filter_hull(hull):
    output = hull[1:]
    output.sort(reverse=True, key=hull_comparator)
    return output[0], output[1]

def get_segments(shell):
    lines = []
    for i in range(len(shell)):
        lines.append([shell[i], shell[(i+1)%len(shell)]])
    return lines 

def segment_comparator(line):
    return math.dist(line[0][0], line[1][0])

def point_comparator(point):
    return point[0]

def handle_solve_maze(req):
    # request for the completed map to be converted to image format [1]
    map_img = rospy.wait_for_message("map_image/full", Image, timeout=20)

    try:
        # convert map_img to cv material [2]
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(map_img, desired_encoding='mono8')
        cv.imwrite('/home/diego/catkin_ws/src/uga-rob-final/test_imgs/map.png', cv_img)
        print("Map image saved")

        # image test [3]
        blur = cv.blur(cv_img, (12,12))
        cv.imwrite('/home/diego/catkin_ws/src/uga-rob-final/test_imgs/blur.png', blur)
        print("Blurred map image saved")
        ret, thresh = cv.threshold(blur, 126,255, cv.THRESH_BINARY)
        cv.imwrite('/home/diego/catkin_ws/src/uga-rob-final/test_imgs/thresh.png', thresh)
        print("Thresh image saved")
        
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        hull = []
        points = []
        for i in range(len(contours)):
            hull.append(cv.convexHull(contours[i]))
            points.append(cv.convexHull(contours[i], returnPoints=True))

        img_contours = np.zeros((thresh.shape[0],thresh.shape[1],3), np.uint8)
        for contour in range(len(contours)):
            cv.drawContours(img_contours, contours, contour, (0,255,0), 1, 8, hierarchy)
            cv.drawContours(img_contours, hull, contour, (0,0,255), 1, 8, hierarchy)
        cv.imwrite('/home/diego/catkin_ws/src/uga-rob-final/test_imgs/contours.png', img_contours)
        print("Contours image saved")

        shells = filter_hull(points)

        polygons = get_segments(shells[0]), get_segments(shells[1])

        diagonal = max(polygons[0], key=segment_comparator), max(polygons[1], key=segment_comparator)

        diagonal = (max(diagonal, key= segment_comparator))

        p1, p2 = diagonal[0][0], diagonal[1][0]

        lines_img = cv.cvtColor(cv_img, cv.COLOR_GRAY2RGB)
        cv.line(lines_img, p1, p2, (255,0,0), 1, 8)        
        cv.imwrite('/home/diego/catkin_ws/src/uga-rob-final/test_imgs/diagonal.png', lines_img)
        print("Diagonal image saved")

        endpoints = image_to_map(p1) , image_to_map(p2)

        exit = max(endpoints, key=dist_to_initial)

        goal = Pose()
        goal.position.x = exit[0]
        goal.position.y = exit[1]
        goal.orientation.w = 1.0

        send_goal(goal)
        return EmptyResponse

    except CvBridgeError as e:
        print(e) 

    return EmptyResponse()


def main():
    rospy.init_node("maze_manager")
    rospy.loginfo("Maze manager started")

    rospy.loginfo("Getting initial pose")
    initial_pose = rospy.wait_for_message("/motion_control/map_pose", PoseStamped, timeout=20)
    global INITIAL_POSE 
    INITIAL_POSE = [initial_pose.pose.position.x,initial_pose.pose.position.y] # in map frame
    print("Pose ", INITIAL_POSE)

    rospy.loginfo("Getting map metadata")
    map = rospy.wait_for_message("/map", OccupancyGrid, timeout=20)
    global SIZE
    SIZE = map.info.height
    global RESOLUTION
    RESOLUTION = map.info.resolution
    global MAP_ORIGIN
    MAP_ORIGIN = [map.info.origin.position.x, map.info.origin.position.y]

    rospy.loginfo("Establishing server")
    rospy.Service("/maze_manager/solve_maze", Empty, handle_solve_maze)

    try:
        rospy.spin()
    except:
        cv.destroyAllWindows()

if __name__ == "__main__":
    try:
        main() 
    except rospy.ROSInterruptException as e:
        pass