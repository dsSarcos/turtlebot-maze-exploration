#!/usr/bin/env python
# Author : Diego Santamaria-sarcos
# Author : Marius Baden
#
# Fascade for interacting with move_base and getting the robot's pose in map frame
# for the rest of the system.
#
# This page uses code from [1] https://stackoverflow.com/questions/69389307/how-to-get-robots-pose-estimation-data-from-the-slam-algorithm
#

import rospy, tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from maze_simulation_sarcos_baden.srv import MotionPose, MotionPoseResponse
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib

def main():
    # TODO Remove debug level in production code
    rospy.init_node("motion_controller", log_level=rospy.DEBUG)
    rospy.loginfo("Motion controller started...")
    
    global tfListener
    tfListener = tf.TransformListener()
    rospy.Subscriber("odom", Odometry, publish_robot_pose_in_map_frame)

    rospy.loginfo("Establishing servers for move_to actions")
    rospy.Service('/motion_control/move_to_nonblocking', MotionPose, move_to_nonblocking)
    rospy.Service('/motion_control/move_to_blocking', MotionPose, move_to_blocking)

    global move_base_client
    rospy.loginfo("Connecting to move_base action server")
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    global map_pose_pub
    map_pose_pub = rospy.Publisher("/motion_control/map_pose", PoseStamped, queue_size=10)

    rospy.spin()

def publish_robot_pose_in_map_frame(msg):
    #Source [1]
    odometry_pose = PoseStamped()
    odometry_pose.header = msg.header
    odometry_pose.pose = msg.pose.pose
    rospy.logdebug("Odometry pose: {}".format(odometry_pose.pose))
    map_pose = tfListener.transformPose("map", odometry_pose)
    rospy.logdebug("Map pose: {}".format(map_pose))
    map_pose_pub.publish(map_pose)
    
def move_to_nonblocking(req):
    new_goal = MoveBaseGoal()
    new_goal.target_pose.header.frame_id = "map"
    new_goal.target_pose.header.stamp = rospy.Time.now()
    new_goal.target_pose.pose = req.goal
    move_base_client.send_goal(new_goal)
    return MotionPoseResponse()

def move_to_blocking(req):
    new_goal = MoveBaseGoal()
    new_goal.target_pose.header.frame_id = "map"
    new_goal.target_pose.header.stamp = rospy.Time.now()
    new_goal.target_pose.pose = req.goal
    move_base_client.send_goal(new_goal)
    move_base_client.wait_for_result()
    return MotionPoseResponse()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException():
        pass