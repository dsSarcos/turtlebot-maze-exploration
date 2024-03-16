#!/usr/bin/env python
# Author : Diego Santamaria-sarcos
# Author : Marius Baden
#
# Class that publishes FrontierPoints to rviz as markers

import copy
import rospy
from visualization_msgs.msg import Marker, MarkerArray

class RvizMarkerArrayPublisher:

    def __init__(self, managed_points_flat):
        self.managed_points_flat = copy.copy(managed_points_flat)
        self.marker_pub = rospy.Publisher("/exploration/frontier_points", MarkerArray, queue_size=1)
 
    # Publishes all points that are set to be managed by this object
    def refresh(self):
        self.publish_points(self.managed_points_flat)

    # Publishes the given points
    def publish_points(self, points_flat):

        marker_arr = MarkerArray()

        for (i,point) in enumerate(points_flat):
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()

            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            marker.type = 3
            marker.id = i
            marker.action = marker.ADD

            # Set the scale of the marker
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Set the color
            if point.visited:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
            elif point.failed:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            elif point.dropped:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0

            # Set the pose of the marker
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker_arr.markers.append(marker)

        self.marker_pub.publish(marker_arr)
