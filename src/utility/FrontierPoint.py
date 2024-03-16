#!/usr/bin/env python
# Author : Diego Santamaria-sarcos
# Author : Marius Baden
#
# Represents a point in the map that is used for exploring the maze. 
# In general, it is a point that the robot is supposed to visit in
# order to explore the maze.

from geometry_msgs.msg import Pose

class FrontierPoint:

    def __init__(self, x, y, visited=False, failed=False, dropped=False):
        self.x = x
        self.y = y
        self.visited = visited
        self.failed = failed
        self.dropped = dropped


    def to_pose(self, orientation):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.orientation = orientation
        return pose

    def __str__(self):
        return "FrontierPoint: ({}, {})".format(self.x, self.y)

    def __repr__(self):
        return "({}, {})".format(self.x, self.y)