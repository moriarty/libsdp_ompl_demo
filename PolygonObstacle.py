#!/usr/bin/env python
"""
# Title: Obstacle.py 
# Description: A working tutorial, learning OMPL
# Author: @moriarty & @amdshameer
# Licence:
"""

class PolygonObstacle(object):
    """docstring for PolygonObstacle"""
    def __init__(self, name, polygon):
        super(PolygonObstacle, self).__init__()
        self.name = name
        self.polygon = polygon

    def move_to(self, x, y):
        curr_x, curr_y = self.polygon.center()
        self.polygon.shift(x-curr_x, y-curr_y)

    def get_vertices(self):
        return self.polygon.contour(0)

    def isInside(self, state):
        x = state.getX()
        y = state.getY()
        return not self.polygon.isInside(x, y)

