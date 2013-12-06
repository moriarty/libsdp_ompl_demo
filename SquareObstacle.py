#!/usr/bin/env python
"""
# Title: SquareObstacle
# Description: A working tutorial, learning OMPL
# Author: @moriarty & @amdshameer
# Licence:
"""

from Polygon.Shapes import Rectangle
from PolygonObstacle import PolygonObstacle

class SquareObstacle(PolygonObstacle):
    """docstring for SquareObstacle"""
    def __init__(self, size=1.0):
        square  = Rectangle(size)
        super(SquareObstacle, self).__init__(
            name="square", polygon=square)
