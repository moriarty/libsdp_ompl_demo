#!/usr/bin/env python
"""
# Title: KinematicCarPlanning.py
# Description: A working tutorial, learning OMPL
# Author: @moriarty & @amdshameer
# Licence:
"""
# Original Demo notes:
# Author: Mark Moll
# Rice University Software Distribution License

import numpy as np
from StringIO import StringIO

# Import ompl assuming it's properly installed
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

class KinematicCarPlanningDemo2D():
    """
    This simple Kinematic Car Planning Demo
    """

    def __init__(self):
        """
        Create a new Kinematic Car Planning Demo.
        """
        self.car = oa.KinematicCarPlanning()
        self.state_space = self.car.getStateSpace()
        self.setBounds()
        self.setStart()
        self.setGoal()
        self.setPlanner()

    def setBounds(self, high=None, low=None, dim=2):
        if high is None:
            high = [10.0, 10.0]
        if low is None:
            low = [-10.0, -10.0]
        bounds = ob.RealVectorBounds(dim)
        (bounds.low[0], bounds.low[1]) = low
        (bounds.high[0], bounds.high[1]) = high
        self.state_space.setBounds(bounds)

    def setStart(self, x=0, y=0, yaw=0):
        self.start = ob.State(self.state_space)
        self.start().setXY(x, y)
        self.start().setYaw(yaw)
        self.car.setStartState(self.start)

    def setGoal(self, x=0, y=0, yaw=0, threshold=0.1):
        self.goal = ob.State(self.state_space)
        self.goal().setXY(x, y)
        self.goal().setYaw(yaw)
        self.car.setGoalState(self.goal, threshold)

    def setPlanner(self): 
        self.planner = oc.RRT(self.car.getSpaceInformation())
        self.car.setPlanner(self.planner)
    
    def solve(self, time=1.0):
        if self.car.solve(time):
            self.path = self.car.getSolutionPath()
            if not self.car.haveExactSolutionPath():
                print("Solution is approximate. Distance to actual goal is %g" 
                    % self.car.getProblemDefinition().getSolutionDifference())

    def getPath(self):
        return self.path.asGeometric()

if __name__ == '__main__':
    demo = KinematicCarPlanningDemo2D()

    demo.setStart(y=3, x=4, yaw=3.14)
    demo.setGoal(y=1,x=1, yaw=3.14)
    
    demo.solve(5.0)

    path = demo.getPath()
    print type(path)

    print np.loadtxt(StringIO(path.printAsMatrix()))
