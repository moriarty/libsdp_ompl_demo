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

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from itertools import product, combinations


def plotPathExample(data):
    """Plot a path using matplotlib pyplot
    It's still nice to have a pathVisualizer without the gui, for quick tests.
    """
    fig = plt.figure()
    axes = fig.gca(projection='3d')
    # draw box which is our simple invalid statespace box. HARDCODED.
    r = [-2, 2]
    for s, e in combinations(np.array(list(product(r, r))), 2):
        if np.sum(np.abs(s-e)) == r[1]-r[0]:
            axes.plot3D(*zip(s, e), color="b")
    axes.plot(data[:, 0], data[:, 1], '.-', c="r")
    plt.show()


class GeometricMultiPlanner():
    """
    This simple geometric Planning Demo
    """

    def __init__(self):
        """
        Create a new Kinematic Car Planning Demo.
        """
        
        self.state_space = ob.SE2StateSpace()
        self.car = og.SimpleSetup(self.state_space)
        self.setBounds()
        self.setStart()
        self.setGoal()

    def setBounds(self, high=None, low=None, dim=2):
        if high is None:
            high = [10.0, 10.0]
        if low is None:
            low = [-10.0, -10.0]
        print "setting bounds to", high, low
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

    def setRRTPlanner(self, goal_bias=0.05): 
        self.planner = og.RRT(self.car.getSpaceInformation())
        self.planner.setGoalBias(goal_bias)
        self.car.setPlanner(self.planner)
    
    def setPDSTPlanner(self, goal_bias=0.05): 
        self.planner = og.PDST(self.car.getSpaceInformation())
        self.planner.setGoalBias(goal_bias)
        self.car.setPlanner(self.planner)

    def setESTPlanner(self, goal_bias=0.05, r=0.00):
        """ Sets up an EST Planner with goal_bias and range r""" 
        self.planner = og.EST(self.car.getSpaceInformation())
        self.planner.setGoalBias(goal_bias)
        self.planner.setRange(r)
        self.car.setPlanner(self.planner)

    def setKPIECE1Planner(self, goal_bias=0.05, border_fraction=0.80): 
        self.planner = og.KPIECE1(self.car.getSpaceInformation())
        self.planner.setGoalBias(goal_bias)
        self.planner.setBorderFraction(border_fraction)
        self.car.setPlanner(self.planner)
    
    def solve(self, time=1.0):
        if self.car.solve(time):
            self.car.simplifySolution()
            self.path = self.car.getSolutionPath()
            if not self.car.haveExactSolutionPath():
                print("Solution is approximate. Distance to actual goal is %g" 
                    % self.car.getProblemDefinition().getSolutionDifference())
            return True
        else:
            return False

    def getPath(self):
        return self.car.getSolutionPath()

    def clear(self):
        self.car.clear()

    def deletePath(self):
        self.path = None
        #self.car.clear()
        pass

    def getPlannerName(self):
        return self.car.getPlanner().getName()

    def setCollisonCheckingResolution(self, value=0.010):
        self.car.getSpaceInformation().setStateValidityCheckingResolution(
            value)


    def setStateValidityChecker(self, checker_fn=True):
        if callable(checker_fn):
            fn = checker_fn
        else:
            fn = lambda state: True
        self.car.setStateValidityChecker(
                ob.StateValidityCheckerFn(fn))



def main():
    """
    Main function of the GeometricMultiPlanner starts here
    """
    time = 5
    paths = []
    demo = GeometricMultiPlanner()

    demo.setStart(y=10, x=-10, yaw=3.14)
    demo.setGoal(y=10,x=10, yaw=3.14)
    demo.setBounds(high=[-50,50], low=[-50,50])

    for i in xrange(3):
        demo.setRRTPlanner()
        if demo.solve(time):
            paths.append([demo.getPath(),demo.getPlannerName()+str(i)])
            demo.deletePath()
            print "Appended one path"
        
        demo.setPDSTPlanner()
        if demo.solve(time):
            paths.append([demo.getPath(),demo.getPlannerName()+str(i)])
            demo.deletePath()
            print "Appeneded one path"

        demo.setESTPlanner()
        if demo.solve(time):
            paths.append([demo.getPath(),demo.getPlannerName()+str(i)])
            demo.deletePath()
            print "Appeneded one path"

    """
    demo.setKPIECE1Planner()
    if demo.solve(time):
        paths.append([demo.getPath(),demo.getPlannerName()])
        demo.deletePath()
        print "Appended one path"
    """

    fig, ax = plt.subplots(1)

    for path in paths:
        data = None
        data = np.loadtxt(StringIO(path[0].printAsMatrix()))
        ax.plot(data[:,0],data[:,1],label=path[1])
        print "plotting path"

    ax.legend(loc='upper right')
    
    plt.show()
    
if __name__ == '__main__':
    main()
