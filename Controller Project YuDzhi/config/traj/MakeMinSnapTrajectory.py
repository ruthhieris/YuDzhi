#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul  1 10:20:12 2018

@author: yudzhi
"""
import minSnapTraj

def makeMinSnapTraj(filename):
    """
    Extracts waypoints from a trajectory file and converts to a min snap
    trajectory by adding more waypoints and velocity\acceleration info
    INPUT: filename
    OUTPUT: MinSnapTrajectory.txt
    (float) time x y z velx vely velz accx accy accz
    """


if __name__ == "__main__":
    filename = 'FigureEightFF.txt'