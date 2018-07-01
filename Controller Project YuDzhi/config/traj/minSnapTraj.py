#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 25 23:34:10 2018

@author: UdacityFlyingCars, YuDzhi
"""

import numpy as np 
import jdc
from numpy.polynomial.polynomial import polyval

def matrix_generation(ts):
    b =np.array([[1, ts,  ts**2, ts**3,    ts**4,    ts**5,    ts**6,    ts**7],
                 [0, 1 ,2*ts,  3*ts**2,  4*ts**3,  5*ts**4,  6*ts**5,  7*ts**6],
                 [0, 0 ,2,     6*ts,    12*ts**2, 20*ts**3, 30*ts**4, 42*ts**5],
                 [0, 0 ,0,     6,       24*ts,    60*ts**2,120*ts**3,210*ts**4],
                 [0, 0 ,0,     0,       24   ,   120*ts   ,360*ts**2,840*ts**3],
                 [0, 0 ,0,     0,       0    ,   120      ,720*ts  ,2520*ts**2],
                 [0, 0 ,0,     0,       0    ,   0        ,720     ,5040*ts   ],
                 [0, 0 ,0,     0,       0    ,   0        ,0       ,5040      ]])
    
    return b

def multiple_waypoints(t):
    """
    Generating M matrix for Mc=b equation,
    Where  c is the vector of polynomial coefficients belonging to the 
    m curves,  b contains the initial, final and intermediary conditions. 
    The  M matrix describes the relation between those coefficients.
    One important thing to keep in mind we will be parametrizing the curve 
    in each section by the dimensionless time which goes from -1 to 1. 
    This will help to maintain the  M matrix well defined and prevent the element 
    value increase due to increase in time as the elements that it contains are t^7
    """
    n= t.shape[0]-1
    
    m= np.zeros((8*n,8*n))
    
    for i in range(n):
    
        if i == 0:
            
            # initial condition of the first curve
            b = matrix_generation(-1.0)
            m[8*i:8*i+4,8*i:8*i+8] = b[:4,:]
            
            # intermidiary condition of the first curve
            b = matrix_generation(1.0)
            m[8*i+4:8*i+7+4,8*i:8*i+8] = b[:-1,:]
            
            # starting condition of the second curve position and derivatives 
            b = matrix_generation(-1.0)
            m[8*i+4+1:8*i+4+7,8*(i+1):8*(i+1)+8] = -b[1:-1,:]
            m[8*i+4+7:8*i+4+8,8*(i+1):8*(i+1)+8] = b[0,:]
            
        elif i!=n-1:
            
            # starting condition of the ith curve position and derivatives 
            b = matrix_generation(1.0)
            m[8*i+4:8*i+7+4,8*i:8*i+8] = b[:-1,:]
            
            # end condition of the ith curve position and derivatives 
            b = matrix_generation(-1.0)
            m[8*i+4+1:8*i+4+7,8*(i+1):8*(i+1)+8] = -b[1:-1,:]
            m[8*i+4+7:8*i+4+8,8*(i+1):8*(i+1)+8] = b[0,:]
        
        if i==n-1: 
            
            # end condition of the final curve position and derivatives (4 boundary conditions) 
            b = matrix_generation(1.0)
            m[8*i+4:8*i+4+4,8*i:8*i+8] = b[:4,:]
            
    return m 

def rhs_generation(x):
    """
    Generating the right-hand side of the equation with boundary conditions and 
    intermediary points. All other entries are equal to zero. 
    Assume the drone starts in a complete still position and comes to a 
    complete stop at the end of the path. 
    """
    n= x.shape[0]-1
    
    big_x = np.zeros((8*n))
    big_x[:4] = np.array([x[0],0,0,0]).T
    big_x[-4:] = np.array([x[-1],0,0,0]).T
    
    for i in range(1,n):
        big_x[8*(i-1)+4:8*(i-1)+8+4] = np.array([x[i],0,0,0,0,0,0,x[i]]).T
            
    return big_x

def get_coeff(x,t):
    m = multiple_waypoints(t)
    b = rhs_generation(x)
    
    coeff = coeff = np.linalg.solve(m,b)
    
    return coeff

def fmt(value):
    return "%.3f" % value

def makeMinSnapTraj(filename):
    """
    Extracts waypoints from a trajectory file and converts to a min snap
    trajectory by adding more waypoints and velocity\acceleration info
    INPUT: filename
    OUTPUT: MinSnapTrajectory.txt
    (float) time x y z velx vely velz accx accy accz
    """
    data = np.loadtxt(filename, delimiter=',', dtype='Float64', usecols=(0,1,2,3))
    
    t = data[:,0]
    wpt_x = data[:,1]
    wpt_y = data[:,2]
    wpt_z = data[:,3]
    
    m = multiple_waypoints(t)

    b_x = rhs_generation(wpt_x)
    b_y = rhs_generation(wpt_y)
    b_z = rhs_generation(wpt_z)
    
    coeff_x  = np.linalg.solve(m,b_x)
    coeff_y  = np.linalg.solve(m,b_y)
    coeff_z  = np.linalg.solve(m,b_z)

if __name__ == "__main__":
    filename = 'FigureEightFF.txt'