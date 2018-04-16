#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  1 12:32:56 2018

@author: yudzhi
"""

"""
create a configuration space given a map of the world and setting a particular 
altitude for your drone. You'll read in a .csv file containing obstacle data 
which consists of six columns  x,  y,  z and  δx,  δy,  δz.
You can look at the .csv file here. The first line gives 
the map center coordinates and the file is arranged such that:
x -> NORTH
y -> EAST
z -> ALTITUDE (positive up, note the difference with NED coords)
Each  (x,y,z)
  coordinate is the center of an obstacle.  δx,  δy,  δz
  are the half widths of the obstacles, meaning for example that an obstacle 
  with  (x=37,y=12,z=8)and  (δx=5,δy=5,δz=8)
  is a 10 x 10 m obstacle that is 16 m high and is centered at the point  
  (x,y)=(37,12)at a height of 8 m.
Given a map like this, the free space in the  (x,y)
  plane is a function of altitude, and you can plan a path around an obstacle, 
  or simply fly over it! You'll extend each obstacle by a safety margin to 
  create the equivalent of a 3 dimensional configuration space.
Your task is to extract a 2D grid map at 1 metre resolution of your 
configuration space for a particular altitude, where each value is assigned 
either a 0 or 1 representing feasible or infeasible (obstacle) spaces respectively.
"""
import numpy as np 
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d
import math
from Bresenham import bres

#%matplotlib inline
plt.rcParams["figure.figsize"] = [12, 12]
filename = 'colliders.csv'
# Read in the data skipping the first two lines.  
# Note: the first line contains the latitude and longitude of map center
# Where is this??
data = np.loadtxt(filename,delimiter=',',dtype='Float64',skiprows=2)
#print(data)
# Static drone altitude (metres)
drone_altitude = 15

# Minimum distance required to stay away from an obstacle (metres)
# Think of this as padding around the obstacles.
safe_distance = 3

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))
    #print(north_min)
    #print(north_max)
    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))
    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
#    north_min_center = np.min(data[:, 0])
#    east_min_center = np.min(data[:, 1])
#    print(north_min_center,east_min_center)

    ###########Like this one more##########3
        # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid

# Here you'll modify the `create_grid()` method from a previous exercise
# In this new function you'll record obstacle centres and
# create a Voronoi graph around those points
def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    
    # Define a list to hold Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(north - d_north - safety_distance - north_min_center),
                int(north + d_north + safety_distance - north_min_center),
                int(east - d_east - safety_distance - east_min_center),
                int(east + d_east + safety_distance - east_min_center),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
            
            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)
    voronoi_plot_2d(graph)
    plt.show()
    #print(points)
    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        p1_gr = [int(round(x)) for x in p1]
        p2_gr = [int(round(x)) for x in p2]
        p = [p1_gr,p2_gr]
        #print(p1, p1_grid, p2, p2_grid)
    
        in_collision = True
        if np.amin(p) > 0 and np.amax(p[:][0]) < grid.shape[0] and np.amax(p[:][1]) < grid.shape[1]:
            track = bres(p1_gr,p2_gr)
            for q in track:
                #print(q)
                q = [int(x) for x in q]
                if grid[q[0],q[1]] == 1:
                    in_collision = True
                    break
                else:
                    in_collision = False
        if not in_collision:
            edges.append((p1,p2))

    return grid, edges

    """
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        grid = obstacle_formation(grid, north_min_center, east_min_center,
                                 drone_altitude, safety_distance,
                                 north, east, alt, d_north, d_east,d_alt)
        
        # TODO: Determine which cells contain obstacles
        # and set them to 1.
        #print('cicle',np.nonzero(grid))
    return grid
def obstacle_formation(grid, gr_north, gr_east, 
                       drone_alt, safe_dist,
                       c_north, c_east, c_alt, d_north, d_east, d_alt):
    if (c_alt + d_alt + safe_dist - drone_alt) > 0:
        gr_row = int(np.ceil(c_north - gr_north))
        gr_col = int(np.ceil(c_east - gr_east))
        dN = int(np.ceil(d_north + safe_dist))
        dE = int(np.ceil(d_east + safe_dist))
        #print(gr_row - dN, gr_row+dN, gr_col-dE, gr_col-dE)
        grid[(gr_row - dN):(gr_row + dN), (gr_col-dE):(gr_col+dE)] = 1
        #print(np.nonzero(grid))
    return grid
    """

grid = create_grid(data, drone_altitude, safe_distance)
 # equivalent to
# plt.imshow(np.flip(grid, 0))
# NOTE: we're placing the origin in the lower lefthand corner here
# so that north is up, if you didn't do this north would be positive down
plt.imshow(grid, origin='lower') 

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()