
# coding: utf-8

# ## Finding Your Way In The City (Graph Edition)
# In this notebook your attention will shift from grids to graphs. At least for search ... 
# 
# Using Voronoi graphs and the medial axis transform we can find paths which maximize safety from obstacles. In addition, graph representation allows further optimizations and more succinct queries.

# In[1]:


# OK this might look a little ugly but...
# need to import the latest version of networkx
# This occassionally fails, so if the next cell 
# doesn't show that you're using networkx 2.1
# please "restart and clear output" from "Kernel" menu
# above and try again.
import sys
#!{sys.executable} -m pip --upgrade pip
get_ipython().system('{sys.executable} -m pip install -I networkx==2.1')
import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx


# In[2]:


nx.__version__


# In[3]:


import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid_and_edges
import numpy.linalg as LA
get_ipython().run_line_magic('matplotlib', 'inline')


# In[4]:


plt.rcParams['figure.figsize'] = 12, 12


# In[5]:


# This is the same obstacle data from the previous lesson.
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)


# Starting and goal positions in *(north, east)*.

# In[6]:


start_ne = (25,  100)
goal_ne = (750., 370.)


# In[7]:


# Static drone altitude (metres)
drone_altitude = 5
safety_distance = 3


# In[8]:


# This is now the routine using Voronoi
grid, edges = create_grid_and_edges(data, drone_altitude, safety_distance)
print(len(edges))


# Plot the edges on top of the grid along with start and goal locations.

# In[9]:


# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower', cmap='Greys') 

for e in edges:
    p1 = e[0]
    p2 = e[1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

    
plt.plot(start_ne[1], start_ne[0], 'rp')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# We now have a graph, well at least visually. The next step is to use the [`networkx`](https://networkx.github.io) to create the graph. **NetworkX** is a popular library handling anything and everything related to graph data structures and algorithms.
# 
# **NOTE:** In the initial import above it was imported with the `nx` alias.
# 
# You're encouraged to read the documentation but here's a super quick tour:
# 
# 1. Create a graph:
# 
# ```
# G = nx.Graph()
# ```
# 
# 2. Add an edge:
# 
# ```
# p1 = (10, 2.2)
# p2 = (50, 40)
# G = nx.add_edge(p1, p2)
# ```
# 
# 3 Add an edge with a weight:
# 
# ```
# p1 = (10, 2.2)
# p2 = (50, 40)
# dist = LA.norm(np.array(p2) - np.array(p1))
# G = nx.add_edge(p1, p2, weight=dist)
# ```

# In[10]:


# TODO: create the graph with the weight of the edges
# set to the Euclidean distance between the points
G = nx.Graph()
for e in edges:
    p1 = tuple(e[0])
    p2 = tuple(e[1])
    dist = LA.norm(np.array(p2) - np.array(p1))
    G.add_edge(p1, p2, weight=dist)
    


# You need a method to search the graph, and you'll adapt A* in order to do this. The notable differences being the actions are now the outgoing edges and the cost of an action is that weight of that edge.

# In[11]:


from queue import PriorityQueue

def heuristic_eucl(n1, n2):
    """Euclidian distance"""
    return LA.norm(np.array(n2) - np.array(n1))

###### THIS IS YOUR OLD GRID-BASED A* IMPLEMENTATION #######
###### With a few minor modifications it can work with graphs! ####
#TODO: modify A* to work with a graph
def a_star_graph(graph, h, start, goal):
    """path, cost = a_star_graph(networkx.Graph(), heuristic_func,
                    tuple(skel_start), tuple(skel_goal))
        INPUT: start, goal = tuple(x,y)
        """
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


# ### Solution
# 
# This solution consists of two parts:
# 
# 1. Find the closest point in the graph to our current location, same thing for the goal location.
# 2. Compute the path from the two points in the graph using the A* algorithm.
# 3. Feel free to use any of the path pruning techniques to make the path even smaller! 
# 4. Plot it up to see the results!

# ### TODO: Write your solution!

# [our solution](/notebooks/Graph-Search-Solution.ipynb)

# In[13]:


def start_goal_graph(G, start, goal):
    """INPUT: (N,E)
    OUTPUT: (N,E)"""
    # TODO: find start and goal on Graph
    # Some useful functions might be:
        # np.nonzero()
        # np.transpose()
        # np.linalg.norm()
        # np.argmin()
    gr_start = point_on_graph(G, start)
    gr_goal = point_on_graph(G, goal)
    return gr_start, gr_goal

def point_on_graph(G, point):
    """Project point onto Graph
    INPUT: G = networkx.Graph()
            point = (x,y)
    OUTPUT: gr_point = (x,y)"""
    if G.has_node(point):
        gr_point = point
    else:
        graph_points = np.array(G.nodes)
        point_addr = np.linalg.norm(np.array(point) - graph_points,
                               axis = 1).argmin()
        gr_point = tuple(graph_points[point_addr])
    
    return gr_point

gr_start, gr_goal = start_goal_graph(G, start_ne, goal_ne)
print(gr_start, gr_goal)

path, cost = a_star_graph(G, heuristic_eucl, gr_start, gr_goal)
print('path = {0}, cost = {1}'.format(len(path), cost))

plt.imshow(grid, origin='lower', cmap='Greys') 

for e in edges:
    p1 = e[0]
    p2 = e[1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

    
plt.plot(start_ne[1], start_ne[0], 'rp')
plt.plot(goal_ne[1], goal_ne[0], 'rx')
plt.plot(gr_start[1], gr_start[0], 'gp')
plt.plot(gr_goal[1], gr_goal[0], 'gx')

pp = np.array(path)
plt.plot(pp[:,1], pp[:,0], 'r-')


plt.xlabel('EAST', fontsize = 18)
plt.ylabel('NORTH', fontsize = 18)
plt.show()

