from enum import Enum
from queue import PriorityQueue
import numpy as np
import utm
from scipy.spatial import Voronoi, voronoi_plot_2d
import math
#from Bresenham import bres
from shapely.geometry import Polygon, Point, LineString
from sklearn import neighbors
import networkx as nx


def global_to_local(global_position, global_home):
    """
    To convert a GPS position (longitude, latitude, altitude) to a local position 
    (north, east, down) you need to define a global home position as the origin 
    of your NED coordinate frame. In general this might be the position your 
    vehicle is in when the motors are armed, or some other home base position. 
    Convert from global position to a local position using the utm.
    INPUT: global_position(lon,lat,alt), global_home(lon,lat,alt)
    OUTPUT: [N,E,D]
    """
    
    (east_home, north_home, _, _) = utm.from_latlon(global_home[1], global_home[0])
    
    (east, north, _, _) = utm.from_latlon(global_position[1], global_position[0])
                                          
    local_position = np.array([north - north_home, east - east_home, -(global_position[2] - global_home[2])])
    print(global_position, global_home, local_position)
    return local_position

def local_to_global(local_position, global_home):
    """
    converting a local position (north, east, down) relative to the home position 
    to a global position (long, lat, _up_)
    INPUT: [N,E,D]
    OUTPUT: [Lon,Lat,Alt]
    """
    (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(
                                                        global_home[1], global_home[0])
    
    (lat, lon) = utm.to_latlon(east_home + local_position[1],
                               north_home + local_position[0], zone_number,
                               zone_letter)
                               
    global_position = np.array([lon, lat, -(local_position[2]-global_home[2])])
    
    return global_position

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

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

    return grid, int(north_min), int(east_min)

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

# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.
    
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))
    
    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
    
    @property
    def cost(self):
        return self.value[2]
    
    @property
    def delta(self):
        return (self.value[0], self.value[1])
    
def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node
    
    # check if the node is off the grid or
    # it's an obstacle
    
    if x - 1 < 0 or grid[x-1, y] == 1:
        valid.remove(Action.NORTH)
    if x + 1 > n or grid[x+1, y] == 1:
        valid.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y-1] == 1:
        valid.remove(Action.WEST)
    if y + 1 > m or grid[x, y+1] == 1:
        valid.remove(Action.EAST)
        
    if (x - 1 < 0 or y - 1 < 0) or grid[x-1,y-1] == 1:
        valid.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x-1,y+1] == 1:
        valid.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x+1,y-1] == 1:
        valid.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x+1,y+1] == 1:
        valid.remove(Action.SOUTH_EAST)
        
    return valid


def a_star(grid, h, start, goal):

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
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
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

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def manh_heuristic(position, goal_position):
    # TODO: define a heuristic
    h = abs(position[0] - goal_position[0]) + abs(position[1] - goal_position[1])
    return h

def eucl_heuristic(position, goal_position):
    h2 = np.sqrt((position[0] - goal_position[0])**2 + (position[1] - goal_position[1])**2)
    return h2

class Sampler():
    
    def __init__(self, data, zlim = 10, safety_distance = 1):
        self._zmax = zlim
        self._polygons = self.extract_polygons(data, safety_distance)
        self.__d = data
        self.NElimits(data)
        
    @property
    def _zmax(self):
        return self.__zmax
    
    @_zmax.setter
    def _zmax(self,zlim):
        if zlim < 0:
            self.__zmax = 0
        else:
            self.__zmax = zlim
            
    def NElimits(self,data):
        self._nmin = self.datalimits(data)[0]
        self._nmax = self.datalimits(data)[1]
        
        self._emin = self.datalimits(data)[2]
        self._emax = self.datalimits(data)[3]
        self._zmin = 0
        
    @staticmethod
    def datalimits(data):
        """
        set data borders
        Input: data
        Output: (nmin, nmax, emin, emax)
        """
        nmin = np.min(data[:, 0] - data[:, 3])
        nmax = np.max(data[:, 0] + data[:, 3])
        emin = np.min(data[:, 1] - data[:, 4])
        emax = np.max(data[:, 1] + data[:, 4])
        return nmin, nmax, emin, emax
        
    def sample(self,num_samp):
        """
        sampling points and removing
        ones conflicting with obstacles.
        """
        nodes = self.random_sample(self.__d, self._zmax, num_samp, False)
        tree = self.KDTree_from_poly(self._polygons)
        to_keep_tree = []
        for point in nodes:
            if not self.collides_tree(tree, self._polygons, point):
                to_keep_tree.append(point)
                
        return to_keep_tree

    
    @staticmethod
    def collides_tree(tree, polygons, point):  
        """
        Determine whether the point collides with any obstacles
        Input: KDTree, polygons, random_node
        Output: True or False
        """
        dist,ind = tree.query(np.asarray(point[:2]).reshape(1,2),k=3)
        collision = False
        for j in range(ind.shape[1]):
            pnum = ind[0][j]
            (p,height) = polygons[pnum].poly
            if p.contains(Point(point)) and height >= point[2]:
                collision = True
                break
        return collision


    @staticmethod
    def KDTree_from_poly(polygons, debug = False):
        center = [np.asarray(pol.p.centroid) for pol in polygons]
        if debug:
            print(center[:10], np.asarray(center).shape)
        tree = neighbors.KDTree(center,leaf_size = 40)
        return tree

            

    @staticmethod
    def random_sample(data, z_lim, num_samples = 200, explicit = True):
    # # Sampling 3D Points
    # 
    # Now that we have the extracted the polygons, we need to sample random 3D points. 
    #Currently we don't know suitable ranges for x, y, and z. 
    #Let's figure out the max and min values for each dimension.
    
        nmin = np.min(data[:, 0] - data[:, 3])
        nmax = np.max(data[:, 0] + data[:, 3])
        
        emin = np.min(data[:, 1] - data[:, 4])
        emax = np.max(data[:, 1] + data[:, 4])
        
        zmin = 1
        # Limit the z axis for the visualization
        zmax = z_lim #np.max(data[:,2] + data[:,5] + 10) #10
        
        if explicit:
            print("N")
            print("min = {0}, max = {1}\n".format(nmin, nmax))
            
            print("E")
            print("min = {0}, max = {1}\n".format(emin, emax))
            
            print("Z")
            print("min = {0}, max = {1}".format(zmin, zmax))
            print()
            
        # Next, it's time to sample points. All that's left is picking the 
        #distribution and number of samples. The uniform distribution makes 
        #sense in this situation since we we'd like to encourage searching the whole space.
            
        #np.random.seed(0)
        nvals = np.random.uniform(nmin, nmax, num_samples)
        evals = np.random.uniform(emin, emax, num_samples)
        zvals = np.random.uniform(zmin, zmax, num_samples)
        
        return list(zip(nvals, evals, zvals))


    @staticmethod
    def extract_polygons(data, sdist = 0):
        """Polygons with or without safety_distance"""
        polygons = []
        for i in range(data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = data[i, :]
            
            # TODO: Extract the 4 corners of the obstacle
            # 
            # NOTE: The order of the points matters since
            # `shapely` draws the sequentially from point to point.
            #
            # If the area of the polygon is 0 you've likely got a weird
            # order.
            p1 = (north + d_north + sdist, east - d_east - sdist)
            p2 = (north + d_north + sdist, east + d_east + sdist)
            p3 = (north - d_north - sdist, east + d_east + sdist)
            p4 = (north - d_north - sdist, east - d_east - sdist)
            corners = [p1, p2, p3, p4]
            
            # TODO: Compute the height of the polygon
            height = alt + d_alt + sdist
    
            # TODO: Once you've defined corners, define polygons
            polygons.append(Prism(corners, height))
    
        return polygons



# ## Create Polygons
class Prism():
    """
    3D structure Prism = 2D Polygon + height
    """
    
    def __init__(self, corners, height):
        self.p = Polygon(corners)
        self.height = height
        
        self.poly = (self.p, self.height)
        
    def __str__(self):
        return '(' + str(self.p) + ',' + str(self.height) + ')'
    
    def crosses(self, line):
        """
        shapely geometry objects have a method .crosses which return 
        True if the geometries cross paths.
        Input: line (from shapely.geometry import LineString)
                or list(tuple1, tuple2, tuple3...)
                or (tuple1, tuple2,...)
        if points = [tuple1, tuple]
        """
        #print('crosses', line, type(line))
        if not type(line) == LineString:
            #print(line, type(line))
            line = LineString(list(line))
        #coords = list(zip(*line)) #[(x1,x2),(y1,y2),(z1,z2)]
        return self.p.crosses(line)
    
    def intersects(self,line):
        #print('crosses', line, type(line))
        if not type(line) == LineString:
            #print(line, type(line))
            line = LineString(list(line))
        #coords = list(zip(*line)) #[(x1,x2),(y1,y2),(z1,z2)]
        return self.p.intersects(line)
        
    def touches(self,line):
        #print('crosses', line, type(line))
        if not type(line) == LineString:
            #print(line, type(line))
            line = LineString(list(line))
        #coords = list(zip(*line)) #[(x1,x2),(y1,y2),(z1,z2)]
        return self.p.touches(line)
    
    def bounds(self):
        """
        Returns a (minx, miny, maxx, maxy) tuple (float values) that bounds the object
        """
        return self.p.bounds
    
    def intersection(self, line):
        """
        Returns a representation of the intersection of this object with the other geometric object.
        """
        if not type(line) == LineString:
            line = LineString(list(line))
        return self.p.intersection(line)

def bres(p1, p2, conservative = True): 
    """
    Extended Bresenham method for any p1 and p2
    """
    x1, y1 = p1
    x2, y2 = p2
    # First, set dx = x2 - x1 and dy = y2 - y1
    dx, dy = x2 - x1, y2 - y1
    #print('dx',dx)
    try:
        x_st = dx/abs(dx)
    except ZeroDivisionError:
        x_move = 0
        y_move = 1
        x_st = 1
    #Creepy Jupyter gets nan when dividing 0.0/0.0 in try block
    if math.isnan(x_st):
        x_move = 0
        y_move = 1
        x_st = 1
    try:
        y_st = dy/abs(dy)
    except ZeroDivisionError:
        y_move = 0
        x_move = 1
        y_st = 1
    if math.isnan(y_st):
        y_move = 0
        x_move = 1
        y_st = 1
        
    cells = []
    # TODO: Determine valid grid cells

    try:
        m = (y2 - y1)/(x2 - x1) #slope
        b = y2 - m * x2
        s = dx/abs(dx) #sign to multipy without replacing < with >
    except ZeroDivisionError:
        b = 0
        s = 1
    if math.isnan(s):
        b = 0
        s = 1
    # The condition we care about is whether 
    # (x + x_step) * m  + b < y + y_step
    # (x + x_step) dy / dx < y + y_step - b 
    # which implies (dx < 0 case included): 
    # s *(x dy - y dx) < s *(y_st*dx - x_st*dy -b*dx)
    # Then define a new quantity: d = x dy - y dx
    # new condition: s*d < s*(y_st*dx - x_st*dy - b*dx)
    # and set d = 0 initially    
    d = x1 * dy - y1 * dx
    # Initialize i, j indices
    i = x1
    j = y1    
    while abs(i-x1) <= abs(dx) and abs(j-y1) <= abs(dy):  
#        print('x,y',(i,j), abs(i-x1), abs(dx),abs(j-y1), abs(dy))
        cells.append([i,j])
#        print('cells',cells)
        if dx == 0 or dy == 0:
            cells.append([i - x_st*y_move, j - y_st*x_move])
        elif s*d < s*(y_st * dx - x_st * dy - b * dx):
            #(x+1)m<y+1      (x+1)m=y+1   (x+1)m>y+1, m > 0
            #|----------|    |-----|      |--|     
            #|          |dy  |     |dy    |  |dy
            #|----------|    |-----|      |--|
            #  dx              dx          dx 
            
            #(x+1)m+b < y+1 => __/ x += 1, dy>0
            #OR 
            #(x-1)m+b < y-1 => y -= 1, dy<0
#            print('<')
            x_move = (abs(dy) + dy)//(2 * abs(dy)) #1 in case dy>0
            y_move = (abs(dy) - dy)//(2 * abs(dy)) #1 in case dy<0
        elif s*d > s*(y_st * dx - x_st * dy - b * dx):
            #(x+1)m+b > y+1 => __/ y += 1, dy>0
            #OR 
            #(x-1)m+b > y-1 => x -= 1, dy<0
#            print('>')
            x_move = (abs(dy) - dy)//(2 * abs(dy))
            y_move = (abs(dy) + dy)//(2 * abs(dy))
            #print('ij',i,j,'xmv,ymv',x_move,y_move,'ix1,jy1',i-x1,j-y1) 
        elif s*d == s*(y_st * dx - x_st * dy - b * dx): 
            # uncomment these two lines for conservative approach
            if conservative:
                cells.append([i + x_st, j])
                cells.append([i, j + y_st])
#            print('=',s*d,s*(- b))
            x_move = 1
            y_move = 1
        else:
            x_move = 0
            y_move = 0
        i += x_st * x_move 
        j += y_st * y_move 
        d += x_st*x_move*dy - y_st*y_move*dx
    return np.array(cells)

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
        print(graph_points.shape, type(graph_points[1]),graph_points[1],'p2',point[2])
        if graph_points.shape[1] < 3:
            graph_points_3D = np.zeros((graph_points.shape[0],3))
            for ind in range(graph_points.shape[0]):
                #print(graph_points[ind],ind, point)
                graph_points_3D[ind] = np.append(graph_points[ind], point[2])
            #print('pointongraph', graph_points.shape, point)
            point_addr = np.linalg.norm(np.array(point) - graph_points_3D,
                               axis = 1).argmin()
            gr_point = tuple(graph_points_3D[point_addr])
            
            return gr_point[0:2]
        
        else: 
            point_addr = np.linalg.norm(np.array(point) - graph_points,axis = 1).argmin()
            gr_point = tuple(graph_points[point_addr])
        
            return gr_point

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-5):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    if path is not None:
        pruned_path = [p for p in path]
        i = 0        
        while i < len(pruned_path) - 2:
            p1 = point(pruned_path[i])
            p2 = point(pruned_path[i+1])
            p3 = point(pruned_path[i+2])
            if collinearity_check(p1,p2,p3):
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
        # TODO: prune the path!
    else:
        pruned_path = path
        
    return pruned_path

# TODO: Your start and goal location defined above
# will not necessarily be on the skeleton so you
# must first identify the nearest cell on the 
# skeleton to start and goal

def find_start_goal_skeleton(skel, start, goal):
    # TODO: find start and goal on skeleton
    # Some useful functions might be:
        # np.nonzero()
        # np.transpose()
        # np.linalg.norm()
        # np.argmin()
    skel_points = np.transpose(skel.nonzero())
    start_addr = np.linalg.norm(np.array(start) - skel_points,
                               axis = 1).argmin()
    near_start = skel_points[start_addr]
    goal_addr = np.linalg.norm(np.array(goal) - skel_points,
                              axis = 1).argmin()
    near_goal = skel_points[goal_addr]
    return near_start, near_goal

# TODO: connect nodes
# Suggested method
    # 1) cast nodes into a graph called "g" using networkx

    # 2) write a method "can_connect()" that:
        # casts two points as a shapely LineString() object
        # tests for collision with a shapely Polygon() object
        # returns True if connection is possible, False otherwise
def can_connect(p1,p2, polygons):
    line = LineString([tuple(p1),tuple(p2)])
    for p in polygons:
        #print(type(line), 'canconnect', line)
        if p.crosses(line) and p.height >= min(p1[2], p2[2]):
            return False
    return True

# 3) write a method "create_graph()" that:
    # defines a networkx graph as g = Graph()
    # defines a tree = KDTree(nodes)
    # test for connectivity between each node and 
        # k of it's nearest neighbors
    # if nodes are connectable, add an edge to graph
def create_graph(nodes,k, polygons):
    g = nx.Graph()
    tree = neighbors.KDTree(nodes)
    for n1 in nodes:
        ind = tree.query([n1], k, return_distance = False)[0]
        #print(ind)
        for j in ind:
            n2 = nodes[j]
            #print(n2)
            #print(j)
            if n2 == n1:
                continue
            #print('n1, n2',n1,n2)    
            if can_connect(n1, n2, polygons):
                dist = np.linalg.norm(np.array(n1) - np.array(n2))
                g.add_edge(n1, n2, weight = dist)
    return g

