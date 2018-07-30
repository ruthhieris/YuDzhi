import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star_graph, heuristic, global_to_local, create_grid_and_edges, create_grid, a_star
from planning_utils import Sampler, start_goal_graph, prune_path
from planning_utils import eucl_heuristic, manh_heuristic
from planning_utils import find_start_goal_skeleton
from planning_utils import create_graph
from udacidrone import Drone
from udacidrone.connection import CrazyflieConnection #MavlinkConnection
from udacidrone.messaging import MsgID
import networkx as nx
from skimage.morphology import medial_axis
from skimage.util import invert
#from udacidrone.frame_utils import global_to_local
#import matplotlib.pyplot as plt

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

class SearchAlg(Enum):
    VORONOI = auto()
    MEDIAL_AXIS = auto()
    PROBABILISTIC = auto()

class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.MANUAL:
            self.plan_path()
        elif self.flight_state == States.PLANNING:
            self.takeoff_transition()
        if self.flight_state == States.TAKEOFF:
            try:
                if len(self.waypoints) < 1.0:
                    print("didn't make it")
                    self.landing_transition()
                elif -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                    self.waypoint_transition()
            except TypeError:
                print(self.waypoints, type(self.waypoints))
                self.landing_transition()
#            else:
#                self.landing_transition()
        elif self.flight_state == States.WAYPOINT:
            #print('lenwpt',len(self.waypoints), 'targ',self.target_position,'loc',self.local_position)
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 0.2:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 0.5:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if abs(self.local_position[2] < 0.01):
                self.manual_transition()

#    def velocity_callback(self):
#        if self.flight_state == States.LANDING:
#            if self.global_position[2] - self.global_home[2] < 0.1:
#                if abs(self.local_position[2]) < 0.01:
#                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        print("arming transition")
        self.arm()
        self.take_control()
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition", self.target_position[2])
        #self.takeoff(3.0)
        self.takeoff(self.target_position[2])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], 
                          self.target_position[2], self.target_position[3])        
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def read_home_position(self, filename):
        """
        INPUT: data file
        OUTPUT: lon0, lat0, alt0
        """
        # TODO: read lat0, lon0 from colliders into floating point values
        #filename = 'colliders.csv'
        f = open(filename)
        line = f.readline()[:-1].split(',')
        lat0, lon0 = [float(l.split(' ')[-1]) for l in line]
        f.close()
        alt0 = 0
        #data0 = np.loadtxt(filename,skiprows = -1, usecols = (1,3) ,converters = 
                          #{1:lambda s: np.char.strip(np.compat.asstr(s),',')})
        return lon0, lat0, alt0

    def calculate_box(self):
        cp = self.local_position
        cp[2] = 0
        local_waypoints = [cp + [0.5, 0.0, 0.5], cp + [0.5, 0.5, 0.5], 
                           cp + [0.0, 0.5, 0.5], cp + [0.0, 0.0, 0.5]]
        return local_waypoints
    
    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 0.5
        SAFETY_DISTANCE = 3
        self.target_position[2] = TARGET_ALTITUDE
        # Set self.waypoints
        self.waypoints = self.choose_path_source(self.calculate_box(), TARGET_ALTITUDE, SAFETY_DISTANCE) #'colliders.csv'
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        #self.send_waypoints()
        
    def choose_path_source(self,path_source, TARGET_ALTITUDE, SAFETY_DISTANCE):
        if isinstance(path_source, str):
            print('str')
            filename = path_source

            #VORONOI, MEDIAL_AXIS, PROBABILISTIC
            self.search_alg = SearchAlg.VORONOI
            # TODO: set home position to (lon0, lat0, 0)
            lon0, lat0, alt0 = self.read_home_position(filename)
            self.set_home_position(lon0, lat0, alt0)        
    
            # TODO: retrieve current global position
            #cur_gl_lon, cur_gl_lat, cur_gl_alt = self.global_position()
            # TODO: convert to current local position using global_to_local()
            local_position = global_to_local(self.global_position, self.global_home)
            
            print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                             self.local_position))
              
            # Read in obstacle map
            data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
    
            # Define starting point on the grid (this is just grid center)
            #grid_start = (-north_offset, -east_offset)
            # TODO: convert start position to current position rather than map center
            
            start_ne = [(local_position[0],local_position[1], TARGET_ALTITUDE)]
            #start_ne = Sampler.random_sample(data, flight_altitude, 1)
            # Set goal as some arbitrary position on the grid
            #grid_goal = (-north_offset + 10, -east_offset + 10)
            #Define a start and goal location   
            
            # TODO: adapt to set goal as latitude / longitude position and convert
            #Modify this to be set as some arbitrary position on the grid given 
            #any geodetic coordinates (latitude, longitude)
            #goal_ne = [(start_ne[0][0]+30,start_ne[0][1]+1,start_ne[0][2])]
            goal_ne = Sampler.random_sample(data, TARGET_ALTITUDE, 1,False)
            #print("RandomStart", start_ne, "RandomGoal", goal_ne)
            
            
            #place random start and goal points on graph
            north_min = Sampler.datalimits(data)[0]
            east_min = Sampler.datalimits(data)[2]
            #print('stne', start_ne, 'nmin',north_min, 'e',east_min,'gne',goal_ne)
            start_v = (start_ne[0][0] - north_min, start_ne[0][1] - east_min, start_ne[0][2])
            goal_v = (goal_ne[0][0] - north_min, goal_ne[0][1] - east_min, goal_ne[0][2])
            print("Start on plot {0}, Goal on plot {1}".format(start_v, goal_v))
            waypoints = self.find_path(data, TARGET_ALTITUDE, SAFETY_DISTANCE, 
                                  start_v, goal_v, north_min, east_min)
            # Convert path to waypoints
            print("North min = {0}, east min = {1}".format(north_min, east_min))
    
            print('len_wpt', len(waypoints), waypoints[0:2])
        else:
            print('boxcalc')
            waypoints = path_source
            for j in range(len(waypoints)):
                waypoints[j] = [waypoints[j][0], waypoints[j][1], waypoints[j][2], 0]
            print(len(waypoints),waypoints)
            
        return waypoints


    def find_path(self, data, TARGET_ALTITUDE, SAFETY_DISTANCE, start_v, goal_v, nmin, emin):
        """
        Path search with different algorithms
        INPUT: search_alg = [VORONOI, MEDIAL_AXIS]
        OUTPUT: path
        """
        if self.search_alg == SearchAlg.VORONOI:
            # Define a grid for a particular altitude and safety margin around obstacles
            #grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            #print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
            import time
            t0 = time.time()
            grid, edges = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            print('Found %5d edges' % len(edges))
            #print(flight_altitude, safety_distance)
            print('graph took {0} seconds to build'.format(time.time()-t0))
            
            # create the graph with the weight of the edges
            # set to the Euclidean distance between the points
            G = nx.Graph()
            for e in edges:
                p1 = tuple(e[0])
                p2 = tuple(e[1])
                dist = np.linalg.norm(np.array(p2) - np.array(p1))
                G.add_edge(p1, p2, weight=dist)

            graph_start, graph_goal = start_goal_graph(G, start_v, goal_v)
            
            # Run A* to find a path from start to goal
            # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
            # or move to a different search space such as a graph (not done here)
            print('Local Start and Goal on Voronoi graph: ', graph_start, graph_goal)
            
            path_graph, cost = a_star_graph(G, eucl_heuristic, graph_start, graph_goal)
            print('Number of edges',len(path_graph), 'Cost', cost)
            # TODO: prune path to minimize number of waypoints
            # TODO (if you're feeling ambitious): Try a different approach altogether!
            waypoints = [[int(p[0] + nmin), int(p[1] + emin), TARGET_ALTITUDE, 0] for p in path_graph]
            
            return waypoints
        
        elif self.search_alg == SearchAlg.MEDIAL_AXIS:
            
            # Define a grid for a particular altitude and safety margin around obstacles
            grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
            # Define starting point on the grid (this is just grid center)
            #grid_start = (-north_offset, -east_offset)
            #grid_start =(int(start_v[0][0]),int(start_v[0][1]))
            # TODO: convert start position to current position rather than map center
            
            # Set goal as some arbitrary position on the grid
            #grid_goal = (-north_offset + 10, -east_offset + 10)
            #grid_goal = (int(goal_v[0][0]),int(goal_v[0][1]))
            # TODO: adapt to set goal as latitude / longitude position and convert
    
            skeleton = medial_axis(invert(grid))
            #print(skeleton.shape)
          
            skel_start, skel_goal = find_start_goal_skeleton(skeleton, start_v[0:2], goal_v[0:2])
            # Run A* to find a path from start to goal
            # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
            # or move to a different search space such as a graph (not done here)
            print('Local Start and Goal on Skeleton: ', skel_start, skel_goal)
            #Manhattan heuristic
            #print(invert(skeleton).astype(np.int))
            path, cost = a_star(invert(skeleton).astype(np.int), manh_heuristic,
                                tuple(skel_start), tuple(skel_goal))
            # TODO: prune path to minimize number of waypoints
            pruned_path = prune_path(path)
            # TODO (if you're feeling ambitious): Try a different approach altogether!
            waypoints = [[int(p[0] + nmin), int(p[1] + emin), TARGET_ALTITUDE, 0] for p in pruned_path]
            
            return waypoints
        
        elif self.search_alg == SearchAlg.PROBABILISTIC:
            
            num_samp = 1000
            sampler = Sampler(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

            start = (start_v[0] + nmin, start_v[1] + emin, start_v[2])
            goal = (goal_v[0] + nmin, goal_v[1] + emin, goal_v[2])
            polygons = sampler._polygons
            nodes = sampler.sample(num_samp)
            #for ind in range(len(nodes)):
               # nodes[ind] = (nodes[ind][0] - nmin, nodes[ind][1] - emin, nodes[ind][2])
            print('number of nodes', len(nodes),'local NE' , nodes[1])
            nearest_neighbors = 10
            import time
            t0 = time.time()
            g = create_graph(nodes, nearest_neighbors, polygons)
            print('graph took {0} seconds to build'.format(time.time()-t0))
            print("Number of edges", len(g.edges))
            grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
            
            prob_start, prob_goal = start_goal_graph(g, start, goal)

            print('Local Start and Goal on Probabilistic map: ', prob_start, prob_goal)
            
            path, cost = a_star_graph(g, heuristic, prob_start, prob_goal)
            #path_pairs = zip(path[:-1], path[1:])    
            
            
            """
            plt.imshow(grid, cmap='Greys', origin='lower')
            
            nmin = np.min(data[:, 0])
            emin = np.min(data[:, 1])
            
            # draw nodes
            for n1 in g.nodes:
                plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
                
            # draw edges
            for (n1, n2) in g.edges:
                plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'grey')
                
            # TODO: add code to visualize the path
            path_pairs = zip(path[:-1], path[1:])
            for (n1, n2) in path_pairs:
                plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green')
            plt.plot(start[1]-emin, start[0]-nmin, 'bv')
            plt.plot(goal[1]-emin, goal[0]-nmin, 'bx')
            plt.xlabel('NORTH')
            plt.ylabel('EAST')
            
            plt.show()
            """
            waypoints = [[int(p[0]), int(p[1]), int(p[2]), 0] for p in path]
            
            return waypoints


    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    #conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    conn = CrazyflieConnection('radio://0/80/2M')
    drone = MotionPlanning(conn)
    time.sleep(2)

    drone.start()
