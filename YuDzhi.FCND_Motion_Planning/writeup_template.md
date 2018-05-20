## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...
**plan_path()** method:
* setting home position
* retrieving current global position and converting it to current local position
* reading obstacle data
* creating grid and placing *start* - *goal* points on it
* A* path search implementation and subsequent wqypoints number minimizing

**planning_utils.py** library includes:
* `create_grid()` - grid representation of a 2D config space based on given obstacle data, drone alt and safety dist
* `Action` class - valid actions enumeration when moving through the grid
* `A-star` algorithm (2D, for grid representation) 
* `heuristic` function - for cost estimation in a-star

And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

### Implementing Your Path Planning Algorithm
`plan_path` function can use several algorithms:
```python
class SearchAlg(Enum):
    VORONOI = auto()
    MEDIAL_AXIS = auto()
    PROBABILISTIC = auto()

def plan_path(self):
    self.flight_state = States.PLANNING
    print("Searching for a path ...")
    TARGET_ALTITUDE = 5
    SAFETY_DISTANCE = 3
    filename = 'colliders.csv'
    self.target_position[2] = TARGET_ALTITUDE
    #VORONOI, MEDIAL_AXIS, PROBABILISTIC
    self.search_alg = SearchAlg.VORONOI
    ```

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the 
self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.
`def read_home_position(self,filename):`
1. Open data-file
2. Read and split the first line without \n: `line = f.readline()[:-1].split(",")`
3. Split each of two peaces to get float position values: `lat0, lon0 = [float(l.split(" ")[-1]) for l in line]`
4. Close file. 
5. return lon0, lat0, alt0
Could be done with *pandas*, but this package isn't included in FCND-environment.

        self.set_home_position(self.read_home_position(filename))       

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

1. Retrieve current global position `cur_gl_lon, cur_gl_lat, cur_gl_alt = self.global_position()`
2. Use `utm.from_latlon` in utility method global_to_local(global_position, global_home)
3. Convert Gl to Loc position `self.local_position = global_to_local(self.global_position, self.global_home)`

#### 3. Set grid start position from local position
1. for Voronoi graph algorithm:
    1. Create grid along with Voronoi graph and use Bresenham to check edges for collisions
    ```python
        def create_grid_and_edges(data, drone_altitude, safety_distance):
            """
            Returns a grid representation of a 2D configuration space
            along with Voronoi graph edges given obstacle data and the
            drone's altitude.
            """
    ```
    2. Create the graph with the weight of the edges set to the Euclidean distance between the points
    ```python    
    	G = nx.Graph()
            for e in edges:
                p1 = tuple(e[0])
                p2 = tuple(e[1])
                dist = np.linalg.norm(np.array(p2) - np.array(p1))
                G.add_edge(p1, p2, weight=dist)
    ```
    3. Convert NED local position to grid NEZ position

        start_ne = [self.local_position[0],self.local_position[1],-self.local_position[2]]
2. for Medial_Axis algorithm:
    1. Create skeleton:
    ```python
    skeleton = medial_axis(invert(grid))
    ```
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map 
and have it rendered to a goal location on the grid.
*class Sample()* (extracts polygons from obstacle data) includes 
1. static method *random_sample* to sample 3D random Points
	
    ```python
	goal_ne = Sampler.random_sample(data, TARGET_ALTITUDE, 1,False)
    ```

2. place random start and goal points on graph:
    1. Voronoi:

    ```python
    	north_min = Sampler.datalimits(data)[0]
            east_min = Sampler.datalimits(data)[2]
    	start_v = (start_ne[0][0] - north_min, start_ne[0][1] - east_min)
            goal_v = (goal_ne[0][0] - north_min, goal_ne[0][1] - east_min)
            gr_start, gr_goal = start_goal_graph(G, start_v, goal_v)
            ```

    2. Medial_Axis:

    ```python
        skel_start, skel_goal = find_start_goal_skeleton(skeleton, start_v, goal_v)
        ```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

1. Voronoi:

```python

    def a_star_graph(graph, h, start, goal):
        """
        path, cost = a_star_graph(networkx.Graph(), heuristic_func,
                        tuple(skel_start), tuple(skel_goal))
        INPUT: start, goal = tuple(x,y)
        """
        ```

2. Medial_Axis:

```python
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

#Manhattan heuristic
#print(invert(skeleton).astype(np.int))
path, cost = a_star(invert(skeleton).astype(np.int), heuristic_func,
                    tuple(skel_start), tuple(skel_goal))
```

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

1. In Voronoi-graph case already used Bresenham when checking edges for collisions
```python
            def point(p):
            return np.array([p[0], p[1], 1.]).reshape(1, -1)

        def collinearity_check(p1, p2, p3, epsilon=1e-6):   
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
```


### Execute the flight
#### 1. Does it work?
Modified `local_posistion_callback` - add case `Failed to find a path`:
```python    
        def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if len(self.waypoints) < 1.0:
                print("didn't make it")
                self.landing_transition()
            elif -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
```

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.

And here is a lovely picture of Probabilistic San Francisco environment from above!
![Probabilistic_gr](./planning_im/im_Prob_graph_time.png)
![Probabilistic_m](./planning_im/im_Prob_map.png)

In comparison with Voronoi!
![Voronoi_gr](./planning_im/im_Vor_graph_time.png)
![Voronoi_m](./planning_im/im_Vor_map.png)
