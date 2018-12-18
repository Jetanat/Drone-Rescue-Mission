#!/usr/bin/env python

import heapq
import rospy
import math
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker

###############################################################################

# AR tag globals

# size of the drone environment, full map
env_width = 7.0 #meters
env_length = 7.0 #meters

# size of object, each dot detected
obj_width = 0.20 #meters
obj_length = 0.20 #meters

#numbers average each five in array
numbers_to_average = 5
lowest_left_x_total = []
lowest_left_y_total = []

highest_right_x_total = []
highest_right_y_total = []

for i in range(0,6):
    lowest_left_x_total.append([])
    lowest_left_y_total.append([])

    highest_right_x_total.append([])
    highest_right_y_total.append([])

#current area coordinate
currentAreaCoor = None
#average area coordinate
averageAreaCoor = None

###############################################################################

# world map globals

MAP_MAX_X=2 # meters
MAP_MAX_Y=2 # meters
MAP_MIN_X=-2
MAP_MIN_Y=-2
MAP_RESOLUTION=0.1 # grid square size (meters/cell)

ROBOT_RADIUS=0.3 # meters

# defines to make easier to read
OPEN=0
OBSTACLE=1
INFLATION=2
GOAL=3

#round radius up to nearest map resolution tick, makes math easier
ROBOT_RADIUS=ROBOT_RADIUS+MAP_RESOLUTION-ROBOT_RADIUS%MAP_RESOLUTION

###############################################################################

# world map

class WorldMap:
    def __init__(self):
        """ init map and goal vars """
        self._map = [
                [0 for y in range(int((MAP_MAX_Y-MAP_MIN_Y)/MAP_RESOLUTION)+1)] 
                for x in range(int((MAP_MAX_X-MAP_MIN_X)/MAP_RESOLUTION)+1)]
        self._goal=None

    def _world_to_map(self,x,y):
        """ convert world x,y to a cell i, j """
        i = round(round(x-MAP_MIN_X,2)/MAP_RESOLUTION)  # round b/c float math
        j = round(round(y-MAP_MIN_Y,2)/MAP_RESOLUTION)  # needs to return int
        
        # if off map return close cell on the map
        if i < 0:
            i=0
        if j < 0:
            j=0
        if i >= len(self._map):
            i = len(self._map)-1
        if j >= len(self._map[0]):
            j = len(self._map[0])-1
        return int(i),int(j)

    def _map_to_world(Self, i, j):
        """ convert world i,j to world x,y """
        x = round((i*MAP_RESOLUTION)+MAP_MIN_X,2)
        y = round((j*MAP_RESOLUTION)+MAP_MIN_Y,2)
        return x,y

    def _is_open(self, i, j):
        """ return bool if map cell isn't obstacle """
        # can't be open if off map
        if i<0 or j<0:
            return False
        if i>=len(self._map) or j>=len(self._map[0]):
            return False

        return self._map[i][j]==OPEN or self._map[i][j]==GOAL

    def neighbors(self, cell):
        """ return list of open neighbor cells """
        i, j = self._world_to_map(cell[0], cell[1])
        neigh = []
        if self._is_open(i-1,j):
            neigh.append((self._map_to_world(i-1,j)))
        if self._is_open(i+1,j):
            neigh.append((self._map_to_world(i+1,j)))
        if self._is_open(i,j-1):
            neigh.append((self._map_to_world(i,j-1)))
        if self._is_open(i,j+1):
            neigh.append((self._map_to_world(i,j+1)))
        # rounding done in map_to_world too but this seems to help
        neigh = [(round(x,2), round(y,2)) for x,y in neigh]
        return neigh

    def dist_to_goal(self, cell):
        # if no goal found yet can't return a dist to it
        if(self._goal == None):  
            return -1
        i,j = self._world_to_map(cell[0],cell[1])
        return ((i-self._goal[0])**2 + (j-self._goal[1])**2)**0.5

    def set_feature(self, low, high, status):
        """
        add feature to map (obstacle/goal
        takes to tuples, bottom left corner of obstacle, and top right
        adds a rectangle to map
        """
        min_i, min_j = self._world_to_map(low[0],low[1])
        max_i, max_j = self._world_to_map(high[0], high[1])

        for i in range(min_i, max_i+1):
            for j in range(min_j, max_j+1):
                self._map[i][j] = status
                if status == OBSTACLE:  # if adding obstacle need to inflate it
                    self._inflate_cell(i,j)
                # update where the map goal is
                # only used for rough heuristic so this is fine
                if status == GOAL:  
                    self._goal == (i,j)

    def _inflate_cell(self, c_i, c_j):
        """ inflates cells around given i,j by the robots radius """
        radius=int(ROBOT_RADIUS/MAP_RESOLUTION)
        # can't inflate off side of map so adjust range accordingly
        for i in range(
                max(int(MAP_MIN_X/MAP_RESOLUTION), c_i-radius+1),
                min(int((MAP_MAX_X-MAP_MIN_X)/MAP_RESOLUTION)+1, c_i+radius)):
            for j in range(
                    max(int(MAP_MIN_X/MAP_RESOLUTION), c_j-radius+1),
                    min(int((MAP_MAX_Y-MAP_MIN_Y)/MAP_RESOLUTION)+1,
                        c_j+radius)):
                # don't inflate over obstacles/goal
                if(self._map[i][j] == OPEN):
                    self._map[i][j] = INFLATION

    def _print_obs_row(self, y):
        """ print yth element of every column """
        print([col[y] for col in self._map])

    def print_obs_map(self):
        """ print map with normal orientation (y up x to the right) """
        for y in range(len(self._map[0])-1, -1, -1):
            self._print_obs_row(y)

###############################################################################

# Getting features from AR tags

def return_area_detected(data):
    global env_length
    global env_width
    global obj_width
    global obj_length
    global currentAreaCoor
    shift_x = 0.0

    #setting ([lowtuple],[hightuple]) coordinate expansions for the drone to move around
    dict_printer = {0:"wall_width", 2:"obstacle_one", 8:"obstacle_two", 9:"obstacle_three",11:"landing",17:"wall_length"}
    dict_param_x_low = {0:-env_width/2 + shift_x, 2:-obj_width/2 + shift_x, 8:-obj_width/2 + shift_x, 9:-obj_width/2 + shift_x,11:0 + shift_x,17:0 + shift_x}
    dict_param_y_low = {0:0, 2:-obj_length/2, 8:-obj_length/2, 9:-obj_length/2,11:0,17:-env_length/2}
    dict_param_x_high = {0:+env_width/2 + shift_x, 2:+obj_width/2 + shift_x, 8:+obj_width/2 + shift_x, 9:+obj_width/2 + shift_x,11:0 + shift_x,17:0 + shift_x}
    dict_param_y_high = {0:0, 2:+obj_length/2, 8:+obj_length/2, 9:+obj_length/2,11:0,17:+env_length/2}

    lowest_left_x = {}
    lowest_left_y = {}
    highest_right_x = {}
    highest_right_y = {}


    signal = []
    for i in range(0,6):
        signal.append(True)


	#check IndexError
    tmp_cache = []
    for i in range(0,6):
        try:
            data.markers[i].id
            tmp_cache.append(data.markers[i].id)
        except IndexError:
            signal[i]=False

    for i in range(0,6):
        if(signal[i]):
            print(dict_printer[data.markers[i].id])
            lowest_left_x[data.markers[i].id] = data.markers[i].pose.pose.position.x+dict_param_x_low[data.markers[i].id]
            lowest_left_y[data.markers[i].id] = data.markers[i].pose.pose.position.y+dict_param_y_low[data.markers[i].id]
            highest_right_x[data.markers[i].id] = data.markers[i].pose.pose.position.x+dict_param_x_high[data.markers[i].id]
            highest_right_y[data.markers[i].id] = data.markers[i].pose.pose.position.y+dict_param_y_high[data.markers[i].id]

    lowest_left_x_new = []
    lowest_left_y_new = []
    highest_right_x_new = []
    highest_right_y_new = []
    for i in [0,2,8,9,11,17]:
    # dict_printer = {0:"wall_width", 2:"obstacle_one", 8:"obstacle_two", 9:"obstacle_three",11:"landing",17:"wall_length"}
        if(i in tmp_cache):
            lowest_left_x_new.append(lowest_left_x[i])
            lowest_left_y_new.append(lowest_left_y[i])
            highest_right_x_new.append(highest_right_x[i])
            highest_right_y_new.append(highest_right_y[i])
        else:
            lowest_left_x_new.append("NO_DETECTION")
            lowest_left_y_new.append("NO_DETECTION")
            highest_right_x_new.append("NO_DETECTION")
            highest_right_y_new.append("NO_DETECTION")

    currentAreaCoor = ([lowest_left_x_new,lowest_left_y_new],[highest_right_x_new,highest_right_y_new])

###############################################################################

# testing code

if __name__ == "__main__":
    world_map = WorldMap()
    world_map.set_feature((-.1,-.1),(0,0), OBSTACLE)
    world_map.set_feature((0.1,0.1),(.4,.4), GOAL)
    world_map.print_obs_map()
    exit(0)

