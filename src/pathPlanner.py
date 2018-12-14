#!/usr/bin/env python

import heapq
import rospy
import math
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker

#size of the drone environment, full map
env_width = 7.0 #meters
env_length = 7.0 #meters

#size of objection, each dot dected
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

######################################

MAP_MAX_X=2 #meters
MAP_MAX_Y=2 #meters
MAP_MIN_X=-2
MAP_MIN_Y=-2
MAP_RESOLUTION=0.1 #grid square size

ROBOT_RADIUS=0.3

OPEN=0
OBSTACLE=1
INFLATION=2
GOAL=3

#round radius up to nearest map resolution tick
ROBOT_RADIUS=ROBOT_RADIUS+MAP_RESOLUTION-ROBOT_RADIUS%MAP_RESOLUTION

class WorldMap:
    def __init__(self):
        self._map = [[0 for y in range(int((MAP_MAX_Y-MAP_MIN_Y)/MAP_RESOLUTION)+1)] for x in range(int((MAP_MAX_X-MAP_MIN_X)/MAP_RESOLUTION)+1)]
        self._goal=None

    def _world_to_map(self,x,y):
        i = round(round(x-MAP_MIN_X,2)/MAP_RESOLUTION)
        j = round(round(y-MAP_MIN_Y,2)/MAP_RESOLUTION)
        if i < 0:
            i=0
        if j < 0:
            j=0
        if i >= len(self._map):
            i = len(self._map)
        if j >= len(self._map[0]):
            j = len(self._map[0])
        return int(i),int(j)

    def _map_to_world(Self, i, j):
        x = round((i*MAP_RESOLUTION)+MAP_MIN_X,2)
        y = round((j*MAP_RESOLUTION)+MAP_MIN_Y,2)
        return x,y

    def _is_open(self, i, j):
        if i<0 or j<0:
            return False
        if i>=len(self._map) or j>=len(self._map[0]):
            return False
        return self._map[i][j]==OPEN or self._map[i][j]==GOAL

    def neighbors(self, cell):
        i, j = self._world_to_map(cell[0], cell[1])
#	print("i,j %s,%s" %(i,j))
        neigh = []
        if self._is_open(i-1,j):
            neigh.append((self._map_to_world(i-1,j)))
        if self._is_open(i+1,j):
            neigh.append((self._map_to_world(i+1,j)))
        if self._is_open(i,j-1):
            neigh.append((self._map_to_world(i,j-1)))
        if self._is_open(i,j+1):
            neigh.append((self._map_to_world(i,j+1)))
        #print(neigh)
        neigh = [(round(x,2), round(y,2)) for x,y in neigh]
 #       print("neigh %s" %str(neigh))
        return neigh

    def dist_to_goal(self, cell):
        if(self._goal == None):
            return -1
        i,j = self._world_to_map(cell[0],cell[1])
        return ((i-self._goal[0])**2 + (j-self._goal[1])**2)**0.5

    def set_feature(self, low, high, status):
        min_i, min_j = self._world_to_map(low[0],low[1])
        max_i, max_j = self._world_to_map(high[0], high[1])
#        if(max_i==min_i):
#            max_i+=1
#        if(max_j==max_j):
#            max_j+=1
        #print("map: %s %s" %(low, high))
        for i in range(min_i, max_i+1):
            for j in range(min_j, max_j+1):
                self._map[i][j] = status
                if status == OBSTACLE:
                    self._inflate_cell(i,j)
                if status == GOAL:
                    self._goal == (i,j)

    def _inflate_cell(self, c_i, c_j):
        radius=int(ROBOT_RADIUS/MAP_RESOLUTION)
        for i in range(max(int(MAP_MIN_X/MAP_RESOLUTION), c_i-radius+1), min(int((MAP_MAX_X-MAP_MIN_X)/MAP_RESOLUTION)+1, c_i+radius)):
            for j in range(max(int(MAP_MIN_X/MAP_RESOLUTION), c_j-radius+1), min(int((MAP_MAX_Y-MAP_MIN_Y)/MAP_RESOLUTION)+1, c_j+radius)):
                if(self._map[i][j] == OPEN):
                    self._map[i][j] = INFLATION

    def inflate(self):
        for x in range(int(MAP_MAX_X/MAP_RESOLUTION)+1-int(MAP_MIN_X/MAP_RESOLUTION)):
            for y in range(int(MAP_MAX_Y/MAP_RESOLUTION)+1-int(MAP_MIN_Y/MAP_RESOLUTION)):
                if(self._map[x][y] == OBSTACLE):
                    self._inflate_cell(x,y)

    def _print_obs_row(self, y):
        print([col[y] for col in self._map])

    def print_obs_map(self):
        for y in range(len(self._map[0])-1, -1, -1):
            self._print_obs_row(y)



def return_area_detected(data):
    global env_length
    global env_width
    global obj_width
    global obj_length
    global currentAreaCoor
    shift_x = 0.3

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


def main():


    world_map = WorldMap()
    world_map.set_feature((-.1,-.1),(0,0), OBSTACLE)
    world_map.set_feature((0.1,0.1),(.4,.4), GOAL)
    #world_map.inflate()
    world_map.print_obs_map()



if __name__ == "__main__":
    main()
    exit(0)
