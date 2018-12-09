#!/usr/bin/env python

import heapq
import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker

#size of the drone environment, full map
env_width = 10.0 #meters
env_length = 10.0 #meters

#size of objection, each dot dected
obj_width = 0.10 #meters
obj_length = 0.10 #meters

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
MAP_MAX_Y=1 #meters
MAP_RESOLUTION=0.1 #grid square size

ROBOT_RADIUS=0.2

OPEN=0
OBSTACLE=1
INFLATION=2
GOAL=3

#round radius up to nearest map resolution tick
ROBOT_RADIUS=ROBOT_RADIUS+MAP_RESOLUTION-ROBOT_RADIUS%MAP_RESOLUTION

class WorldMap:
    def __init__(self):
        self._map = [[0 for y in range(int(MAP_MAX_Y/MAP_RESOLUTION)+1)] for x in range(int(MAP_MAX_X/MAP_RESOLUTION)+1)]
        self._goal=None

    def _world_to_map(self,x,y):
        i = int((x-x%MAP_RESOLUTION)/MAP_RESOLUTION)
        j = int((y-y%MAP_RESOLUTION)/MAP_RESOLUTION)
        return i,j

    def _map_to_world(Self, i, j):
        x = i*MAP_RESOLUTION
        y = j*MAP_RESOLUTION
        return x,y

    def _is_open(self, i, j):
        return self._map[i][j]==OPEN or self._map[i][j]==GOAL

    def neighbors(self, cell):
        i, j = self._world_to_map(cell[0], cell[1])
        neigh = []
        if self._is_open(i-1,j):
            neigh.append([self._map_to_world(i-1,j)])
        if self._is_open(i+1,j):
            neigh.append([self._map_to_world(i+1,j)])
        if self._is_open(i,j-1):
            neigh.append([self._map_to_world(i,j-1)])
        if self._is_open(i,j+1):
            neigh.append([self._map_to_world(i,j+1)])
        return neigh

    def dist_to_goal(self, cell):
        if(self.goal == None):
            return -1
        i,j = self._world_to_map(cell[0],cell[1])
        return ((i-self._goal[0])**2 + (j-self._goal[1])**2)**0.5

    def set_feature(self, low, high, status):
        #round input (x,y) to map resolution
        #round down min
        min_i, min_j = self._world_to_map(low[0],low[1])

        #round up max
        max_i, max_j = self._world_to_map(high[0], high[1])
        if(max_i==min_i):
            max_i+=1
        if(max_j==max_j):
            max_j+=1
        #max_x=int((high[0]+(MAP_RESOLUTION-high[0]%MAP_RESOLUTION)+MAP_RESOLUTION)/MAP_RESOLUTION)
        #max_y=int((high[1]+(MAP_RESOLUTION-high[1]%MAP_RESOLUTION)+MAP_RESOLUTION)/MAP_RESOLUTION)
        for i in range(min_i, max_i):
            for j in range(min_j, max_j):
                self._map[i][j] = status
                if status == OBSTACLE:
                    self._inflate_cell(i,j)

    def _inflate_cell(self, c_i, c_j):
        radius=int(ROBOT_RADIUS/MAP_RESOLUTION)
        for i in range(max(0, c_i-radius), min(int(MAP_MAX_X/MAP_RESOLUTION)+1, c_i+radius+1)):
            for j in range(max(0, c_j-radius), min(int(MAP_MAX_Y/MAP_RESOLUTION)+1, c_j+radius+1)):
                if(self._map[i][j] == OPEN):
                    self._map[i][j] = INFLATION

    def inflate(self):
        for x in range(int(MAP_MAX_X/MAP_RESOLUTION)+1):
            for y in range(int(MAP_MAX_Y/MAP_RESOLUTION)+1):
                if(self._map[x][y] == OBSTACLE):
                    self._inflate_cell(x,y)

    def _print_obs_row(self, y):
        print([col[y] for col in self._map])

    def print_obs_map(self):
        for y in range(len(self._map[0])-1, -1, -1):
            self._print_obs_row(y)

def return_area_detected(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    # print(data)
    # print(type(data))
    #ID : -> 0 : wall_width, 1 : obstacle_one, 2 : obstacle_two, 3 : obstacle_three, 4 : landing, 5 : wall_length
    no_dict_okay = {0:"wall_width", 2:"obstacle_one", 8:"obstacle_two", 9:"obstacle_three",11:"landing",13:"wall_length"}
    # data = {0:[0,11,2],1:[2,1,22],2:[8,2,15],3:[9,15,13],4:[11,19,11],5:[13,1,0]}
    global env_length, env_width
    global obj_width,obj_length
    global currentAreaCoor

    lowest_left_x = []
    lowest_left_y = []
    highest_right_x = []
    highest_right_y = []
    for i in range(0,6):
        lowest_left_x.append(-1.0)
        lowest_left_y.append(-1.0)
        highest_right_x.append(-1.0)
        highest_right_y.append(-1.0)
        # print(data.markers[4].pose)

    signal = []
    for i in range(0,6):
        signal.append(True)
    #check KeyError
    for i in range(0,6):
        try:
            data.markers[i].id
        except IndexError:
            signal[i]=False

    if(signal[0]): #"wall_width"
        print "wall_width"
        lowest_left_x[0] = data.markers[0].pose.pose.position.x-env_width/2
        lowest_left_y[0] = data.markers[0].pose.pose.position.y

        highest_right_x[0] = data.markers[0].pose.pose.position.x+env_width/2
        highest_right_y[0] = data.markers[0].pose.pose.position.y
    else:
        lowest_left_x[0] = "NO_DETECTION"
        lowest_left_y[0] = "NO_DETECTION"

        highest_right_x[0] = "NO_DETECTION"
        highest_right_y[0] = "NO_DETECTION"
        # return [(lowest_left_x,lowest_left_y),(highest_right_x,highest_right_y)]

    if(signal[1]): #"obstacle_one"
        print("obstacle_one")
        lowest_left_x[1]=data.markers[1].pose.pose.position.x-obj_width/2
        lowest_left_y[1]=data.markers[1].pose.pose.position.y-obj_width/2

        highest_right_x[1] = data.markers[1].pose.pose.position.x+obj_width/2
        highest_right_y[1] = data.markers[1].pose.pose.position.y+obj_width/2
    else:
        lowest_left_x[1] = "NO_DETECTION"
        lowest_left_y[1] = "NO_DETECTION"

        highest_right_x[1] = "NO_DETECTION"
        highest_right_y[1] = "NO_DETECTION"
        # return [(lowest_left_x,lowest_left_y),(highest_right_x,highest_right_y)]

    if(signal[2]): #"obstacle_two"
        print("obstacle_two")
        lowest_left_x[2]=data.markers[2].pose.pose.position.x-obj_width/2
        lowest_left_y[2]=data.markers[2].pose.pose.position.y-obj_width/2

        highest_right_x[2] = data.markers[2].pose.pose.position.x+obj_width/2
        highest_right_y[2] = data.markers[2].pose.pose.position.y+obj_width/2
        # return [(lowest_left_x,lowest_left_y),(highest_right_x,highest_right_y)]
    else:
        lowest_left_x[2] = "NO_DETECTION"
        lowest_left_y[2] = "NO_DETECTION"

        highest_right_x[2] = "NO_DETECTION"
        highest_right_y[2] = "NO_DETECTION"

    if(signal[3]): #"obstacle_three"
        print("obstacle_three")
        lowest_left_x[3]=data.markers[3].pose.pose.position.x-obj_width/2
        lowest_left_y[3]=data.markers[3].pose.pose.position.y-obj_width/2

        highest_right_x[3] = data.markers[3].pose.pose.position.x+obj_width/2
        highest_right_y[3] = data.markers[3].pose.pose.position.y+obj_width/2
        # return [(lowest_left_x,lowest_left_y),(highest_right_x,highest_right_y)]
    else:
        lowest_left_x[3] = "NO_DETECTION"
        lowest_left_y[3] = "NO_DETECTION"

        highest_right_x[3] = "NO_DETECTION"
        highest_right_y[3] = "NO_DETECTION"

    if(signal[4]): #"landing"
        print("landing")
        lowest_left_x[4]=data.markers[4].pose.pose.position.x
        lowest_left_y[4]=data.markers[4].pose.pose.position.y

        highest_right_x[4] = data.markers[4].pose.pose.position.x
        highest_right_y[4] = data.markers[4].pose.pose.position.y
        # return [(lowest_left_x,lowest_left_y),(highest_right_x,highest_right_y)]
    else:
        lowest_left_x[4] = "NO_DETECTION"
        lowest_left_y[4] = "NO_DETECTION"

        highest_right_x[4] = "NO_DETECTION"
        highest_right_y[4] = "NO_DETECTION"

    if(signal[5]): #"wall_length"
        print("wall_length")
        lowest_left_x[5]=data.markers[5].pose.pose.position.x
        lowest_left_y[5]=data.markers[5].pose.pose.position.y-env_length/2

        highest_right_x[5] = data.markers[5].pose.pose.position.x
        highest_right_y[5] = data.markers[5].pose.pose.position.y+env_length/2
        # return [(lowest_left_x,lowest_left_y),(highest_right_x,highest_right_y)]
    else:
        lowest_left_x[5] = "NO_DETECTION"
        lowest_left_y[5] = "NO_DETECTION"

        highest_right_x[5] = "NO_DETECTION"
        highest_right_y[5] = "NO_DETECTION"

    currentAreaCoor = ([lowest_left_x,lowest_left_y],[highest_right_x,highest_right_y])
    # print(currentAreaCoor)


def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, return_area_detected)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    world_map = WorldMap()
    world_map.set_feature((1,.15),(1,0.25), OBSTACLE)
    world_map.set_feature((1.5,0.8),(1.7,1), GOAL)
    world_map.inflate()
    world_map.print_obs_map()



if __name__ == "__main__":
    main()
    exit(0)
