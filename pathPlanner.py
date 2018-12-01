import heapq

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

class Cell:
    def __init__(self):
        self.status=OPEN

    def get_status(self):
        return self.status

    def set_status(self, status):
        self.status=status

class WorldMap:
    def __init__(self):
        self.map = [[Cell() for y in range(int(MAP_MAX_Y/MAP_RESOLUTION)+1)] for x in range(int(MAP_MAX_X/MAP_RESOLUTION)+1)]
        self.goal=None

    def __world_to_map(self,x,y):
        i = int((x-x%MAP_RESOLUTION)/MAP_RESOLUTION)
        j = int((y-y%MAP_RESOLUTION)/MAP_RESOLUTION)
        return i,j

    def dist_to_goal(x,y):
        if(self.goal == None):
            return -1
        i,j = self.__world_to_map(x,y)
        return ((i-self.goal[0])**2 + (j-self.goal[1])**2)**0.5

    def set_feature(self, low, high, status):
        #round input (x,y) to map resolution
        #round down min
        min_x, min_y = self.__world_to_map(low[0],low[1])
        #round up max
        max_x=int((high[0]+(MAP_RESOLUTION-high[0]%MAP_RESOLUTION)+MAP_RESOLUTION)/MAP_RESOLUTION)
        max_y=int((high[1]+(MAP_RESOLUTION-high[1]%MAP_RESOLUTION)+MAP_RESOLUTION)/MAP_RESOLUTION)
        for i in range(min_x, max_x):
            for j in range(min_y, max_y):
                self.map[i][j].set_status(status)
        if(status==GOAL):
            self.goal=((min_x+max_x)/2, (min_y+max_y)/2)

    def __inflate_cell(self, x, y):
        radius=int(ROBOT_RADIUS/MAP_RESOLUTION)
        for i in range(max(0, x-radius), min(int(MAP_MAX_X/MAP_RESOLUTION)+1, x+radius)):
            for j in range(max(0, y-radius), min(int(MAP_MAX_Y/MAP_RESOLUTION)+1, y+radius)):
                if(self.map[i][j].get_status() == OPEN):
                    self.map[i][j].set_status(INFLATION)

    def inflate(self):
        for x in range(int(MAP_MAX_X/MAP_RESOLUTION)+1):
            for y in range(int(MAP_MAX_Y/MAP_RESOLUTION)+1):
                if(self.map[x][y].get_status() == OBSTACLE):
                    self.__inflate_cell(x,y)

    def __print_obs_row(self, y):
        print([col[y].get_status() for col in self.map])

    def print_obs_map(self):
        for y in range(len(self.map[0])-1, -1, -1):
            self.__print_obs_row(y)


def main():
    world_map = WorldMap()
    world_map.set_feature((1,.15),(1,0.25), OBSTACLE)
    world_map.set_feature((1.5,0.8),(1.7,1), GOAL)
    world_map.inflate()
    world_map.print_obs_map()



if __name__ == "__main__":
    main()
    exit(0)
