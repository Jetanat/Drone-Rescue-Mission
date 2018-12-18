import heapq
import pathPlanner as p

class PriorityQueue:
	def __init__(self):
		self.elements = []

	def empty(self):
		return len(self.elements) == 0

	def put(self, item, priority): #Pushes elements from the queue in order of priority
		heapq.heappush(self.elements, (priority, item))

	def get(self): #Pops elements from the queue in order of priority
		return heapq.heappop(self.elements)[1]

class Grid:
	def __init__(self, width, height):
		self.width = width
		self.height = height
		self.walls = []

	def inbound(self, id):#Looks to see if an item exists within the bounds
		(x, y) = id
		return 0 <= x < self.width and 0 <= y < self.height

	def passable(self, id): #Looks to see if a cell is passable
		return id not in self.walls

	def neighbors(self, id): #Looks at all neighbors of the current cell
		(x, y) = id
		results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
		if (x + y) % 2 == 0: results.reverse() 
		results = filter(self.inbound, results)
		results = filter(self.passable, results)
		return results

def cost(fromnode, tonode): #Cost function used for A* 
	from_y = fromnode[1]
	from_x = fromnode[0]
	to_y = tonode[1]
	to_x = tonode[0]
	return 10 if to_y - from_y and to_x - from_x else 5

def dist_to_goal(a, b): #Simple distance heuristic placeholder for heuristic in pathPlanner
	(x1, y1) = a
	(x2, y2) = b
	return abs(x1 - x2) + abs(y1 - y2)

def a_star(graph, start, goal):
	frontier = PriorityQueue()
	frontier.put(start, 0) #Push start to the priority queue
	camefrom = {}
	costsofar = {}
	camefrom[start] = None
	costsofar[start] = 0
	default = 0

	while not frontier.empty(): #If the grid is not empty
		current = frontier.get() #Pop the current element 
		if (current == goal): 
			break
		cost = costsofar[current]+1
		for nextthing in graph.neighbors(current): #Looks at neighbors of the current cell 
			
#newcost = costsofar[current] + cost(current, nextthing) #Looks at costs of moving to other cells

			if (nextthing not in costsofar or cost < costsofar[nextthing]) and nextthing != start: #Update
				costsofar[nextthing] = cost #newcost
				#priority = newcost + dist_to_goal(goal, nextthing)
				frontier.put(nextthing, 0) #priority)
				camefrom[nextthing] = current
	return camefrom, costsofar

def path(camefrom, start, goal): #Translates camefrom list into an actual path
	current = goal
	path = []
	# (-10,-10) chosen because it is known to be outside of all possible poses
	path.append((-10,-10))
	while current != start:
		path.append(current)
		new_numbers = camefrom[current]
		current = new_numbers
	path.append(start)
	path.reverse()
	#now shorten the path so the drone makes smoother movements
	prev_pose = (0,0)
	d=None
	newPath=[]
	for pose in path:
	    if not newPath:
	        newPath.append(pose)
	    else:
	        if d is None:
	            if newPath[-1][0]==pose[0]:
	                d=0
	            else:
	                d=1
	        if newPath[-1][d]!=pose[d]:
	        	newPath.append(prev_pose)
	        	newPath.append(pose)
	        	d=None
	    prev_pose = pose

	if newPath[-1] != path[-1]:
	   newPath.append(path[-1])

	return newPath

def main(): # Testing within python interpreter
	world_map = p.WorldMap()
	world_map.set_feature((1,.15),(1,0.25), 0)
	world_map.set_feature((1.5,0.8),(1.7,1), 1)
	world_map.set_feature((1.8,1.3),(1.8,1.3),3)
	world_map.inflate()
	world_map.print_obs_map()

	path(a_star(world_map, (0,0), (1.8,1.3))[0], (0,0), (1.8,1.3))
if __name__ == "__main__":
	main()
	exit(0)
