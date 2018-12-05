import heapq

class PriorityQueue:
	def __init__(self):
		self.elements = []

	def empty(self):
		return len(self.elements) == 0

	def put(self, item, priority):
		heapq.heappush(self.elements, (priority, item))

	def get(self):
		return heapq.heappop(self.elements)[1]

class Grid:
	def __init__(self, width, height):
		self.width = width
		self.height = height
		self.walls = []

	def inbound(self, id):
		(x, y) = id
		return 0 <= x < self.width and 0 <= y < self.height

	def passable(self, id):
		return id not in self.walls

	def neighbors(self, id):
		(x, y) = id
		results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
		if (x + y) % 2 == 0: results.reverse() 
		results = filter(self.inbound, results)
		results = filter(self.passable, results)
		return results

class GridWeights(Grid):
	def __init__(self, width, height):
		super().__init__(width, height)
		self.weights = {}

	def cost(self, fromnode, tonode):
		return self.weights.get(tonode, 1)


def heuristic(a, b):
	(x1, y1) = a
	(x2, y2) = b
	return abs(x1 - x2) + abs(y1 - y2)

def a_star(graph, start, goal):
	frontier = PriorityQueue()
	frontier.put(start, 0)
	camefrom = {}
	costsofar = {}
	camefrom[start] = None
	costsofar[start] = 0

	while not frontier.empty():
		current = frontier.get()

		if (current == goal):
			break

		for next in graph.neighbors(current):
			newcost = costsofar[current] + graph.cost(current, next)

			if next not in costsofar or newcost < costsofar[next]:
				costsofar[next] = newcost
				priority = newcost + heuristic(goal, next)
				frontier.put(next, priority)
				camefrom[next] = current

	return camefrom, costsofar

def path(camefrom, start, goal):
	current = goal
	path = []
	while current != start:
		path.append(current)
		currnet = came_from[current]
	path.append(start)
	path.reverse()
	return path
