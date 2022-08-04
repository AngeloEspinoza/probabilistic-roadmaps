import pygame
import random
import math
import numpy as np
import queue

class Graph():
	"""
	A class for the Probabilistic RoadMap (PRM).
	
	Attributes
	----------
	start : tuple
		Initial position of the tree in X and Y respectively.
	goal : tuple
		End position of the tree in X and Y respectively.
	map_dimensions : tuple
		Map width and height in pixels.
	"""

	def __init__(self, start, goal, map_dimensions):
		self.x_init = start
		self.x_goal = goal

		self.WIDTH, self.HEIGHT = map_dimensions
		self.MAX_NODES = 100
		self.neighbors = {}

		self.obstacles = None

		# Colors 
		self.WHITE = (255, 255, 255)
		self.BLACK = (0, 0, 0)
		self.RED = (255, 0, 0)
		self.GREEN = (0, 255, 0)
		self.BLUE = (0, 0, 255)
		self.BROWN = (189, 154, 122)
		self.YELLOW = (255, 255, 0)
		self.TURQUOISE = (64, 224, 208)
		self.FUCSIA = (255, 0, 255)


	def is_free(self, point, obstacles, tree=None):
		"""Checks whether a node is colliding with an obstacle or not.

		When dealing with obstacles it is necessary to check 
		for the collision with them from the generated node.

		Parameters
		----------
		point : tuple
			Point to be checked.
		obstacles : pygame.Rect
			Rectangle obstacle.
		tree : list
			Tree containing all the coordinate nodes.

		Returns
		-------
		bool
		"""
		for obstacle in obstacles:
			if obstacle.collidepoint(point):
				return False

		return True

	def generate_random_node(self):
		"""Generates a random node on the screen.

		The x and y coordinate is generated given an uniform
		distribution of the size of the screen width and height.

		Parameters
		----------
		None

		Returns
		-------
		tuple
			Coordinates of the random node. 
		"""
		x, y = random.uniform(0, self.WIDTH),\
		random.uniform(0, self.HEIGHT)
		self.x_rand = int(x), int(y) # To use within the class

		return self.x_rand

	def euclidean_distance(self, p1, p2):
		"""Euclidean distance between two points.

		Parameters
		----------
		p1 : int
			Start point.
		p2 : int 
			End point.

		Returns
		-------
		float
			Euclidean distance metric.
		"""
		return  int(math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2))

	def k_nearest(self, graph, x_rand, configuration, k=2):
		"""Given k, it returns the k-nearest neighbors of x_rand.
		
		Searches in the graph the k-nearest neighbors.

		Parameters
		----------
		graph : list
			Graph containing all the coordinate nodes.
		x_rand : tuple 
			Coordinate of the random node generated.
		configuration : tuple
			Current configuration to be search its k-neighbors.
		k : int
			Number of the closest neighbors to examine for each configuration.

		Returns
		-------
		tuple
			Nearest node to the random node generated.	
		"""
		distances = []
		near = []

		for state in graph:
			distance = self.euclidean_distance(state, x_rand)
			distances.append(distance)

		# Index of the minimum distance to the generated random node
		self.min_distance = np.argmin(distances) 
		x_near = graph[self.min_distance]

		# Indices of the k-smallest distances 
		self.distances = np.asarray(distances.copy())
		self.min_distances = np.argpartition(distances, k)

		# Get the k-smallest values from the graph
		for i in range(k):
			near.append(graph[self.min_distances[i]])

		self.neighbors.update({configuration: near})

		return near

	def cross_obstacle(self, p1, p2, map_=None):
		"""Checks whether a line crosses and obstacle or not.

		Given two points p1, p2 an interpolation between 
		such two points is done to check if any of the
		points in between lie on the obstacle.

		Parameters
		----------
		p1 : tuple 
			Initial point.
		p2 : tuple 
			End point.

		Returns
		-------
		bool
		"""
		obs = self.obstacles.copy()
		
		p11, p12 = p1[0], p1[1]
		p21, p22 = p2[0], p2[1]

		while len(obs) > 0:
			rectangle = obs.pop(0)
			# Interpolation
			for i in range(0, 101):
				u = i / 100
				x = p11 * u + p21 * (1 - u)
				y = p12 * u + p22 * (1 - u)

				if rectangle.collidepoint(x, y):
					return True

		return False

	def a_star(self, start=(50, 50), end=(540, 380), nodes=None, map_=None):
		"""A star algorithm.

		A star algorithm for pathfinding in the graph.

		start : tuple
			Start node.
		end : tuple
			End node.
		nodes : list
			Collection of nodes in the graph.
		map_ : pygame.Surface
			Environment to draw on.
		"""		
		# self.x_init = self.x_init
		# self.x_goal = self.x_goal

		self.draw_initial_node(map_=map_)
		self.draw_goal_node(map_=map_)

		open_set = queue.PriorityQueue()
		open_set.put((0, self.x_init)) # (f_score, start)
		came_from = {}
		last_current = (0, 0)

		# Initialize to infinity all g-score and f-score nodes but the start 
		g_score = {node: float('inf') for node in nodes}
		g_score[self.x_init] = 0
		f_score = {node: float('inf') for node in nodes}
		f_score[self.x_init] = self.heuristic(self.x_init, self.x_goal)
		open_set_hash = {self.x_init}

		while not open_set.empty(): 
			current = open_set.get()[1]
			open_set_hash.remove(current)

			if current == self.x_goal:
				self.reconstruct_path(came_from, current, map_)
				return True
			
			# k-nearest
			for neighbor in self.neighbors[current]:
				temp_g_score = g_score[current] + self.euclidean_distance(current, neighbor)
				cross_obstacle = self.cross_obstacle(p1=current, p2=neighbor)

				if temp_g_score < g_score[neighbor] and not cross_obstacle:
					came_from[neighbor] = current
					g_score[neighbor] = temp_g_score
					f_score[neighbor] = temp_g_score + self.heuristic(neighbor, end)

					if neighbor not in open_set_hash:
						open_set.put((f_score[neighbor], neighbor))
						open_set_hash.add(neighbor)

				last_current = current

	def reconstruct_path(self, came_from, current, map_):
		"""Reconstruct the path from point A to B."""
		paths = []

		while current in came_from:
			current = came_from[current]
			paths.append(current)

		self.draw_paths(paths, map_)
	
	def draw_paths(self, paths, map_):
		"""Draw the path on the map."""
		pygame.draw.line(surface=map_,
			color=(255, 0, 0), start_pos=self.x_goal,
			end_pos=paths[0], width=4)

		for i in range(len(paths)-1):
			pygame.draw.line(surface=map_,
				color=(255, 0, 0), start_pos=paths[i],
				end_pos=paths[i+1], width=4)
		
	def heuristic(self, p1, p2):
		"""Heuristic distance from point to point."""
		return self.euclidean_distance(p1, p2)

	def draw_random_node(self, map_):
		"""Draws the x_rand node."""
		pygame.draw.circle(surface=map_, color=self.GREEN, 
			center=self.x_rand, radius=3)

	def draw_initial_node(self, map_):
		"""Draws the x_init node."""
		pygame.draw.circle(surface=map_, color=self.BLUE, 
			center=self.x_init, radius=4)

	def draw_goal_node(self, map_):
		"""Draws the x_goal node."""
		pygame.draw.circle(surface=map_, color=self.RED, 
			center=self.x_goal, radius=4)

	def draw_local_planner(self, p1, p2, map_):
		"""Draws the local planner from node to node."""
		pygame.draw.line(surface=map_, color=self.BLACK,
			start_pos=p1, end_pos=p2)