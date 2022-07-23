import pygame
import random
import math
import numpy as np

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

		self.obstacles = None

		# Colors 
		self.WHITE = (255, 255, 255)
		self.BLACK = (0, 0, 0)
		self.RED = (255, 0, 0)
		self.GREEN = (0, 255, 0)
		self.BLUE = (0, 0, 255)
		self.BROWN = (189, 154, 122)
		self.YELLOW = (255, 255, 0)

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
				# tree.remove(point)
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
		self.x_rand = x, y # To use within the class

		return x, y

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
		return  math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

	def nearest_neighbor(self, graph, x_rand, k=2):
		"""Returns the index of the nearest neighbor.
		
		The nearest neighbor from all the nodes in the graph
		to the randomly generated node.

		Parameters
		----------
		graph : list
			Graph containing all the coordinate nodes.
		x_rand : tuple 
			Coordinate of the random node generated.
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
		distances = np.asarray(distances.copy())
		min_distances = np.argpartition(distances, k)

		# Get the k-smallest values from the graph
		for i in range(k):
			near.append(graph[min_distances[i]])

		return near

	def cross_obstacle(self, p1, p2):
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

	def draw_random_node(self, map_):
		"""Draws the x_rand node."""
		pygame.draw.circle(surface=map_, color=self.GREEN, 
			center=self.x_rand, radius=3)
