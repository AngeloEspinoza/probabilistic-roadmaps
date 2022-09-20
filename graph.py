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

	def __init__(self, start, goal, map_dimensions, radius):
		self.x_init = start
		self.x_goal = goal
		self.robot_radius = radius

		self.WIDTH, self.HEIGHT = map_dimensions
		self.neighbors = {}

		self.obstacles = None
		self.smooth_path = []

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
		"""Checks if a configuration is colliding with an obstacle.

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
			if obstacle.colliderect(point):
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
		x, y = random.uniform(0, self.WIDTH), random.uniform(0, self.HEIGHT)
		x_rand = int(x), int(y) # To use within the class

		# Rectangle generated around the generated random node
		left = x_rand[0] - self.robot_radius
		top = x_rand[1] - self.robot_radius
		width = 2*self.robot_radius
		height = width
		self.x_rand = pygame.Rect(left, top, width, height)

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
		return int(math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2))

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
		# Get only the coordinates of the rects of the generated random nodes
		graph_ = [coordinate.center for coordinate in graph]
		x_rand = x_rand.center
		configuration = configuration.center

		distances = []
		near = []
		near_configurations = [] # Same as near variable but for rectangles

		for state in graph_:
			distance = self.euclidean_distance(state, x_rand)
			distances.append(distance)

		# Index of the minimum distance to the generated random node
		self.min_distance = np.argmin(distances) 
		x_near = graph_[self.min_distance]

		# Indices of the k-smallest distances 
		self.distances = np.asarray(distances.copy())
		self.min_distances = np.argpartition(distances, k)

		# Get the k-smallest values from the graph
		for i in range(k):
			k_smallest = graph_[self.min_distances[i]]
			near.append(k_smallest)
			for node in graph:
				if node.center == k_smallest:
					near_configurations.append(node)

		self.neighbors.update({configuration: near})

		return near_configurations

	def interpolation(self, p1, p2):
		p11, p12 = p1[0], p1[1]
		p21, p22 = p2[0], p2[1]
		coordinates = []

		for i in range(0, 21):
			u = i / 20
			x = p11 * u + p21 * (1 - u)
			y = p12 * u + p22 * (1 - u)
			coordinates.append((x, y))

		return coordinates

	def cross_obstacle(self, configuration1, configuration2, map_):
		"""Checks if a set of configurations crosses an obstacle.

		Given two configurations configuration1, configuration2
		an interpolation between such two configurations is done
		to check if any of the points in between lie on the obstacle.

		Parameters
		----------
		configuration1 : tuple 
			Initial configuration.
		configuration2 : tuple 
			End configuration.

		Returns
		-------
		bool
		"""
		obs = self.obstacles.copy()

		# Get the rectangle center 
		configuration1_ = configuration1.center
		configuration2_ = configuration2.center
		
		configuration11, configuration12 = configuration1_[0], configuration1_[1]
		configuration21, configuration22 = configuration2_[0], configuration2_[1]

		while len(obs) > 0:
			rectangle = obs.pop(0)
			# Interpolation
			for i in range(0, 101):
				u = i / 100
				x = configuration11 * u + configuration21 * (1 - u)
				y = configuration12 * u + configuration22 * (1 - u)

				# Copy and reposition the center
				configuration2_copy = configuration2.copy()
				configuration2_copy.center = (x, y)

				if rectangle.colliderect(configuration2_copy):
					return True

		return False

	def a_star(self, start=(50, 50), end=(540, 380), nodes=None, map_=None):
		"""A* algorithm.

		A* algorithm for pathfinding in the graph.

		start : tuple
			Start node.
		end : tuple
			End node.
		nodes : list
			Collection of nodes in the graph.
		map_ : pygame.Surface
			Environment to draw on.
		"""		
		open_set = queue.PriorityQueue()
		open_set.put((0, self.x_init)) # (f_score, start)
		came_from = {}
		last_current = (0, 0)

		# Initialize to infinity all g-score and f-score nodes but the start 
		g_score = {node.center: float('inf') for node in nodes}
		g_score[self.x_init] = 0
		f_score = {node.center: float('inf') for node in nodes}
		f_score[self.x_init] = self.heuristic(self.x_init, self.x_goal)
		open_set_hash = {self.x_init}

		while not open_set.empty(): 
			current = open_set.get()[1]
			open_set_hash.remove(current)

			# Get the correspondant rectangle of the center point for the current configuration
			for node in nodes:
				if node.center == current:
					current_ = node

			if current == self.x_goal:
				self.reconstruct_path(came_from, current, map_)
				return True
			
			# k-nearest
			for neighbor in self.neighbors[current]:
				# Get the correspondant rectangle of the center point for the 
				# k-nearest neighbor configuration
				for node in nodes:
					if node.center == neighbor:
						neighbor_ = node

				temp_g_score = g_score[current] + self.euclidean_distance(current, neighbor)
				cross_obstacle = self.cross_obstacle(configuration1=current_, configuration2=neighbor_, map_=map_)

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
		self.path_coordinates = []
		self.path_coordinates.append(self.x_goal)

		while current in came_from:
			current = came_from[current]
			self.path_coordinates.append(current)

		self.generate_smooth_path()

	def generate_smooth_path(self):
		for i in range(len(self.path_coordinates)-1):
			interpolation = self.interpolation(p1=self.path_coordinates[i],
				p2=self.path_coordinates[i+1])
			self.smooth_path.append(interpolation)

		# Flat smooth path list
		self.smooth = [coordinate for coordinates in self.smooth_path[::-1] for coordinate in coordinates]
		self.smooth_path = []
	
	def draw_path_to_goal(self, map_, environment, obstacles):
		"""Draws the path from the x_goal node to the x_init node."""
		self.draw_initial_node(map_=map_)
		self.draw_goal_node(map_=map_)

		if obstacles != []:
			environment.draw_obstacles()

		for i in range(len(self.path_coordinates)-1):
			pygame.draw.line(surface=map_, color=self.RED, start_pos=self.path_coordinates[i],
				end_pos=self.path_coordinates[i+1], width=4)

		self.refresh_screen(map_=environment.map, seconds=3)

	def heuristic(self, p1, p2):
		"""Heuristic distance from point to point."""
		return self.euclidean_distance(p1, p2)

	def draw_random_node(self, map_):
		"""Draws the x_rand node."""
		pygame.draw.circle(surface=map_, color=self.GREEN, center=self.x_rand.center, 
			radius=self.robot_radius, width=0)

	def draw_initial_node(self, map_):
		"""Draws the x_init node."""
		return pygame.draw.circle(surface=map_, color=self.BLUE, center=self.x_init, 
			radius=self.robot_radius)

	def draw_goal_node(self, map_):
		"""Draws the x_goal node."""
		return pygame.draw.circle(surface=map_, color=self.RED, center=self.x_goal, 
			radius=self.robot_radius)

	def draw_local_planner(self, p1, p2, map_):
		"""Draws the local planner from node to node."""
		pygame.draw.line(surface=map_, color=self.BLACK, start_pos=p1.center, end_pos=p2.center)

	def move_robot(self, position, map_):
		"""Draws the robot moving at the given position."""
		pygame.draw.circle(surface=map_, color=(0, 0, 255),	center=position, 
			radius=self.robot_radius)

	def draw_roadmap(self, configurations, nears, map_, k, is_constant=False):
		"""Draws the roadmap constantly. Used to display it in an infinite loop."""
		self.draw_initial_node(map_=map_)
		self.draw_goal_node(map_=map_)

		for i, near in enumerate(nears):
			for j in range(len(near)):
				self.draw_local_planner(p1=configurations[i], p2=nears[i][j], map_=map_)
		
		if is_constant:
			return
		else:
			self.refresh_screen(map_=map_, seconds=3)

	def refresh_screen(self, map_, seconds):
		"""Updates the screen information and waits the given seconds."""
		seconds = int(seconds * 1000)

		# Refresh the screen
		pygame.display.update()
		pygame.time.delay(seconds)
		map_.fill(self.WHITE)

	def draw_trajectory(self, configurations, nears, environment, obstacles, k, keep_roadmap):
		"""Draws the robot moving in the map."""
		for i in range(len(self.smooth)):
			robot_position = self.smooth[i]

			if obstacles != []:
				environment.draw_obstacles()

			if keep_roadmap:
				self.draw_roadmap(configurations=configurations, nears=nears, map_=environment.map,
					k=k, is_constant=True)

			# Draw inital and final robot configuration constantly
			self.draw_initial_node(map_=environment.map)
			self.draw_goal_node(map_=environment.map)

			# Draw path to goal, and the robot movement constantly
			self.move_robot(position=robot_position, map_=environment.map)
			self.refresh_screen(map_=environment.map, seconds=0.02)