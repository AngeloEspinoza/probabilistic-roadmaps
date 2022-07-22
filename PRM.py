import pygame
import environment 
import graph

# Constants
MAP_DIMENSIONS = 640, 480

# Initial and final position of the robots
x_init = 50, 50
x_goal = 540, 380

# Instantiating the environment and the graph
environment_ = environment.Environment(dimensions=MAP_DIMENSIONS)
graph_ = graph.Graph(start=x_init, goal=x_goal, 
		map_dimensions=MAP_DIMENSIONS)

def main():
	run = True
	clock = pygame.time.Clock()
	obstacles = environment_.draw_obstacles()
	configurations = []

	n = 0 # Number of nodes to put in the roadmap
	k = 3 # Number of the closest neighbors to examine for each configuration

	while run:
		# Make sure the loop runs at 60 FPS
		clock.tick(environment_.FPS) 
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		x_rand = graph_.generate_random_node()
		collision_free = graph_.is_free(point=x_rand, obstacles=obstacles)
		sampling = n <= graph_.MAX_NODES # Sampling time

		if collision_free and sampling:
			graph_.draw_random_node(map_=environment_.map)
			configurations.append(x_rand)

		if not sampling:
			for configuration in configurations:
				# Remove the current configuration 
				new_configurations = configurations.copy()
				new_configurations.remove(configuration)
				x_near = graph_.nearest_neighbor(graph=new_configurations, 
					x_rand=configuration, k=k)
				for i in range(k):
					pygame.draw.line(surface=environment_.map, color=(0, 0, 255)
						, start_pos=configuration, end_pos=x_near[i])


		n += 1 # Counter for the maximum allowed nodes		
		pygame.display.update()

if __name__ == '__main__':
	main()
