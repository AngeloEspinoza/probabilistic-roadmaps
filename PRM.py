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
	graph_.obstacles = obstacles
	is_simulation_finished = False

	n = 0 # Number of nodes to put in the roadmap
	k = 7 # Number of the closest neighbors to examine for each configuration

	while run:
		clock.tick(environment_.FPS) 
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		x_rand = graph_.generate_random_node()
		collision_free = graph_.is_free(point=x_rand, obstacles=obstacles)
		sampling = n < graph_.MAX_NODES # Sampling time

		if collision_free and sampling:
			graph_.draw_random_node(map_=environment_.map)
			configurations.append(x_rand)

		if not sampling and not is_simulation_finished:
			for configuration in configurations:
				# Remove the current configuration 
				new_configurations = configurations.copy()
				new_configurations.remove(configuration)
				near = graph_.k_nearest(graph=new_configurations, # k-nearest
					x_rand=configuration, configuration=configuration, k=k)

				for i in range(k):
					cross_obstacle = graph_.cross_obstacle(p1=configuration,
						p2=near[i])
					if not cross_obstacle:
						graph_.draw_local_planner(p1=configuration, p2=near[i],
							map_=environment_.map)

			graph_.a_star(nodes=configurations, map_=environment_.map)
			is_simulation_finished = True

		n += 1 # Counter for the maximum allowed nodes		
		pygame.display.update()

if __name__ == '__main__':
	main()