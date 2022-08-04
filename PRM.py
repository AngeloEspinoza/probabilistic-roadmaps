import pygame
import environment 
import graph
import argparse

# Command line arguments
parser = argparse.ArgumentParser(description='Implements the PRM algorithm for path planning.')
parser.add_argument('-o', '--obstacles', type=bool, action=argparse.BooleanOptionalAction,
	metavar='', required=False, help='Obstacles on the map')
parser.add_argument('-init', '--x_init', nargs='+', type=int, metavar='', required=False,
	help='Initial node position in X and Y respectively')
parser.add_argument('-goal', '--x_goal', nargs='+', type=int, metavar='', required=False,
	help='Goal node position in X and Y respectively')
parser.add_argument('-srn', '--show_random_nodes', type=bool, action=argparse.BooleanOptionalAction,
	metavar='', required=False, help='Show random nodes on screen')
parser.add_argument('-n', '--nodes', type=int, metavar='', required=False,
	help='Number of nodes to put in the roadmap')
parser.add_argument('-k', '--k_nearest', type=int, metavar='', required=False,
	help='Number of the closest neighbors to examine for each configuration')
args = parser.parse_args()

# Constants
MAP_DIMENSIONS = 640, 480

# Initial and final position of the robots
x_init = tuple(args.x_init) if args.x_init is not None else (50, 50)
x_goal = tuple(args.x_goal) if args.x_goal is not None else (540, 380)

# Instantiating the environment and the graph
environment_ = environment.Environment(dimensions=MAP_DIMENSIONS)
graph_ = graph.Graph(start=x_init, goal=x_goal, 
		map_dimensions=MAP_DIMENSIONS)

def main():
	run = True
	clock = pygame.time.Clock()
	obstacles = environment_.draw_obstacles() if args.obstacles is True else []
	configurations = []
	configurations.append(x_init)
	configurations.append(x_goal)
	graph_.obstacles = obstacles
	is_simulation_finished = False

	 # Number of nodes to put in the roadmap
	n = args.nodes if args.nodes is not None else 10

	# Number of the closest neighbors to examine for each configuration
	k = args.k_nearest if args.k_nearest is not None else 15 

	while run:
		clock.tick(environment_.FPS) 
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		x_rand = graph_.generate_random_node()
		collision_free = graph_.is_free(point=x_rand, obstacles=obstacles)
		sampling = n < graph_.MAX_NODES # Sampling time

		if collision_free and sampling:
			if args.show_random_nodes:
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
						p2=near[i], map_=environment_.map)
					if not cross_obstacle:
						graph_.draw_local_planner(p1=configuration, p2=near[i],
							map_=environment_.map)

			graph_.a_star(nodes=configurations, map_=environment_.map)
			is_simulation_finished = True

		n += 1 # Counter for the maximum allowed nodes		
		pygame.display.update()

if __name__ == '__main__':
	main()