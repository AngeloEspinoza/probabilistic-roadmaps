import pygame
import environment 
import graph
import argparse
import sys

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
parser.add_argument('-n', '--nodes', type=int, metavar='', required=False, default=100,
	help='Number of nodes to put in the roadmap')
parser.add_argument('-k', '--k_nearest', type=int, metavar='', required=False,
	help='Number of the closest neighbors to examine for each configuration')
parser.add_argument('-kt', '--keep_roadmap', type=bool, action=argparse.BooleanOptionalAction, 
	metavar='', required=False, help='Keeps the tree while the robot is moving towards the goal')
parser.add_argument('-r', '--radius', type=int, metavar='', required=False, default=10,
	help='Set the robot radius')
args = parser.parse_args()

# Initialization 
pygame.init()

# Constants
MAP_DIMENSIONS = 640, 480

# Initial and final position of the robot
x_init = tuple(args.x_init) if args.x_init is not None else (50, 50)
x_goal = tuple(args.x_goal) if args.x_goal is not None else (540, 380)

# Instantiating the environment and the graph
environment_ = environment.Environment(map_dimensions=MAP_DIMENSIONS)
graph_ = graph.Graph(start=x_init, goal=x_goal, 
		map_dimensions=MAP_DIMENSIONS, radius=args.radius)

def main():
	run = True
	clock = pygame.time.Clock()
	configurations = []
	nears = []
	success_configurations = []
	success_nears = []
	initial = graph_.draw_initial_node(map_=environment_.map)
	goal = graph_.draw_goal_node(map_=environment_.map)
	configurations.append(initial)
	configurations.append(goal)
	environment_.make_obstacles()
	obstacles = environment_.draw_obstacles() if args.obstacles else []
	graph_.obstacles = obstacles
	is_simulation_finished = False
	is_configuration_free = True

	# Number of nodes to put in the roadmap
	n = 0

	# Number of the closest neighbors to examine for each configuration
	k = args.k_nearest if args.k_nearest is not None else 15 

	while run:
		clock.tick(environment_.FPS) 
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		obstacles = environment_.draw_obstacles() if args.obstacles else []
		x_rand = graph_.generate_random_node()
		collision_free = graph_.is_free(point=x_rand, obstacles=obstacles)
		sampling = n < args.nodes # Sampling time

		if collision_free and sampling:
			if args.show_random_nodes:
				graph_.draw_random_node(map_=environment_.map)

			configurations.append(x_rand)
			n += 1 # Counter for the maximum allowed nodes		

		if not sampling and not is_simulation_finished:
			for configuration in configurations:
				# Remove the current configuration 
				new_configurations = configurations.copy()
				new_configurations.remove(configuration)
				near = graph_.k_nearest(graph=new_configurations, # k-nearest
					x_rand=configuration, configuration=configuration, k=k)

				for i in range(k):
					cross_obstacle = graph_.cross_obstacle(configuration1=configuration,
						configuration2=near[i], map_=environment_.map)
					
					if not cross_obstacle:
						graph_.draw_local_planner(p1=configuration, p2=near[i],
							map_=environment_.map)
						nears.append(near[i])

						if is_configuration_free: 
							success_configurations.append(configuration)
							is_configuration_free = False

				if nears != []:
					success_nears.append(nears)
					nears = []			
				is_configuration_free = True

			graph_.a_star(nodes=configurations, map_=environment_.map)
			is_simulation_finished = True

		if is_simulation_finished:
			graph_.draw_roadmap(configurations=success_configurations, nears=success_nears, 
				map_=environment_.map, k=k)
			graph_.draw_trajectory(configurations=success_configurations, nears=success_nears, 
				environment=environment_, obstacles=obstacles, k=k, keep_roadmap=args.keep_roadmap)
			graph_.draw_path_to_goal(map_=environment_.map, environment=environment_,
				 obstacles=obstacles)

		pygame.display.update()

	pygame.quit()
	sys.exit()

if __name__ == '__main__':
	main()