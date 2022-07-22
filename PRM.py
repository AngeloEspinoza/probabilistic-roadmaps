import pygame
import environment 
import graph

# Constants
MAP_DIMENSIONS = 640, 480

# Instantiating the environment
environment_ = environment.Environment(dimensions=MAP_DIMENSIONS)

def main():
	run = True
	clock = pygame.time.Clock()
	obstacles = environment_.draw_obstacles()

	while run:
		# Make sure the loop runs at 60 FPS
		clock.tick(environment_.FPS) 
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

		pygame.display.update()
		
if __name__ == '__main__':
	main()