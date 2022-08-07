# Probabilistic Roadmaps
<p align="center">
  <img src="https://user-images.githubusercontent.com/40195016/182752546-10de15d6-8a09-4281-9c34-e2012cf860fc.gif" alt="animated" />
</p>

## Description
A 2D simulation in the framework Pygame of the paper [Probabilistic roadmaps for path planning in high-dimensional configuration spaces](https://dspace.library.uu.nl/bitstream/handle/1874/17328/kavraki_94_probabilistic.pdf?sequence=1%22).
The environment has 2 non-convex obstacles that can be used or not. It also uses the __A* algorithm__ to find the more suitable an optimal path from the forest created by the PRM. 

## Usage

```
usage: PRM.py [-h] [-o | --obstacles | --no-obstacles] [-init  [...]]
              [-goal  [...]]
              [-srn | --show_random_nodes | --no-show_random_nodes] [-n] [-k]

Implements the PRM algorithm for path planning.

options:
  -h, --help            show this help message and exit
  -o, --obstacles, --no-obstacles
                        Obstacles on the map
  -init  [ ...], --x_init  [ ...]
                        Initial node position in X and Y respectively
  -goal  [ ...], --x_goal  [ ...]
                        Goal node position in X and Y respectively
  -srn, --show_random_nodes, --no-show_random_nodes
                        Show random nodes on screen
  -n , --nodes          Number of nodes to put in the roadmap
  -k , --k_nearest      Number of the closest neighbors to examine for each
                        configuration
```

 ## Examples
Generate obstacles in the map and show the random nodes $\mathbf{x_{\mathit{rand}}}$
 
 ```python3 PRM.py --obstacles --show_random_nodes```
 
 No obstacles, initial configuration $\mathbf{x_{\mathit{init}}} = (300, 300)$ and goal configuration $\mathbf{x_{\mathit{goal}}} = (50, 100)$
 
 ```python3 PRM.py --no-obstacles --x_init 300, 300 --x_goal 50, 100```
 
 
 ## License 
 MIT License

Copyright (c) [2022] [Angelo Espinoza]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
