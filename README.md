
# Drone Pathfinding 

## Names and Responsibilities
1) Hiba Zahra - Responsible for the visual layout of the project.
2) Baraa Bashir - Implementation and Debugging. 
3) Aya Awissat - Animation & Timing Control
4) Sarah Ayoubi - File I/O and Makefile build system


## Description About our Project :)
We started by thinking about how to get from the Hospital to an Accident as fast as possible. We used Dijkstra’s Algorithm as the "brain" of the simulation. It works by looking at every possible street (edge) and constantly updating the "best known distance" from the start until it locks in the absolute shortest path to the destination.

## Implementation Descriptions
- **Milestone 1**: Implemented the core Dijkstra’s Algorithm logic. The program reads a graph from a text file and outputs the shortest path distances and predecessors to the console.
- **Milestone 2**: Created a visual representation of the graph using the Raylib library. Nodes are plotted in a circular layout, and the calculated shortest path is highlighted in purple.
- **Milestone 3**: Added a dynamic animation system. A drone (Hexagon) follows the shortest path. We implemented a state machine to handle a 1-second delay at intersections and discrete 300ms "jumps" along edges based on their weight (1 jump per 1km).

## How to Build
1. **Milestone 1**: `make milestone1`
2. **Milestone 2**: `make milestone2`
3. **Milestone 3**: `make milestone3`

## How to Run
After building, run the executable with the input file as an argument: `./dijkstra input.txt` for milestone 1 and `./sim input.txt` for milestones 2 and 3.

note: we used one laptop because this was the only available option , but we worked as a group :)  
