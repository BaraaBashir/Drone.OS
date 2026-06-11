
# Drone Pathfinding 

## Names and Responsibilities
1) Hiba Zahra - Responsible for the visual layout of the project.
2) Baraa Bashir - Implementation and Debugging. 
3) Aya Awissat - Animation & Timing Control
4) Sarah Ayoubi - File I/O and Makefile build system


## Description About our Project :)
 We started by thinking about how to get a single emergency drone from the Hospital to an Accident as fast as possible, using Dijkstra’s Algorithm to lock in the absolute shortest path. But then we realized: what if multiple drones need to deploy at the same time? We rebuilt the "brain" of our simulation from the ground up. Now, instead of one program controlling everything, every drone has a mind of its own. They run as independent processes, talk to our Raylib GUI through anonymous pipes, and even wait politely in traffic queues using Shared Memory Semaphores so they don't crash into each other at intersections!

## Implementation Descriptions
- **Milestone 1**: Implemented the core Dijkstra’s Algorithm logic. The program reads a graph from a text file and outputs the shortest path distances and predecessors to the console.
- **Milestone 2**: Created a visual representation of the graph using the Raylib library. Nodes are plotted in a circular layout, and the calculated shortest path is highlighted in purple.
- **Milestone 3**: Added a dynamic animation system. A drone (Hexagon) follows the shortest path. We implemented a state machine to handle a 1-second delay at intersections and discrete 300ms "jumps" along edges based on their weight (1 jump per 1km).
- **Milestone 4**: we upgraded our simulation from a single traveler to multiple passengers ( in our story , if 1 drone isnt enough , we send 3 drones instead)  moving across the graph simultaneously, where the main Raylib GUI program acts as the Parent process that calculates everyone's Dijkstra paths, draws each passenger in a unique color with a matching custom trail, and controls the animation speed.
- **Milestone 5**:shifts the project from a centralized tracking model to a decentralized, distributed multi-processing architecture by giving full navigation autonomy to sandboxed child processes. The parent process is forbidden from calculating paths or handling drone routing; instead, each independent child process reads the graph, executes its own internal Dijkstra's algorithm, and uses anonymous pipes to stream live telemetry packets back to the parent.

## How to Build
1. **Milestone 1**: `make milestone1`
2. **Milestone 2**: `make milestone2`
3. **Milestone 3**: `make milestone3`
4. **Milestone 4**: `make milestone4`
5. **Milestone 5**: `make milestone5`
   
## How to Run
After building, run the executable with the input file as an argument: `./dijkstra input.txt` for milestone 1 and `./sim input.txt` for milestones 2,3,4 and 5 .
