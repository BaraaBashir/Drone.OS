
# Drone Pathfinding 

## Names and Responsibilities 
1) Hiba Zahra - GUI & visuals 
2) Baraa Bashir - Multi-Processing & Synchronization 
3) Aya Awissat - Communication & Timing
4) Sarah Ayoubi - Pathfinding Engine & Data Parsing


## Description About our Project :)
 We started by thinking about how to get a single emergency drone from the Hospital to an Accident as fast as possible, using Dijkstra’s Algorithm to lock in the absolute shortest path. But then we realized: what if multiple drones need to deploy at the same time? We rebuilt the "brain" of our simulation from the ground up. Now, instead of one program controlling everything, every drone has a mind of its own. They run as independent processes, talk to our Raylib GUI through anonymous pipes, and even wait politely in traffic queues using Shared Memory Semaphores so they don't crash into each other at intersections!

## Implementation Descriptions
- **Milestone 1**: Implemented the core Dijkstra’s Algorithm logic. The program reads a graph from a text file and outputs the shortest path distances and predecessors to the console.
- **Milestone 2**: Created a visual representation of the graph using the Raylib library. Nodes are plotted in a circular layout, and the calculated shortest path is highlighted in purple.
- **Milestone 3**: Added a dynamic animation system. A drone (Hexagon) follows the shortest path. We implemented a state machine to handle a 1-second delay at intersections and discrete 300ms "jumps" along edges based on their weight (1 jump per 1km).
- **Milestone 4**: we upgraded our simulation from a single traveler to multiple passengers ( in our story , if 1 drone isnt enough , we send 3 drones instead)  moving across the graph simultaneously, where the main Raylib GUI program acts as the Parent process that calculates everyone's Dijkstra paths, draws each passenger in a unique color with a matching custom trail, and controls the animation speed.
- **Milestone 5**:shifts the project from a centralized tracking model to a decentralized, distributed multi-processing architecture by giving full navigation autonomy to sandboxed child processes. The parent process is forbidden from calculating paths or handling drone routing; instead, each independent child process reads the graph, executes its own internal Dijkstra's algorithm, and uses anonymous pipes to stream live telemetry packets back to the parent.
- **Milestone 6**: We added a realistic traffic control system to our drone network. Since multiple drones might try to enter the same intersection at once, we implemented POSIX Semaphores in Shared Memory to act as traffic locks. Now, only one drone can enter a node at a time (waiting inside for exactly 1 second). Any other drones that arrive must wait outside in a queue which is visibly marked with a glowing red traffic halo in the GU until the intersection is clear.
- **Milestone 7**: We upgraded our intersection traffic locks from random OS-controlled semaphores to Parent-controlled scheduling algorithms. The parent process now manages a dedicated waiting queue for each intersection. We implemented two distinct schedulers: First-Come-First-Serve (FCFS) and Priority. By passing the algorithm choice via the command line, we can actively observe how wait times change; for example, the Priority algorithm dynamically allows high-priority travelers (like an ambulance) to skip the queue at a congested node, completely bypassing the standard wait times enforced by FCFS.


## How to Build
1. **Milestone 1**: `make milestone1`
2. **Milestone 2**: `make milestone2`
3. **Milestone 3**: `make milestone3`
4. **Milestone 4**: `make milestone4`
5. **Milestone 5**: `make milestone5`
6. **Milestone 6**: `make milestone6`
7. **Milestone 7**: `make milestone7`
   
## How to Run
 After building, run the executable with the input file as an argument: ./dijkstra input.txt for milestone 1 and ./sim input.txt for milestones 2, 3, 4, and 5. For milestone 6, run ./sim input_m6.txt. For milestone 7, the executable name changes to sim-schd and requires you to specify the scheduling algorithm (fcfs or priority) before the input file: ./sim-schd fcfs input_m7.txt or ./sim-schd priority input_m7.txt

