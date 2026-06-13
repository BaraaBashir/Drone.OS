#define _DEFAULT_SOURCE
#include "raylib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <semaphore.h>

#define MAX_NODES 15
#define INF INT_MAX
#define MAX_TRAVELERS 100

// Tracks the current execution and animation phase of a drone
typedef enum { START, MOVING, WAITING_FOR_LOCK, IN_NODE, ARRIVED } DroneState;

typedef struct {
    int src;
    int dst;
    Vector2 dronePos;
    DroneState state;
    int current_node;
    int next_node;
    int prev_node; // Added to know which street they are waiting on!
    Color color;
    Color pathColor;
    int has_started;
    float timer;
} Traveler;

// IPC payload sent from child worker processes to the main GUI process
typedef struct {
    pid_t pid;
    int traveler_idx;
    int current_node;
    int next_node;
    int prev_node; // Pass history through pipe
    int is_finished;
    DroneState state;
} IPCMessage;

int graph[MAX_NODES][MAX_NODES];
Vector2 positions[MAX_NODES];
char *stationNames[] = {"Hospital", "Street 1", "Street 2", "Street 3", "Street 4", "Accident"};

Traveler travelers[MAX_TRAVELERS];
int num_travelers = 0;
pid_t child_pids[MAX_TRAVELERS];
int num_nodes_global = 0;

// Shared pointer for semaphores accessible by all forked child processes
sem_t *node_semaphores;

void DrawDirectedArrow(Vector2 start, Vector2 end, Color color, float thickness) {
    float angle = atan2f(end.y - start.y, end.x - start.x);
    float startOffset = 25.0f;
    float endOffset = 35.0f;
    float headSize = 15.0f;
    Vector2 lineStart = { start.x + startOffset * cosf(angle), start.y + startOffset * sinf(angle) };
    Vector2 tip = { end.x - endOffset * cosf(angle), end.y - endOffset * sinf(angle) };
    DrawLineEx(lineStart, tip, thickness, color);
    DrawPoly(tip, 3, headSize, (angle * RAD2DEG) + 240, color);
}

// Standard Dijkstra routine to populate fullPath with nodes from src to dst
void Dijkstra(int n, int src, int dst, int *fullPath, int *pathCount) {
    int d[MAX_NODES];
    int pi[MAX_NODES];
    int S[MAX_NODES] = {0};

    for (int i = 0; i < n; i++) {
        d[i] = INF;
        pi[i] = -1;
    }
    d[src] = 0;

    for (int i = 0; i < n; i++) {
        int min = INF, u = -1;
        for (int v = 0; v < n; v++) {
            if (S[v] == 0 && d[v] <= min) {
                min = d[v];
                u = v;
            }
        }
        if (u == -1 || d[u] == INF) break;
        S[u] = 1;
        for (int v = 0; v < n; v++) {
            if (graph[u][v] != INF && S[v] == 0) {
                if (d[u] != INF && d[u] + graph[u][v] < d[v]) {
                    d[v] = d[u] + graph[u][v];
                    pi[v] = u;
                }
            }
        }
    }

    int count = 0;
    int temp = dst;
    if (d[dst] == INF) {
        *pathCount = 0;
        return;
    }
    while (temp != -1) {
        fullPath[count++] = temp;
        temp = pi[temp];
    }
    for (int i = 0; i < count / 2; i++) {
        int t = fullPath[i];
        fullPath[i] = fullPath[count - 1 - i];
        fullPath[count - 1 - i] = t;
    }
    *pathCount = count;
}

void GetTravelerColors(int index, Color *droneColor, Color *pathColor) {
    Color dColors[] = { MAGENTA, ORANGE, PURPLE, LIME, GOLD, PINK, RED, MAROON, GREEN };
    Color pColors[] = { 
        (Color){ 180, 0, 180, 255 },
        (Color){ 180, 90, 0, 255 },
        (Color){ 90, 0, 135, 255 },
        (Color){ 0, 150, 0, 255 },
        (Color){ 160, 130, 0, 255 },
        (Color){ 180, 80, 110, 255 },
        (Color){ 150, 0, 0, 255 },
        (Color){ 90, 0, 0, 255 },
        (Color){ 0, 100, 0, 255 }
    };
    int idx = index % (sizeof(dColors) / sizeof(Color));
    *droneColor = dColors[idx];
    *pathColor = pColors[idx];
}

void SendIPCMessage(int write_fd, int pid, int traveler_idx, int current_node, int next_node, int prev_node, int is_finished, DroneState state) {
    IPCMessage msg;
    msg.pid = pid;
    msg.traveler_idx = traveler_idx;
    msg.current_node = current_node;
    msg.next_node = next_node;
    msg.prev_node = prev_node; // Send history
    msg.is_finished = is_finished;
    msg.state = state;
    write(write_fd, &msg, sizeof(IPCMessage));
}

void RunChildProcess(int index, int n, int write_fd) {
    int childPath[MAX_NODES];
    int childPathCount = 0;

    Dijkstra(n, travelers[index].src, travelers[index].dst, childPath, &childPathCount);

    if (childPathCount == 0) {
        exit(1);
    }

    int my_pid = getpid();

    for (int step = 0; step < childPathCount; step++) {
        int u = childPath[step];
        int v = (step < childPathCount - 1) ? childPath[step + 1] : -1;
        int prev = (step > 0) ? childPath[step - 1] : -1; // Keep track of where we came from
        int is_finished = (v == -1) ? 1 : 0;

        // 1. Tell GUI we are waiting outside the node
        SendIPCMessage(write_fd, my_pid, index, u, v, prev, is_finished, WAITING_FOR_LOCK);

        // 2. Request Lock (Wait in traffic)
        sem_wait(&node_semaphores[u]);

        // 3. Lock Acquired! Tell GUI we are inside
        SendIPCMessage(write_fd, my_pid, index, u, v, prev, is_finished, IN_NODE);

        // 4. Mandatory 1 second stay inside the node
        sleep(1);

        // 5. Leave node and unlock it for others
        sem_post(&node_semaphores[u]);

        // 6. Travel down the street to the next node
        if (!is_finished) {
            SendIPCMessage(write_fd, my_pid, index, u, v, prev, is_finished, MOVING);
            int W = graph[u][v];
            for (int jump = 0; jump < W; jump++) {
                usleep(300000); 
            }
        }
    }

    close(write_fd);
    exit(0);
}

int main(int argc, char *argv[]) {
    if (argc != 2) return 1;
    FILE *file = fopen(argv[1], "r");
    if (!file) return 1;

    int n = 0, m = 0;
    char line[256];
    
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        sscanf(line, "%d %d", &n, &m);
        break;
    }
    num_nodes_global = n;

    // Allocate shared, anonymous memory map so children can share the same array of semaphores
    node_semaphores = mmap(NULL, sizeof(sem_t) * n, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (node_semaphores == MAP_FAILED) {
        perror("mmap failed");
        exit(1);
    }

    // Initialize each node semaphore with a value of 1 (binary mutex lock, shared between processes)
    for (int i = 0; i < n; i++) {
        if (sem_init(&node_semaphores[i], 1, 1) != 0) {
            perror("sem_init failed");
            exit(1);
        }
    }

    // Calculate circular visual positions for rendering nodes
    for (int i = 0; i < n; i++) {
        positions[i].x = 400 + 240 * cos(i * 2 * PI / n);
        positions[i].y = 300 + 240 * sin(i * 2 * PI / n);
    }

    for (int i = 0; i < MAX_NODES; i++) {
        for (int j = 0; j < MAX_NODES; j++) {
            graph[i][j] = INF;
        }
    }

    int edges_read = 0;
    while (edges_read < m && fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        int u, v, w;
        if (sscanf(line, "%d %d %d", &u, &v, &w) == 3) {
            graph[u][v] = w;
            edges_read++;
        }
    }

    while (fgets(line, sizeof(line), file)) {
        if (strstr(line, "#travelers")) break;
    }

    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        sscanf(line, "%d", &num_travelers);
        break;
    }

    int travelers_read = 0;
    while (travelers_read < num_travelers && fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        int src, dst;
        if (sscanf(line, "%d %d", &src, &dst) == 2) {
            travelers[travelers_read].src = src;
            travelers[travelers_read].dst = dst;
            travelers[travelers_read].dronePos = positions[src];
            travelers[travelers_read].state = START;
            travelers[travelers_read].current_node = src;
            travelers[travelers_read].next_node = -1;
            travelers[travelers_read].prev_node = -1;
            travelers[travelers_read].has_started = 1; 
            travelers[travelers_read].timer = 0.0f;    
            GetTravelerColors(travelers_read, &travelers[travelers_read].color, &travelers[travelers_read].pathColor);
            travelers_read++;
        }
    }
    fclose(file);

    int pipe_fds[2];
    if (pipe(pipe_fds) == -1) {
        perror("Pipe creation failed");
        return 1;
    }

    // Set reading end to non-blocking to prevent the Raylib GUI window loop from freezing
    fcntl(pipe_fds[0], F_SETFL, O_NONBLOCK);

    for (int i = 0; i < num_travelers; i++) {
        pid_t pid = fork();
        if (pid < 0) {
            perror("Fork failed");
            exit(1);
        } else if (pid == 0) {
            close(pipe_fds[0]); // Child process only writes to pipe
            RunChildProcess(i, n, pipe_fds[1]);
        } else {
            child_pids[i] = pid;
        }
    }
    close(pipe_fds[1]); // Main process only reads from pipe

    InitWindow(800, 600, "Project Sentinel - Milestone 6 (Traffic Jam)");
    SetTargetFPS(60);

    bool isPlaying = false;
    Rectangle btn = { 20, 20, 100, 40 };

    while (!WindowShouldClose()) {
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(GetMousePosition(), btn)) {
            isPlaying = true; 
        }

        if (isPlaying) {
            IPCMessage msg;
            // Drain all available messages from the pipe buffer each frame
            while (read(pipe_fds[0], &msg, sizeof(IPCMessage)) > 0) {
                int idx = msg.traveler_idx;
                travelers[idx].current_node = msg.current_node;
                travelers[idx].next_node = msg.next_node;
                travelers[idx].prev_node = msg.prev_node;
                travelers[idx].state = msg.state;
                
                if (msg.state == MOVING) {
                    travelers[idx].timer = 0.0f;
                }

                if (msg.state == IN_NODE) {
                    if (msg.is_finished) {
                        travelers[idx].state = ARRIVED;
                        printf("[PID=%d] arrived and LOCKED node %d | DESTINATION\n", msg.pid, msg.current_node);
                    } else {
                        printf("[PID=%d] arrived and LOCKED node %d | next node: %d\n", msg.pid, msg.current_node, msg.next_node);
                    }
                } else if (msg.state == WAITING_FOR_LOCK) {
                    printf("[PID=%d] waiting outside node %d...\n", msg.pid, msg.current_node);
                }
                fflush(stdout);
            }

            float dt = GetFrameTime();
            for (int i = 0; i < num_travelers; i++) {
                if (travelers[i].state == MOVING && travelers[i].next_node != -1) {
                    // Linearly interpolate positions based on edge weight duration (Weight * 0.3 seconds)
                    travelers[i].timer += dt;
                    int u = travelers[i].current_node;
                    int v = travelers[i].next_node;
                    int W = graph[u][v];
                    
                    float total_time = W * 0.3f;
                    float progress = travelers[i].timer / total_time;
                    if (progress > 1.0f) progress = 1.0f; 
                    
                    travelers[i].dronePos.x = positions[u].x + progress * (positions[v].x - positions[u].x);
                    travelers[i].dronePos.y = positions[u].y + progress * (positions[v].y - positions[u].y);
                    
                } else if (travelers[i].state == WAITING_FOR_LOCK && travelers[i].prev_node != -1) {
                    // Offset drawing vector back along the incoming street to visually queue outside the locked node
                    float dx = positions[travelers[i].prev_node].x - positions[travelers[i].current_node].x;
                    float dy = positions[travelers[i].prev_node].y - positions[travelers[i].current_node].y;
                    float dist = sqrtf(dx*dx + dy*dy);
                    
                    if (dist > 0) {
                        // Push them back 45 pixels along the street they came from
                        travelers[i].dronePos.x = positions[travelers[i].current_node].x + (dx / dist) * 45.0f;
                        travelers[i].dronePos.y = positions[travelers[i].current_node].y + (dy / dist) * 45.0f;
                    }
                } else {
                    // If IN_NODE, ARRIVED, or starting: lock to the exact center
                    travelers[i].dronePos = positions[travelers[i].current_node];
                }
            }
        }

        BeginDrawing();
        ClearBackground(BLACK);

        // Render entire structural graph map layout
        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (graph[u][v] != INF) {
                    DrawDirectedArrow(positions[u], positions[v], DARKGRAY, 2.0f);
                    DrawText(TextFormat("%d km", graph[u][v]), (int)(positions[u].x + positions[v].x) / 2 + 12, (int)(positions[u].y + positions[v].y) / 2 + 12, 18, BLUE);
                }
            }
        }

        // Draw active tracking lines for drones currently in transit
        for (int i = 0; i < num_travelers; i++) {
            if (travelers[i].has_started && travelers[i].state != ARRIVED && travelers[i].next_node != -1) {
                DrawDirectedArrow(positions[travelers[i].current_node], positions[travelers[i].next_node], travelers[i].pathColor, 4.0f);
            }
        }

        // Render static node bases and names
        for (int i = 0; i < n; i++) {
            Color nodeColor = SKYBLUE;
            for (int t = 0; t < num_travelers; t++) {
                if (travelers[t].src == i && travelers[t].state != ARRIVED) nodeColor = GREEN;
                else if (travelers[t].dst == i && travelers[t].state != ARRIVED) nodeColor = WHITE;
            }
            DrawCircleV(positions[i], 22, nodeColor);
            DrawText(TextFormat("%d", i), (int)positions[i].x - 6, (int)positions[i].y - 9, 20, BLACK);
            if (i < 6) DrawText(stationNames[i], (int)positions[i].x - 30, (int)positions[i].y + 28, 16, nodeColor);
        }

        if (!isPlaying) {
            DrawRectangleRec(btn, GREEN);
            DrawText("PLAY", (int)btn.x + 20, (int)btn.y + 10, 20, WHITE);
        } else {
            DrawRectangleRec(btn, GRAY);
            DrawText("RUNNING", (int)btn.x + 8, (int)btn.y + 10, 16, WHITE);
        }

        // Draw active drone icons and add visual ring warnings if stuck waiting for an intersection lock
        for (int i = 0; i < num_travelers; i++) {
            if (travelers[i].has_started && travelers[i].state != ARRIVED) {
                if (travelers[i].state == WAITING_FOR_LOCK) {
                    DrawCircleLines((int)travelers[i].dronePos.x, (int)travelers[i].dronePos.y, 30.0f, RED);
                }
                
                DrawPoly(travelers[i].dronePos, 6, 18.0f, 0.0f, travelers[i].color);
                DrawPolyLinesEx(travelers[i].dronePos, 6, 18.0f, 0.0f, 2.0f, WHITE);
            }
        }

        int all_arrived = 1;
        for (int i = 0; i < num_travelers; i++) {
            if (travelers[i].state != ARRIVED) all_arrived = 0;
        }
        if (all_arrived && num_travelers > 0 && isPlaying) {
            DrawText("ALL TRAVELERS ARRIVED!", 280, 25, 24, GREEN);
        }

        EndDrawing();
    }

    close(pipe_fds[0]);
    CloseWindow();

    // Cleanly reclaim resources and prevent zombie processes or dangling OS shared maps
    for (int i = 0; i < num_travelers; i++) {
        waitpid(child_pids[i], NULL, 0);
    }

    for (int i = 0; i < n; i++) {
        sem_destroy(&node_semaphores[i]);
    }
    munmap(node_semaphores, sizeof(sem_t) * n);

    return 0;
}

