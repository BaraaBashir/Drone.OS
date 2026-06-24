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

#define ALGO_FCFS 0
#define ALGO_PRIORITY 1

typedef enum { START, MOVING, WAITING_FOR_LOCK, IN_NODE, LEFT_NODE, ARRIVED } DroneState;

typedef struct {
    int src;
    int dst;
    int priority; // NEW: Priority level
    Vector2 dronePos;
    DroneState state;
    int current_node;
    int next_node;
    int prev_node;
    Color color;
    Color pathColor;
    int has_started;
    float timer;
} Traveler;

typedef struct {
    pid_t pid;
    int traveler_idx;
    int current_node;
    int next_node;
    int prev_node;
    int is_finished;
    DroneState state;
} IPCMessage;

int graph[MAX_NODES][MAX_NODES];
Vector2 positions[MAX_NODES];
char *stationNames[] = {"Hospital", "Street 1", "Street 2", "Street 3", "Center", "Exit 1", "Exit 2", "Street 4"};

Traveler travelers[MAX_TRAVELERS];
int num_travelers = 0;
pid_t child_pids[MAX_TRAVELERS];
int num_nodes_global = 0;

// NEW: One semaphore per traveler, NOT per node!
sem_t *traveler_semaphores;

// NEW: Parent queues for Scheduling
int node_queues[MAX_NODES][MAX_TRAVELERS];
int queue_sizes[MAX_NODES] = {0};
int node_occupied[MAX_NODES] = {0};

int current_algo = ALGO_FCFS;

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

void Dijkstra(int n, int src, int dst, int *fullPath, int *pathCount) {
    int d[MAX_NODES];
    int pi[MAX_NODES];
    int S[MAX_NODES] = {0};

    for (int i = 0; i < n; i++) { d[i] = INF; pi[i] = -1; }
    d[src] = 0;

    for (int i = 0; i < n; i++) {
        int min = INF, u = -1;
        for (int v = 0; v < n; v++) {
            if (S[v] == 0 && d[v] <= min) { min = d[v]; u = v; }
        }
        if (u == -1 || d[u] == INF) break;
        S[u] = 1;
        for (int v = 0; v < n; v++) {
            if (graph[u][v] != INF && S[v] == 0) {
                if (d[u] != INF && d[u] + graph[u][v] < d[v]) {
                    d[v] = d[u] + graph[u][v]; pi[v] = u;
                }
            }
        }
    }

    int count = 0;
    int temp = dst;
    if (d[dst] == INF) { *pathCount = 0; return; }
    while (temp != -1) { fullPath[count++] = temp; temp = pi[temp]; }
    for (int i = 0; i < count / 2; i++) {
        int t = fullPath[i]; fullPath[i] = fullPath[count - 1 - i]; fullPath[count - 1 - i] = t;
    }
    *pathCount = count;
}

void GetTravelerColors(int index, Color *droneColor, Color *pathColor) {
    Color dColors[] = { MAGENTA, ORANGE, PURPLE, LIME, GOLD, PINK, RED, MAROON, GREEN };
    Color pColors[] = { 
        (Color){ 180, 0, 180, 255 }, (Color){ 180, 90, 0, 255 }, (Color){ 90, 0, 135, 255 },
        (Color){ 0, 150, 0, 255 }, (Color){ 160, 130, 0, 255 }, (Color){ 180, 80, 110, 255 },
        (Color){ 150, 0, 0, 255 }, (Color){ 90, 0, 0, 255 }, (Color){ 0, 100, 0, 255 }
    };
    int idx = index % 9;
    *droneColor = dColors[idx];
    *pathColor = pColors[idx];
}

void SendIPCMessage(int write_fd, int pid, int idx, int u, int v, int prev, int is_fin, DroneState state) {
    IPCMessage msg = {pid, idx, u, v, prev, is_fin, state};
    write(write_fd, &msg, sizeof(IPCMessage));
}

void RunChildProcess(int index, int n, int write_fd) {
    int childPath[MAX_NODES];
    int childPathCount = 0;

    Dijkstra(n, travelers[index].src, travelers[index].dst, childPath, &childPathCount);
    if (childPathCount == 0) exit(1);

    int my_pid = getpid();

    for (int step = 0; step < childPathCount; step++) {
        int u = childPath[step];
        int v = (step < childPathCount - 1) ? childPath[step + 1] : -1;
        int prev = (step > 0) ? childPath[step - 1] : -1;
        int is_finished = (v == -1) ? 1 : 0;

        // 1. Tell parent we arrived
        SendIPCMessage(write_fd, my_pid, index, u, v, prev, is_finished, WAITING_FOR_LOCK);

        // 2. Sleep until Parent wakes US specifically!
        sem_wait(&traveler_semaphores[index]);

        // 3. We are in!
        SendIPCMessage(write_fd, my_pid, index, u, v, prev, is_finished, IN_NODE);
        sleep(1);

        // 4. Tell parent we left so they can wake the next person
        SendIPCMessage(write_fd, my_pid, index, u, v, prev, is_finished, LEFT_NODE);

        // 5. Move
        if (!is_finished) {
            SendIPCMessage(write_fd, my_pid, index, u, v, prev, is_finished, MOVING);
            int W = graph[u][v];
            for (int jump = 0; jump < W; jump++) usleep(300000); 
        }
    }
    close(write_fd);
    exit(0);
}

// NEW: Parent algorithm to pick the winner
void WakeNextTraveler(int node) {
    if (queue_sizes[node] == 0) return; // Nobody waiting
    
    int winner_q_idx = 0; 
    
    if (current_algo == ALGO_PRIORITY) {
        int max_pri = -1;
        for (int i = 0; i < queue_sizes[node]; i++) {
            int t_idx = node_queues[node][i];
            if (travelers[t_idx].priority > max_pri) {
                max_pri = travelers[t_idx].priority;
                winner_q_idx = i;
            }
        }
    }
    // If FCFS, winner_q_idx remains 0 (first in line)

    int winner_t_idx = node_queues[node][winner_q_idx];

    // Shift queue left
    for (int i = winner_q_idx; i < queue_sizes[node] - 1; i++) {
        node_queues[node][i] = node_queues[node][i+1];
    }
    queue_sizes[node]--;

    node_occupied[node] = 1; // Mark node as busy
    sem_post(&traveler_semaphores[winner_t_idx]); // Wake up the winner!
}


int main(int argc, char *argv[]) {
    // NEW: Command Line Parsing
    if (argc != 3) {
        printf("Usage: ./sim-schd <fcfs|priority> <file_name>\n");
        return 1;
    }
    
    if (strcmp(argv[1], "fcfs") == 0) { current_algo = ALGO_FCFS; }
    else if (strcmp(argv[1], "priority") == 0) { current_algo = ALGO_PRIORITY; }
    else {
        printf("Invalid algorithm. Use 'fcfs' or 'priority'.\n");
        return 1;
    }

    FILE *file = fopen(argv[2], "r");
    if (!file) return 1;

    int n = 0, m = 0;
    char line[256];
    
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        if (sscanf(line, "%d %d", &n, &m) == 2) break;
    }
    num_nodes_global = n;

    for (int i = 0; i < n; i++) {
        positions[i].x = 400 + 240 * cos(i * 2 * PI / n);
        positions[i].y = 300 + 240 * sin(i * 2 * PI / n);
        for (int j = 0; j < MAX_NODES; j++) graph[i][j] = INF;
    }

    int edges_read = 0;
    while (edges_read < m && fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        int u, v, w;
        if (sscanf(line, "%d %d %d", &u, &v, &w) == 3) {
            graph[u][v] = w; edges_read++;
        }
    }

    while (fgets(line, sizeof(line), file)) { if (strstr(line, "#travelers")) break; }

    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        sscanf(line, "%d", &num_travelers); break;
    }

    // Allocate semaphores per traveler, NOT per node
    traveler_semaphores = mmap(NULL, sizeof(sem_t) * num_travelers, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    for (int i = 0; i < num_travelers; i++) {
        sem_init(&traveler_semaphores[i], 1, 0); // Init to 0 (Everyone starts locked)
    }

    int travelers_read = 0;
    while (travelers_read < num_travelers && fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        int src, dst, pri = 1;
        if (sscanf(line, "%d %d %d", &src, &dst, &pri) >= 2) { // Read 2 or 3 args
            travelers[travelers_read].src = src;
            travelers[travelers_read].dst = dst;
            travelers[travelers_read].priority = pri; // Save priority
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
    pipe(pipe_fds);
    fcntl(pipe_fds[0], F_SETFL, O_NONBLOCK);

    for (int i = 0; i < num_travelers; i++) {
        pid_t pid = fork();
        if (pid == 0) {
            close(pipe_fds[0]);
            RunChildProcess(i, n, pipe_fds[1]);
        } else { child_pids[i] = pid; }
    }
    close(pipe_fds[1]);

    InitWindow(800, 600, "Project Sentinel - Milestone 7 (Scheduling)");
    SetTargetFPS(60);

    bool isPlaying = false;
    Rectangle btn = { 20, 20, 100, 40 };

    while (!WindowShouldClose()) {
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(GetMousePosition(), btn)) {
            isPlaying = true; 
        }

        if (isPlaying) {
            IPCMessage msg;
            while (read(pipe_fds[0], &msg, sizeof(IPCMessage)) > 0) {
                int idx = msg.traveler_idx;
                int u = msg.current_node;
                travelers[idx].current_node = u;
                travelers[idx].next_node = msg.next_node;
                travelers[idx].prev_node = msg.prev_node;
                
                // NEW: Parent Queue Management
                if (msg.state == WAITING_FOR_LOCK) {
                    travelers[idx].state = WAITING_FOR_LOCK;
                    node_queues[u][queue_sizes[u]++] = idx; // Add to waitlist
                    if (!node_occupied[u]) WakeNextTraveler(u); // Node is free, wake someone!
                }
                else if (msg.state == LEFT_NODE) {
                    node_occupied[u] = 0; // Node is free now
                    WakeNextTraveler(u); // Wake next in line
                }
                else if (msg.state == IN_NODE) {
                    travelers[idx].state = msg.is_finished ? ARRIVED : IN_NODE;
                }
                else if (msg.state == MOVING) {
                    travelers[idx].state = MOVING;
                    travelers[idx].timer = 0.0f;
                }
            }

            float dt = GetFrameTime();
            for (int i = 0; i < num_travelers; i++) {
                if (travelers[i].state == MOVING && travelers[i].next_node != -1) {
                    travelers[i].timer += dt;
                    int u = travelers[i].current_node, v = travelers[i].next_node, W = graph[u][v];
                    float progress = fminf(travelers[i].timer / (W * 0.3f), 1.0f);
                    travelers[i].dronePos.x = positions[u].x + progress * (positions[v].x - positions[u].x);
                    travelers[i].dronePos.y = positions[u].y + progress * (positions[v].y - positions[u].y);
                } else if (travelers[i].state == WAITING_FOR_LOCK && travelers[i].prev_node != -1) {
                    float dx = positions[travelers[i].prev_node].x - positions[travelers[i].current_node].x;
                    float dy = positions[travelers[i].prev_node].y - positions[travelers[i].current_node].y;
                    float dist = sqrtf(dx*dx + dy*dy);
                    if (dist > 0) {
                        travelers[i].dronePos.x = positions[travelers[i].current_node].x + (dx / dist) * 45.0f;
                        travelers[i].dronePos.y = positions[travelers[i].current_node].y + (dy / dist) * 45.0f;
                    }
                } else {
                    travelers[i].dronePos = positions[travelers[i].current_node];
                }
            }
        }

        BeginDrawing();
        ClearBackground(BLACK);

        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (graph[u][v] != INF) {
                    DrawDirectedArrow(positions[u], positions[v], DARKGRAY, 2.0f);
                }
            }
        }

        for (int i = 0; i < num_travelers; i++) {
            if (travelers[i].has_started && travelers[i].state != ARRIVED && travelers[i].next_node != -1) {
                DrawDirectedArrow(positions[travelers[i].current_node], positions[travelers[i].next_node], travelers[i].pathColor, 4.0f);
            }
        }

        for (int i = 0; i < n; i++) {
            DrawCircleV(positions[i], 22, SKYBLUE);
            DrawText(TextFormat("%d", i), (int)positions[i].x - 6, (int)positions[i].y - 9, 20, BLACK);
            if (i < 8) DrawText(stationNames[i], (int)positions[i].x - 30, (int)positions[i].y + 28, 16, RAYWHITE);
        }

        if (!isPlaying) {
            DrawRectangleRec(btn, GREEN); DrawText("PLAY", (int)btn.x + 20, (int)btn.y + 10, 20, WHITE);
        }

        // NEW: Draw Algorithm Text
        DrawText(TextFormat("ALGORITHM: %s", current_algo == ALGO_FCFS ? "FCFS" : "PRIORITY"), 300, 20, 24, YELLOW);

        for (int i = 0; i < num_travelers; i++) {
            if (travelers[i].has_started && travelers[i].state != ARRIVED) {
                if (travelers[i].state == WAITING_FOR_LOCK) DrawCircleLines((int)travelers[i].dronePos.x, (int)travelers[i].dronePos.y, 30.0f, RED);
                DrawPoly(travelers[i].dronePos, 6, 18.0f, 0.0f, travelers[i].color);
                DrawPolyLinesEx(travelers[i].dronePos, 6, 18.0f, 0.0f, 2.0f, WHITE);
                
                // NEW: Draw Priority Number on Drone
                DrawText(TextFormat("%d", travelers[i].priority), (int)travelers[i].dronePos.x - 4, (int)travelers[i].dronePos.y - 6, 12, BLACK);
            }
        }

        EndDrawing();
    }

    close(pipe_fds[0]);
    CloseWindow();
    for (int i = 0; i < num_travelers; i++) waitpid(child_pids[i], NULL, 0);
    for (int i = 0; i < num_travelers; i++) sem_destroy(&traveler_semaphores[i]);
    munmap(traveler_semaphores, sizeof(sem_t) * num_travelers);
    return 0;
}
