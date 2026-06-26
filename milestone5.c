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

#define MAX_NODES 15
#define INF INT_MAX
#define MAX_TRAVELERS 100

typedef enum { START, MOVING, WAITING, ARRIVED } DroneState;

typedef struct {
    int src;
    int dst;
    Vector2 dronePos;
    DroneState state;
    int current_node;
    int next_node;
    Color color;
    Color pathColor;
    int has_started;
} Traveler;

typedef struct {
    pid_t pid;
    int traveler_idx;
    int current_node;
    int next_node;
    int prev_node;
    int is_finished;
    DroneState state;
    int no_path_found;  // NEW: 1 if no path exists, 0 otherwise
} IPCMessage;

int graph[MAX_NODES][MAX_NODES];
Vector2 positions[MAX_NODES];
char *stationNames[] = {"Hospital", "Street 1", "Street 2", "Street 3", "Street 4", "Accident"};

Traveler travelers[MAX_TRAVELERS];
int num_travelers = 0;
pid_t child_pids[MAX_TRAVELERS];
int num_nodes_global = 0;

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
/* Child process entry point — each traveler runs as a separate process.
 * Computes its own shortest path via Dijkstra, then writes IPCMessage
 * structs step-by-step into the shared write end of the pipe. */

 void RunChildProcess(int index, int n, int write_fd) {
    int childPath[MAX_NODES];
    int childPathCount = 0;

   
    Dijkstra(n, travelers[index].src, travelers[index].dst, childPath, &childPathCount);

    if (childPathCount == 0) {
        IPCMessage error_msg;
        error_msg.pid = getpid();
        error_msg.traveler_idx = index;
        error_msg.current_node = travelers[index].src;
        error_msg.next_node = travelers[index].dst;
        error_msg.prev_node = -1;
        error_msg.is_finished = 0;
        error_msg.state = ARRIVED;
        error_msg.no_path_found = 1;  // Mark as "no path" message
        
        write(write_fd, &error_msg, sizeof(IPCMessage));
        close(write_fd);
        exit(0);
    }

   
    for (int step = 0; step < childPathCount; step++) {
        IPCMessage msg;
        msg.pid = getpid();
        msg.traveler_idx = index;
        msg.current_node = childPath[step];
        msg.is_finished = 0;

        if (step < childPathCount - 1) {
            msg.next_node = childPath[step + 1];
        } else {
            msg.next_node = -1;
            msg.is_finished = 1;
        }
        /* Atomic for small writes on Linux (PIPE_BUF = 4096 bytes). Since
         * sizeof(IPCMessage) is well under that limit, multiple children writing
         * concurrently will not interleave — each write is delivered whole. */

       
        write(write_fd, &msg, sizeof(IPCMessage));

   
        if (!msg.is_finished) {
            int u = childPath[step];
            int v = childPath[step + 1];
            int W = graph[u][v];

      /* Simulates travel time proportional to edge weight W.
             * Each "jump" represents one km of travel (300 ms per km). */
            for (int jump = 0; jump < W; jump++) {
                usleep(300000); 
            }
            
            
            sleep(1);
        }
    }

    close(write_fd);
    exit(0);
}

int main(int argc, char *argv[]) {
    if (argc != 2) return 1;
    FILE *file = fopen(argv[1], "r");
    if (!file) return 1;

    int n, m;
    char line[256];
    
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        sscanf(line, "%d %d", &n, &m);
        break;
    }
    num_nodes_global = n;

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
            travelers[travelers_read].has_started = 0;
            GetTravelerColors(travelers_read, &travelers[travelers_read].color, &travelers[travelers_read].pathColor);
            travelers_read++;
        }
    }
    fclose(file);

     /* Single shared pipe for all child → parent communication.
     * Safe here because IPCMessage fits within PIPE_BUF, guaranteeing
     * atomic writes even with multiple concurrent child processes. */

    int pipe_fds[2];
    if (pipe(pipe_fds) == -1) {
        perror("Pipe creation failed");
        return 1;
    }

     /* Set read end non-blocking so the render loop is never stalled waiting
     * for a child. read() returns EAGAIN immediately when the pipe is empty,
     * keeping the GUI at 60 FPS regardless of child activity. */

    fcntl(pipe_fds[0], F_SETFL, O_NONBLOCK);

    for (int i = 0; i < num_travelers; i++) {
        pid_t pid = fork();
        if (pid < 0) {
            perror("Fork failed");
            exit(1);
        } else if (pid == 0) {
            close(pipe_fds[0]);
            RunChildProcess(i, n, pipe_fds[1]);
        } else {
            child_pids[i] = pid;
        }
    }
     /* Parent closes its copy of the write end. Until this point, the pipe
     * has one write-end fd per child plus this one. Closing here ensures
     * read() will eventually return 0 (EOF) once all children exit and close
     * their own write-end copies — without this, the pipe never signals EOF. */
    close(pipe_fds[1]);

    InitWindow(800, 600, "Project Sentinel - Milestone 5 (IPC Pipes)");
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
        
        // NEW: Handle special "no path found" message
        if (msg.no_path_found == 1) {
            printf("\n[PID=%d] ERROR: No path found for traveler %d (from node %d to node %d)\n",
                   msg.pid, msg.traveler_idx, msg.current_node, msg.next_node);
            travelers[msg.traveler_idx].state = ARRIVED;
            fflush(stdout);
            continue;  // Skip normal processing
        }
        
        // Original message processing
        int idx = msg.traveler_idx;
        travelers[idx].has_started = 1;
        travelers[idx].current_node = msg.current_node;
        travelers[idx].next_node = msg.next_node;
        travelers[idx].prev_node = msg.prev_node;
        travelers[idx].state = msg.state;
        
                if (msg.is_finished) {
                    travelers[idx].state = ARRIVED;
                    printf("[PID=%d] arrived at node %d | DESTINATION\n", msg.pid, msg.current_node);
                    printf("[PID=%d] finished\n", msg.pid);
                } else {
                    travelers[idx].state = MOVING;
                    printf("[PID=%d] arrived at node %d | next node: %d\n", msg.pid, msg.current_node, msg.next_node);
                }
                fflush(stdout);
            }
        }

        BeginDrawing();
        ClearBackground(BLACK);

        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (graph[u][v] != INF) {
                    DrawDirectedArrow(positions[u], positions[v], DARKGRAY, 2.0f);
                    DrawText(TextFormat("%d km", graph[u][v]), (int)(positions[u].x + positions[v].x) / 2 + 12, (int)(positions[u].y + positions[v].y) / 2 + 12, 18, BLUE);
                }
            }
        }

        for (int i = 0; i < num_travelers; i++) {
            if (travelers[i].has_started && travelers[i].state != ARRIVED && travelers[i].next_node != -1) {
                DrawDirectedArrow(positions[travelers[i].current_node], positions[travelers[i].next_node], travelers[i].pathColor, 4.0f);
            }
        }

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

        for (int i = 0; i < num_travelers; i++) {
            if (travelers[i].has_started && travelers[i].state != ARRIVED) {
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

     /* Reap all child processes to avoid zombies. Called after CloseWindow()
     * so the GUI doesn't block; by then every child has had time to finish
     * and exit normally. */

    for (int i = 0; i < num_travelers; i++) {
        waitpid(child_pids[i], NULL, 0);
    }

    return 0;
}
