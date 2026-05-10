 #include "raylib.h" //hiiii
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

// Define graph limits and INF constant for unreachable nodes (standard for Dijkstra)
#define MAX_NODES 15
#define INF INT_MAX

// Global arrays to store shortest path metadata: distances (d), parents (pi), and visited status (S)
int d[MAX_NODES];
int pi[MAX_NODES];
int S[MAX_NODES];
int graph[MAX_NODES][MAX_NODES];
Vector2 positions[MAX_NODES];
char *stationNames[] = {"Hospital", "Street 1", "Street 2", "Street 3", "Street 4", "Accident"};

typedef enum { START, MOVING, WAITING, ARRIVED } DroneState;

// Helper to render directional lines with arrowheads between nodes
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

// Initialization: set all distances to infinity and mark all nodes as unvisited
void Initialize(int n, int src) {
    for (int i = 0; i < n; i++) { d[i] = INF; pi[i] = -1; S[i] = 0; }
    d[src] = 0;
}

// Greedily find the node with the smallest distance that hasn't been visited yet
int ExtractMin(int n) {
    int min = INF, min_index = -1;
    for (int v = 0; v < n; v++) {
        if (S[v] == 0 && d[v] <= min) { min = d[v]; min_index = v; }
    }
    return min_index;
}

// Relaxation: check if passing through node 'u' provides a shorter path to node 'v'
void Relax(int u, int v, int weight) {
    if (d[u] != INF && d[u] + weight < d[v]) {
        d[v] = d[u] + weight;
        pi[v] = u;
    }
}

void Dijkstra(int n, int src, int dst) {
    Initialize(n, src);
    for (int i = 0; i < n; i++) {
        int u = ExtractMin(n);
        if (u == -1 || d[u] == INF) break;
        S[u] = 1;
        for (int v = 0; v < n; v++) {
            if (graph[u][v] != INF && S[v] == 0) Relax(u, v, graph[u][v]);
        }
    }
}

int main(int argc, char *argv[]) {
    if (argc != 2) return 1;
    FILE *file = fopen(argv[1], "r");
    if (!file) return 1;

 // Input Parsing: Load graph topology and calculate node positions in a circular layout
    int n, m;
    fscanf(file, "%d %d", &n, &m);
    for (int i = 0; i < n; i++) {
        positions[i].x = 400 + 240 * cos(i * 2 * PI / n);
        positions[i].y = 300 + 240 * sin(i * 2 * PI / n);
    }
    for (int i = 0; i < MAX_NODES; i++) for (int j = 0; j < MAX_NODES; j++) graph[i][j] = INF;
    for (int i = 0; i < m; i++) {
        int u, v, w;
        fscanf(file, "%d %d %d", &u, &v, &w);
        graph[u][v] = w;
    }
    int src, dst;
    fscanf(file, "%d %d", &src, &dst);
    fclose(file);

    Dijkstra(n, src, dst);

 // Reconstruct the final path by following the parent (pi) pointers back to source
    int fullPath[MAX_NODES];
    int pathCount = 0;
    int temp = dst;
    while (temp != -1) {
        fullPath[pathCount++] = temp;
        temp = pi[temp];
    }
    for (int i = 0; i < pathCount / 2; i++) {
        int t = fullPath[i];
        fullPath[i] = fullPath[pathCount - 1 - i];
        fullPath[pathCount - 1 - i] = t;
    }

    InitWindow(800, 600, "Project Sentinel - Milestone 3");
    SetTargetFPS(60);

    Vector2 dronePos = positions[src];
    DroneState state = START;
    int currentStep = 0;
    int jumpIndex = 0;
    float timer = 0.0f;
    bool isPlaying = false;
    Rectangle btn = { 20, 20, 100, 40 };

    while (!WindowShouldClose()) {
     // Input Handling: Toggle the simulation on/off when the button is clicked
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(GetMousePosition(), btn)) {
            isPlaying = !isPlaying;
        }

        if (isPlaying && state != ARRIVED) {
            timer += GetFrameTime();
         // Timing Logic: Handle the 1-second delay for node-to-node transitions
            if (state == START || state == WAITING) {
                if (timer >= 1.0f) {
                    timer = 0;
                    state = MOVING;
                    jumpIndex = 0;
                }
            } else if (state == MOVING) {
             // Interpolation: Calculate the drone's position between current and next node
                int u = fullPath[currentStep];
                int v = fullPath[currentStep + 1];
                int W = graph[u][v];
                if (timer >= 0.3f) {
                    timer = 0;
                    jumpIndex++;
                    dronePos.x = positions[u].x + (float)jumpIndex / W * (positions[v].x - positions[u].x);
                    dronePos.y = positions[u].y + (float)jumpIndex / W * (positions[v].y - positions[u].y);
                    if (jumpIndex >= W) {
                        currentStep++;
                        if (currentStep >= pathCount - 1) state = ARRIVED;
                        else state = WAITING;
                    }
                }
            }
        }

        BeginDrawing();
        ClearBackground(BLACK);
// Rendering: Draw all edges in the graph and label them with their weights (km)
        for (int u = 0; u < n; u++) {
            for (int v = 0; v < n; v++) {
                if (graph[u][v] != INF) {
                    DrawDirectedArrow(positions[u], positions[v], DARKGRAY, 2.0f);
                    DrawText(TextFormat("%d km", graph[u][v]), (int)(positions[u].x + positions[v].x) / 2 + 12, (int)(positions[u].y + positions[v].y) / 2 + 12, 18, BLUE);
                }
            }
        }

     // Visualization Overlay: Highlight the calculated shortest path and draw the drone
        for (int i = 0; i < pathCount - 1; i++) {
            DrawDirectedArrow(positions[fullPath[i]], positions[fullPath[i + 1]], VIOLET, 4.0f);
        }

        for (int i = 0; i < n; i++) {
            Color nodeColor = (i == src) ? GREEN : (i == dst) ? WHITE : SKYBLUE;
            DrawCircleV(positions[i], 22, nodeColor);
            DrawText(TextFormat("%d", i), (int)positions[i].x - 6, (int)positions[i].y - 9, 20, BLACK);
            if (i < 6) DrawText(stationNames[i], (int)positions[i].x - 30, (int)positions[i].y + 28, 16, nodeColor);
        }

        DrawRectangleRec(btn, isPlaying ? RED : GREEN);
        DrawText(isPlaying ? "STOP" : "PLAY", (int)btn.x + 20, (int)btn.y + 10, 20, WHITE);
        DrawPoly(dronePos, 6, 18.0f, 0.0f, MAGENTA);
        DrawPolyLinesEx(dronePos, 6, 18.0f, 0.0f, 2.0f, WHITE);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
