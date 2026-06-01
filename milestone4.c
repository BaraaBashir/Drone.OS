 #define _DEFAULT_SOURCE
 #include "raylib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>

#define MAX_NODES 15
#define INF INT_MAX
#define MAX_TRAVELERS 100

typedef enum { START, MOVING, WAITING, ARRIVED } DroneState;

typedef struct {
    int src;
    int dst;
    int fullPath[MAX_NODES];
    int pathCount;
    Vector2 dronePos;
    DroneState state;
    int currentStep;
    int jumpIndex;
    float timer;
    Color color;
    Color pathColor;
} Traveler;

int graph[MAX_NODES][MAX_NODES];
Vector2 positions[MAX_NODES];
char *stationNames[] = {"Hospital", "Street 1", "Street 2", "Street 3", "Street 4", "Accident"};

Traveler travelers[MAX_TRAVELERS];
int num_travelers = 0;
pid_t child_pids[MAX_TRAVELERS];

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
        (Color){ 180, 0, 180, 255 },   // Dark Magenta
        (Color){ 180, 90, 0, 255 },   // Dark Orange
        (Color){ 90, 0, 135, 255 },    // Dark Purple
        (Color){ 0, 150, 0, 255 },     // Dark Lime/Green
        (Color){ 160, 130, 0, 255 },   // Dark Gold
        (Color){ 180, 80, 110, 255 },  // Dark Pink
        (Color){ 150, 0, 0, 255 },     // Dark Red
        (Color){ 90, 0, 0, 255 },      // Deeper Maroon
        (Color){ 0, 100, 0, 255 }      // Deeper Green
    };
    int idx = index % (sizeof(dColors) / sizeof(Color));
    *droneColor = dColors[idx];
    *pathColor = pColors[idx];
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
            travelers[travelers_read].currentStep = 0;
            travelers[travelers_read].jumpIndex = 0;
            travelers[travelers_read].timer = 0.0f;
            GetTravelerColors(travelers_read, &travelers[travelers_read].color, &travelers[travelers_read].pathColor);
            travelers_read++;
        }
    }
    fclose(file);

    for (int i = 0; i < num_travelers; i++) {
        Dijkstra(n, travelers[i].src, travelers[i].dst, travelers[i].fullPath, &travelers[i].pathCount);
        
        pid_t pid = fork();
        if (pid < 0) {
            perror("Fork failed");
            exit(1);
        } else if (pid == 0) {
            printf("[%d] started\n", getpid());
            fflush(stdout);
            while (1) {
                sleep(1);
            }
            exit(0);
        } else {
            child_pids[i] = pid;
        }
    }

    InitWindow(800, 600, "Project Sentinel - Milestone 4");
    SetTargetFPS(60);

    bool isPlaying = false;
    Rectangle btn = { 20, 20, 100, 40 };

    while (!WindowShouldClose()) {
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(GetMousePosition(), btn)) {
            isPlaying = !isPlaying;
        }

        if (isPlaying) {
            float dt = GetFrameTime();
            for (int i = 0; i < num_travelers; i++) {
                if (travelers[i].state == ARRIVED || travelers[i].pathCount == 0) {
                    if (child_pids[i] != 0) {
                        kill(child_pids[i], SIGKILL);
                        waitpid(child_pids[i], NULL, 0);
                        child_pids[i] = 0;
                    }
                    continue;
                }

                travelers[i].timer += dt;

                if (travelers[i].state == START || travelers[i].state == WAITING) {
                    if (travelers[i].timer >= 1.0f) {
                        travelers[i].timer = 0;
                        travelers[i].state = MOVING;
                        travelers[i].jumpIndex = 0;
                    }
                } else if (travelers[i].state == MOVING) {
                    int u = travelers[i].fullPath[travelers[i].currentStep];
                    int v = travelers[i].fullPath[travelers[i].currentStep + 1];
                    int W = graph[u][v];

                    if (travelers[i].timer >= 0.3f) {
                        travelers[i].timer = 0;
                        travelers[i].jumpIndex++;
                        travelers[i].dronePos.x = positions[u].x + (float)travelers[i].jumpIndex / W * (positions[v].x - positions[u].x);
                        travelers[i].dronePos.y = positions[u].y + (float)travelers[i].jumpIndex / W * (positions[v].y - positions[u].y);

                        if (travelers[i].jumpIndex >= W) {
                            travelers[i].currentStep++;
                            if (travelers[i].currentStep >= travelers[i].pathCount - 1) {
                                travelers[i].state = ARRIVED;
                                if (child_pids[i] != 0) {
                                    kill(child_pids[i], SIGKILL);
                                    waitpid(child_pids[i], NULL, 0);
                                    child_pids[i] = 0;
                                }
                            } else {
                                travelers[i].state = WAITING;
                            }
                        }
                    }
                }
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
            if (travelers[i].state != ARRIVED && travelers[i].pathCount > 0) {
                for (int j = travelers[i].currentStep; j < travelers[i].pathCount - 1; j++) {
                    DrawDirectedArrow(positions[travelers[i].fullPath[j]], positions[travelers[i].fullPath[j + 1]], travelers[i].pathColor, 4.0f);
                }
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

        DrawRectangleRec(btn, isPlaying ? RED : GREEN);
        DrawText(isPlaying ? "STOP" : "PLAY", (int)btn.x + 20, (int)btn.y + 10, 20, WHITE);

        for (int i = 0; i < num_travelers; i++) {
            if (travelers[i].state != ARRIVED) {
                DrawPoly(travelers[i].dronePos, 6, 18.0f, 0.0f, travelers[i].color);
                DrawPolyLinesEx(travelers[i].dronePos, 6, 18.0f, 0.0f, 2.0f, WHITE);
            }
        }

        int all_arrived = 1;
        for (int i = 0; i < num_travelers; i++) {
            if (travelers[i].state != ARRIVED) all_arrived = 0;
        }
        if (all_arrived && num_travelers > 0) {
            DrawText("ALL TRAVELERS ARRIVED!", 280, 25, 24, GREEN);
        }

        EndDrawing();
    }

    CloseWindow();

    for (int i = 0; i < num_travelers; i++) {
        if (child_pids[i] != 0) {
            kill(child_pids[i], SIGKILL);
            waitpid(child_pids[i], NULL, 0);
        }
    }

    return 0;
}
