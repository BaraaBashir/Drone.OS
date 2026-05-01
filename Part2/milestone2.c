 #include "raylib-5.0_linux_amd64/include/raylib.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#define MAX_NODES 15
#define INF INT_MAX

int d[MAX_NODES];    
int pi[MAX_NODES];   
int S[MAX_NODES]; 
int graph[MAX_NODES][MAX_NODES];
Vector2 positions[MAX_NODES];

 
char *stationNames[] = {"Hospital", "Street 1", "Street 2", "Street 3", "Street 4", "Accident"};

void DrawDirectedArrow(Vector2 start, Vector2 end, Color color, float thickness) {
    float angle = atan2f(end.y - start.y, end.x - start.x);
    
    float startOffset = 25.0f; 
    float endOffset = 35.0f;   
    float headSize = 15.0f; 

    Vector2 lineStart = {
        start.x + startOffset * cosf(angle),
        start.y + startOffset * sinf(angle)
    };

    Vector2 tip = {
        end.x - endOffset * cosf(angle),
        end.y - endOffset * sinf(angle)
    };
 
    DrawLineEx(lineStart, tip, thickness, color);
 
    DrawPoly(tip, 3, headSize, (angle * RAD2DEG) + 240 , color);
}

void Initialize(int n, int src){
    for(int i = 0; i < n; i++){
        d[i] = INF; pi[i] = -1; S[i] = 0; 
    }
    d[src] = 0;
}

int ExtractMin(int n){
    int min = INF, min_index = -1;
    for(int v = 0; v < n; v++){
        if(S[v] == 0 && d[v] <= min){
            min = d[v]; min_index = v;
        }
    }
    return min_index; 
}

void Relax(int u, int v, int weight){
    if(d[u] != INF && d[u] + weight < d[v]) { 
        d[v] = d[u] + weight;
        pi[v] = u;
    }
}

void Dijkstra(int n, int src, int dst) {
    Initialize(n, src);
    for(int i = 0 ; i < n ; i++){
        int u = ExtractMin(n);
        if ( u == -1 || d[u] == INF) break;
        S[u] = 1;
        for(int v = 0; v < n; v++){
            if(graph[u][v] != INF && S[v] == 0) Relax(u, v, graph[u][v]);
        }
    }
}

int main(int argc, char *argv[]) {
    if (argc != 2) return 1;
    FILE *file = fopen(argv[1], "r");
    if (!file) return 1;

    int n, m;
    fscanf(file, "%d %d", &n, &m);

    for(int i=0 ; i < n ; i++){
        positions[i].x = 400 + 240 * cos(i * 2 * PI / n);
        positions[i].y = 300 + 240 * sin(i * 2 * PI / n);
    }

    for (int i = 0; i < MAX_NODES; i++) 
        for (int j = 0; j < MAX_NODES; j++) graph[i][j] = INF;

    for (int i = 0; i < m; i++) {
        int u, v, w;
        fscanf(file, "%d %d %d", &u, &v, &w);
        graph[u][v] = w; 
    }

    int src, dst;
    fscanf(file, "%d %d", &src, &dst);
    fclose(file);

    Dijkstra(n, src, dst);

    InitWindow(800, 600, "Project Sentinel - Accident Scout");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
            ClearBackground(BLACK);
            
 
            for(int u=0; u<n; u++) {
                for(int v=0; v<n; v++) {
                    if(graph[u][v] != INF) {
                        DrawDirectedArrow(positions[u], positions[v], DARKGRAY, 2.0f);
                        DrawText(TextFormat("%d km", graph[u][v]), 
                                 (int)(positions[u].x + positions[v].x)/2 + 12, 
                                 (int)(positions[u].y + positions[v].y)/2 + 12, 18, BLUE);
                    }
                }
            }
            
 
            int curr = dst;
            while(curr != -1 && pi[curr] != -1) {
                int parent = pi[curr];
                DrawDirectedArrow(positions[parent], positions[curr], RED, 5.0f);
                curr = parent;
            }
            
 
            for(int i=0; i<n; i++) {
                Color nodeColor = SKYBLUE;
                if (i == src) nodeColor = GREEN;
                else if (i == dst) nodeColor = WHITE;
                
                DrawCircleV(positions[i], 22, nodeColor);
 
                DrawText(TextFormat("%d", i), (int)positions[i].x-6, (int)positions[i].y-9, 20, BLACK);
                
 
                if (i < 6) {
                    DrawText(stationNames[i], (int)positions[i].x - 30, (int)positions[i].y + 28, 16, nodeColor);
                }
            }
            
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
