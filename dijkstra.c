#include <stdlib.h>
#include <limits.h>
#include <stdio.h>

#define MAX_NODES 15
#define INF INT_MAX

// Set initial values: distances to infinity, parents to null, and visited set to empty

void Initialize(int n, int src, int d[], int pi[], int S[]){
  for(int i = 0; i < n; i++){
    d[i] = INF;
    pi[i] = -1; 
    S[i] = 0; 
    }
  d[src] = 0;
}

// Find the unvisited node with the smallest known distance
int ExtractMin(int n, int d[], int S[]){
  int min = INF;
  int min_index = -1;
  for(int v = 0; v < n; v++){
    if(S[v] == 0 && d[v] <= min){
      min = d[v]; 
      min_index = v;
      }
    }
  return min_index; 
}

// Update the distance to node 'v' if a shorter path is found via node 'u'
void Relax(int u, int v, int weight, int d[], int pi[]){
  if(d[u] != INF && d[u] + weight < d[v]) { 
      d[v] = d[u] + weight;
      pi[v] = u;
      }
 }

// Recursively backtrack through the 'pi' array to print the path from source to destination
void printPath(int pi[], int j) {
   if(pi[j] == -1 ) {
   printf("%d",j);
   return;
   
  }
  printPath(pi,pi[j]);
  printf("->%d",j);
  }

 void Dijkstra(int graph[MAX_NODES][MAX_NODES], int n, int src, int dst) {
 if (src == dst ) {
   printf("0\n");
   return;
   }

 int d[MAX_NODES];    // Stores shortest distance from source to each node
 int pi[MAX_NODES];   // Stores the predecessor of each node
 int S[MAX_NODES];    // Set of visited nodes (1 if visited, 0 otherwise)

Initialize(n, src, d, pi, S);
for(int i = 0 ; i<n ; i++){
// Pick the best candidate node to process next
 int u = ExtractMin(n, d, S);
 if ( u == -1 || d[u] == INF){
   break; // No more reachable nodes
   }
   
  S[u] = 1; // Mark node as "solved"

  // Check all neighbors of the current node
  for(int v = 0; v<n; v++){
   if(graph[u][v] != INF && S[v] == 0){
   Relax(u, v, graph[u][v], d, pi);
     }
    }
   }

   // Output the final path and the total weight
 if(d[dst] == INF) {
  printf("No path found\n0\n");
  } else { 
   printPath(pi,dst);
   printf("\n%d\n", d[dst]);
    }
   }
   


  int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("Usage: %s <input_file>\n", argv[0]);
        return 1;
    }

    FILE *file = fopen(argv[1], "r");
    if (!file) {
        printf("Error opening file.\n");
        return 1;
    }

    int n, m;
    if (fscanf(file, "%d %d", &n, &m) != 2) return 1;

    // Initialize the adjacency matrix with INF to represent no edges
    int graph[MAX_NODES][MAX_NODES];
    for (int i = 0; i < MAX_NODES; i++) {
        for (int j = 0; j < MAX_NODES; j++) {
            graph[i][j] = INF;
        }
    }

 // Read edges from file and build the graph
    for (int i = 0; i < m; i++) {
        int u, v, weight;
        fscanf(file, "%d %d %d", &u, &v, &weight);
       // Dijkstra doesn't vibe with negative weights
        if (weight < 0) {
            printf("Error: Negative weights are not allowed.\n");
            fclose(file);
            return 1;
        }
        graph[u][v] = weight; 
    }

 
    int src, dst;
    fscanf(file, "%d %d", &src, &dst);
    fclose(file);
    Dijkstra(graph, n, src, dst);

    return 0;
} 
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
 
 



    
