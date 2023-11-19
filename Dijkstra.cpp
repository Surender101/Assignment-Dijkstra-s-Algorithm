#include <iostream>
#include <vector>
#include <climits>

using namespace std;

const int INF = INT_MAX; // A high value representing infinity for unreachable nodes

class Dijkstra {
private:
    int vertices; // Number of vertices in the graph
    vector<vector<int>> graph; // Adjacency matrix representing edges and their weights
    vector<int> predecessors;  // Keeps track of predecessors to reconstruct the shortest path

public:
    Dijkstra(int v) : vertices(v), graph(v, vector<int>(v, INF)), predecessors(v, -1) {}

    // Adds an undirected edge between two nodes with a specified weight
    void addEdge(int from, int to, int weight) {
        graph[from][to] = weight;
        graph[to][from] = weight; // Assuming an undirected graph
    }

    // Applies Dijkstra's algorithm to find the shortest path from startNode to endNode
    void dijkstraAlgorithm(int startNode, int endNode) {
        vector<int> distance(vertices, INF); // Initializes distances to infinity
        vector<bool> visited(vertices, false);

        distance[startNode] = 0; // Sets the distance to the starting node as 0

        // Iterates over vertices - 1 times to find the shortest paths
        for (size_t i = 0; i < vertices - 1; ++i) {
            int minDist = INF;
            size_t minIndex = 0;

            // Finds the unvisited node with the minimum distance
            for (size_t j = 0; j < vertices; ++j) {
                if (!visited[j] && distance[j] < minDist) {
                    minDist = distance[j];
                    minIndex = j;
                }
            }

            visited[minIndex] = true;

            // Updates distances and predecessors for neighboring nodes
            for (size_t k = 0; k < vertices; ++k) {
                if (!visited[k] && graph[minIndex][k] != INF && distance[minIndex] != INF &&
                    distance[minIndex] + graph[minIndex][k] < distance[k]) {
                    distance[k] = distance[minIndex] + graph[minIndex][k];
                    predecessors[k] = minIndex;  // Updates predecessors
                }
            }
        }

        // Outputs the result
        cout << "Shortest distance from node " << startNode << " to node " << endNode << ": " << distance[endNode] << endl;
        cout << "Path: ";
        printPath(startNode, endNode);
        cout << endl;
    }

    // Recursively prints the path from startNode to endNode
    void printPath(int startNode, int endNode) {
        if (endNode == startNode) {
            cout << startNode;
            return;
        }

        // Recursively prints the path
        printPath(startNode, static_cast<int>(predecessors[endNode]));
        cout << " -> " << endNode;
    }
};

int main() {
    // Example usage
    Dijkstra dijkstra(6);

    // Adds edges to the graph
    dijkstra.addEdge(0, 1, 4);
    dijkstra.addEdge(0, 2, 2);
    dijkstra.addEdge(1, 2, 5);
    dijkstra.addEdge(1, 3, 10);
    dijkstra.addEdge(2, 3, 3);
    dijkstra.addEdge(3, 4, 7);
    dijkstra.addEdge(4, 5, 2);

    int startNode, endNode;
    cout << "Enter the starting node: ";
    cin >> startNode;
    cout << "Enter the ending node: ";
    cin >> endNode;

    // Applies Dijkstra's algorithm and prints the result
    dijkstra.dijkstraAlgorithm(startNode, endNode);

    return 0;
}
