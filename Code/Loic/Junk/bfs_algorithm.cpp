#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <list>

class Graph {
public:
    Graph(int vertices);
    void addEdge(int v, int w);
    std::list<int> bfs(int start, int end);

private:
    int V; // Number of vertices
    std::unordered_map<int, std::list<int>> adj; // Adjacency list
};

Graph::Graph(int vertices) : V(vertices) {}

void Graph::addEdge(int v, int w) {
    adj[v].push_back(w);
    adj[w].push_back(v); // Assuming undirected graph
}

std::list<int> Graph::bfs(int start, int end) {
    std::queue<int> active_queue;
    std::unordered_set<int> closed_set;
    std::unordered_map<int, int> parent;

    active_queue.push(start);
    closed_set.insert(start);
    parent[start] = -1; // Start vertex has no parent

    while (!active_queue.empty()) {
        int current = active_queue.front();
        active_queue.pop();

        if (current == end) {
            // Reconstruct the path
            std::list<int> path;
            for (int v = end; v != -1; v = parent[v]) {
                path.push_front(v);
            }
            return path;
        }

        for (int next : adj[current]) {
            if (closed_set.find(next) == closed_set.end()) {
                active_queue.push(next);
                closed_set.insert(next);
                parent[next] = current;
            }
        }
    }

    return {}; // Return empty list if no path found
}

int main() {
    Graph g(100000); // Example graph with 100000 vertices

    // Add edges (example)
    g.addEdge(19791, 12345);
    g.addEdge(12345, 50179);
    g.addEdge(73964, 272851);
    // Add more edges as needed

    // Find path from 19791 to 50179
    std::list<int> path1 = g.bfs(19791, 50179);
    std::cout << "Path from 19791 to 50179: ";
    for (int v : path1) {
        std::cout << v << " ";
    }
    std::cout << "\nLength of path: " << path1.size() - 1 << "\n";

    // Find path from 73964 to 272851
    std::list<int> path2 = g.bfs(73964, 272851);
    std::cout << "Path from 73964 to 272851: ";
    for (int v : path2) {
        std::cout << v << " ";
    }
    std::cout << "\nLength of path: " << path2.size() - 1 << "\n";

    return 0;
}