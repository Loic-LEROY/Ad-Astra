#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <list>
#include <cmath>
#include <limits>

struct Vertex {
    int id;
    double x, y; // Cartesian coordinates
    double weight;
    Vertex(int id, double x, double y) : id(id), x(x), y(y), weight(std::numeric_limits<double>::max()) {}
};

class Graph {
public:
    Graph(int vertices);
    void addVertex(int id, double lon, double lat);
    void addEdge(int v, int w, double weight = -1);
    std::list<int> dijkstra(int start, int end);

private:
    int V; // Number of vertices
    std::unordered_map<int, Vertex> vertices;
    std::unordered_map<int, std::list<std::pair<int, double>>> adj; // Adjacency list with weights

    double computeDistance(const Vertex& v1, const Vertex& v2);
    void convertToCartesian(double lon, double lat, double& x, double& y);
};

Graph::Graph(int vertices) : V(vertices) {}

void Graph::addVertex(int id, double lon, double lat) {
    double x, y;
    convertToCartesian(lon, lat, x, y);
    vertices[id] = Vertex(id, x, y);
}

void Graph::addEdge(int v, int w, double weight) {
    if (weight < 0) {
        weight = computeDistance(vertices[v], vertices[w]);
    }
    adj[v].emplace_back(w, weight);
    adj[w].emplace_back(v, weight); // Assuming undirected graph
}

double Graph::computeDistance(const Vertex& v1, const Vertex& v2) {
    return std::sqrt(std::pow(v1.x - v2.x, 2) + std::pow(v1.y - v2.y, 2));
}

void Graph::convertToCartesian(double lon, double lat, double& x, double& y) {
    // Local Mercator projection
    x = lon * 20037508.34 / 180;
    y = std::log(std::tan((90 + lat) * M_PI / 360)) / (M_PI / 180);
    y = y * 20037508.34 / 180;
}

std::list<int> Graph::dijkstra(int start, int end) {
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> active_queue;
    std::unordered_map<int, double> shortest_path;
    std::unordered_map<int, int> parent;

    for (auto& [id, vertex] : vertices) {
        shortest_path[id] = std::numeric_limits<double>::max();
    }
    shortest_path[start] = 0;
    active_queue.emplace(0, start);

    while (!active_queue.empty()) {
        int current = active_queue.top().second;
        active_queue.pop();

        if (current == end) {
            break;
        }

        for (const auto& [next, weight] : adj[current]) {
            double tentative_weight = shortest_path[current] + weight;
            if (tentative_weight < shortest_path[next]) {
                shortest_path[next] = tentative_weight;
                parent[next] = current;
                active_queue.emplace(tentative_weight, next);
            }
        }
    }

    std::list<int> path;
    for (int v = end; v != start; v = parent[v]) {
        path.push_front(v);
    }
    path.push_front(start);
    return path;
}

int main() {
    Graph g(100000); // Example graph with 100000 vertices

    // Add vertices (example)
    g.addVertex(19791, -77.0365, 38.8977); // Example coordinates
    g.addVertex(50179, -77.0091, 38.8895); // Example coordinates
    g.addVertex(73964, -77.0369, 38.9072); // Example coordinates
    g.addVertex(272851, -77.0502, 38.8893); // Example coordinates

    // Add edges (example)
    g.addEdge(19791, 50179);
    g.addEdge(73964, 272851);
    // Add more edges as needed

    // Find shortest path from 73964 to 272851
    std::list<int> path = g.dijkstra(73964, 272851);
    std::cout << "Path from 73964 to 272851: ";
    for (int v : path) {
        std::cout << v << " ";
    }
    std::cout << "\nLength of path: " << path.size() - 1 << "\n";

    return 0;
}