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
    double g; // Cost from start to this vertex
    double f; // Estimated cost from start to end through this vertex
    Vertex(int id, double x, double y) : id(id), x(x), y(y), g(std::numeric_limits<double>::max()), f(std::numeric_limits<double>::max()) {}
};

class Graph {
public:
    Graph(int vertices);
    void addVertex(int id, double lon, double lat);
    void addEdge(int v, int w, double weight = -1);
    std::list<int> astar(int start, int end);

private:
    int V; // Number of vertices
    std::unordered_map<int, Vertex> vertices;
    std::unordered_map<int, std::list<std::pair<int, double>>> adj; // Adjacency list with weights

    double computeDistance(const Vertex& v1, const Vertex& v2);
    void convertToCartesian(double lon, double lat, double& x, double& y);
    double heuristic(const Vertex& v1, const Vertex& v2);
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

double Graph::heuristic(const Vertex& v1, const Vertex& v2) {
    return computeDistance(v1, v2);
}

std::list<int> Graph::astar(int start, int end) {
    auto cmp = [](const std::pair<double, int>& left, const std::pair<double, int>& right) { return left.first > right.first; };
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, decltype(cmp)> active_queue(cmp);
    std::unordered_map<int, double> g_score;
    std::unordered_map<int, double> f_score;
    std::unordered_map<int, int> parent;

    for (auto& [id, vertex] : vertices) {
        g_score[id] = std::numeric_limits<double>::max();
        f_score[id] = std::numeric_limits<double>::max();
    }
    g_score[start] = 0;
    f_score[start] = heuristic(vertices[start], vertices[end]);
    active_queue.emplace(f_score[start], start);

    while (!active_queue.empty()) {
        int current = active_queue.top().second;
        active_queue.pop();

        if (current == end) {
            break;
        }

        for (const auto& [next, weight] : adj[current]) {
            double tentative_g = g_score[current] + weight;
            if (tentative_g < g_score[next]) {
                parent[next] = current;
                g_score[next] = tentative_g;
                f_score[next] = g_score[next] + heuristic(vertices[next], vertices[end]);
                active_queue.emplace(f_score[next], next);
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
    std::list<int> path = g.astar(73964, 272851);
    std::cout << "Path from 73964 to 272851: ";
    for (int v : path) {
        std::cout << v << " ";
    }
    std::cout << "\nLength of path: " << path.size() - 1 << "\n";

    return 0;
}