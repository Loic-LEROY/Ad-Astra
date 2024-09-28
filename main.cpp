#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <list>
#include <cmath>
#include <limits>
#include <set>
#include "commify.h"

struct Vertex {
    int id;
    double x, y; // Cartesian coordinates
    double g; // Cost from start to this vertex
    double f; // Estimated cost from start to end through this vertex
    Vertex(int id, double x, double y) : id(id), x(x), y(y), g(std::numeric_limits<double>::max()), f(std::numeric_limits<double>::max()) {}
};

class Graph {
public:
    Graph() = default;
    void addVertex(int id, double lon, double lat);
    void addEdge(int v, int w, double weight = -1);
    std::list<int> bfs(int start, int end);
    std::list<int> dijkstra(int start, int end);
    std::list<int> astar(int start, int end, std::set<int>& openSet, std::set<int>& closedSet);
    double computeDistance(const Vertex& v1, const Vertex& v2);
    void convertToCartesian(double lon, double lat, double& x, double& y);
    double heuristic(const Vertex& v1, const Vertex& v2);

private:
    std::unordered_map<int, Vertex> vertices;
    std::unordered_map<int, std::list<std::pair<int, double>>> adj;
};

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

std::list<int> Graph::bfs(int start, int end) {
    std::queue<int> active_queue;
    std::unordered_set<int> closed_set;
    std::unordered_map<int, int> parent;

    active_queue.push(start);
    closed_set.insert(start);
    parent[start] = -1;

    while (!active_queue.empty()) {
        int current = active_queue.front();
        active_queue.pop();

        if (current == end) {
            std::list<int> path;
            for (int v = end; v != -1; v = parent[v]) {
                path.push_front(v);
            }
            return path;
        }

        for (const auto& [next, weight] : adj[current]) {
            if (closed_set.find(next) == closed_set.end()) {
                active_queue.push(next);
                closed_set.insert(next);
                parent[next] = current;
            }
        }
    }

    return {};
}

std::list<int> Graph::dijkstra(int start, int end) {
    auto cmp = [](const std::pair<double, int>& left, const std::pair<double, int>& right) { return left.first > right.first; };
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, decltype(cmp)> active_queue(cmp);
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
            std::list<int> path;
            for (int v = end; v != start; v = parent[v]) {
                path.push_front(v);
            }
            path.push_front(start);
            return path;
        }

        for (const auto& [next, weight] : adj[current]) {
            double tentative_weight = shortest_path[current] + weight;
            if (tentative_weight < shortest_path[next]) {
                parent[next] = current;
                shortest_path[next] = tentative_weight;
                active_queue.emplace(tentative_weight, next);
            }
        }
    }

    return {};
}

std::list<int> Graph::astar(int start, int end, std::set<int>& openSet, std::set<int>& closedSet) {
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
    openSet.insert(start);

    while (!active_queue.empty()) {
        int current = active_queue.top().second;
        active_queue.pop();
        openSet.erase(current);
        closedSet.insert(current);

        if (current == end) {
            std::list<int> path;
            for (int v = end; v != start; v = parent[v]) {
                path.push_front(v);
            }
            path.push_front(start);
            return path;
        }

        for (const auto& [next, weight] : adj[current]) {
            double tentative_g = g_score[current] + weight;
            if (tentative_g < g_score[next]) {
                parent[next] = current;
                g_score[next] = tentative_g;
                f_score[next] = g_score[next] + heuristic(vertices[next], vertices[end]);
                active_queue.emplace(f_score[next], next);
                openSet.insert(next);
            }
        }
    }

    return {};
}

void loadGraph(const std::string& filename, Graph& graph) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file");
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(iss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.size() >= 3) {
            int id = std::stoi(tokens[0]);
            double lon = std::stod(tokens[1]);
            double lat = std::stod(tokens[2]);
            graph.addVertex(id, lon, lat);
        }

        if (tokens.size() >= 5) {
            int v1 = std::stoi(tokens[0]);
            int v2 = std::stoi(tokens[1]);
            double weight = std::stod(tokens[4]);
            graph.addEdge(v1, v2, weight);
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <start_vertex> <end_vertex> <algorithm> <graph_file>\n";
        return 1;
    }

    int start_vertex = std::stoi(argv[1]);
    int end_vertex = std::stoi(argv[2]);
    std::string algorithm = argv[3];
    std::string graph_file = argv[4];

    Graph graph;
    loadGraph(graph_file, graph);

    std::list<int> path;
    std::set<int> openSet, closedSet;
    auto startTime = std::chrono::high_resolution_clock::now();

    if (algorithm == "bfs") {
        path = graph.bfs(start_vertex, end_vertex);
    } else if (algorithm == "dijkstra") {
        path = graph.dijkstra(start_vertex, end_vertex);
    } else if (algorithm == "astar") {
        path = graph.astar(start_vertex, end_vertex, openSet, closedSet);
    } else {
        std::cerr << "Unknown algorithm: " << algorithm << "\n";
        return 1;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

    // Calculate path length
    double pathLength = 0;
    for (auto it = path.begin(); it != std::prev(path.end()); ++it) {
        int v1 = *it;
        int v2 = *std::next(it);
        pathLength += graph.computeDistance(graph.getVertex(v1), graph.getVertex(v2));
    }

    // Print path and time to console
    std::cout << "Path: ";
    for (int v : path) {
        std::cout << v << " ";
    }
    std::cout << "\nINFO: path calculated in " << Commify(duration.count()) << "us\n";
    std::cout << "Number of vertices in path: " << path.size() << "\n";
    std::cout << "Length of path: " << pathLength << " km\n";
    std::cout << "Total number of searched vertices: " << openSet.size() + closedSet.size() << "\n";

    return 0;
}