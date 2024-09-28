#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <unordered_map>
#include <string>

// Vertex class to represent a vertex in the graph
class Vertex {
public:
    int id;
    double longitude;
    double latitude;
    double x;
    double y;

    Vertex(int id, double longitude, double latitude, double x = 0, double y = 0)
        : id(id), longitude(longitude), latitude(latitude), x(x), y(y) {}
};

// Edge class to represent an edge in the graph
class Edge {
public:
    int source;
    int destination;
    double length;
    std::string name;

    Edge(int source, int destination, double length, const std::string& name)
        : source(source), destination(destination), length(length), name(name) {}
};

// Graph class to represent the entire graph
class Graph {
public:
    std::unordered_map<int, Vertex> vertices;
    std::vector<Edge> edges;

    Graph(const std::string& filename);
    void loadFromFile(const std::string& filename);
};

#endif // GRAPH_H