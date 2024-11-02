#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <vector>

class Edge; // Forward declaration

class Vertex {
public:
    int id;
    double longitude;
    double latitude;
    double x;
    double y;
    std::vector<Edge*> edges;

    Vertex(int id, double longitude, double latitude);
};

#endif // VERTEX_HPP