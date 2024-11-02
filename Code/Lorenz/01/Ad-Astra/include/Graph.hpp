#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <map>
#include <QFile>
#include "Vertex.hpp"
#include "Edge.hpp"

class Graph {
public:
    std::map<int, Vertex*> vertices;
    std::vector<Edge*> edges;

    bool loadFromFile(const std::string& filename);
};

#endif // GRAPH_HPP
