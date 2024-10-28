#include "Graph.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

bool Graph::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    bool readingVertices = false;
    bool readingEdges = false;

    while (std::getline(file, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') {
            if (line.find("Vertex List") != std::string::npos) {
                readingVertices = true;
                readingEdges = false;
            } else if (line.find("Edge List") != std::string::npos) {
                readingVertices = false;
                readingEdges = true;
            }
            continue;
        }

        std::stringstream ss(line);
        std::string token;

        if (readingVertices) {
            // Parse vertex line
            std::getline(ss, token, ','); // 'V'
            if (token != "V") continue;

            std::getline(ss, token, ',');
            int id = std::stoi(token);

            std::getline(ss, token, ',');
            double longitude = std::stod(token);

            std::getline(ss, token, ',');
            double latitude = std::stod(token);

            Vertex* vertex = new Vertex(id, longitude, latitude);
            vertices[id] = vertex;
        } else if (readingEdges) {
            // Parse edge line
            std::getline(ss, token, ','); // 'E'
            if (token != "E") continue;

            std::getline(ss, token, ',');
            int sourceId = std::stoi(token);

            std::getline(ss, token, ',');
            int destId = std::stoi(token);

            std::getline(ss, token, ',');
            double length = std::stod(token);

            std::getline(ss, token, ',');
            std::string name = token;

            Vertex* source = vertices[sourceId];
            Vertex* dest = vertices[destId];

            if (source && dest) {
                Edge* edge = new Edge(source, dest, length, name);
                edges.push_back(edge);
                source->edges.push_back(edge);
            } else {
                std::cerr << "Invalid edge: " << sourceId << " -> " << destId << std::endl;
            }
        }
    }

    file.close();
    return true;
}
