#include "DijkstraAlgorithm.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"
#include <algorithm>

struct VertexDistance {
    Vertex* vertex;
    double distance;

    bool operator>(const VertexDistance& other) const {
        return distance > other.distance;
    }
};

std::vector<Vertex*> DijkstraAlgorithm::findPath(Vertex* start, Vertex* goal) {
    std::unordered_map<Vertex*, double> distances;
    std::unordered_map<Vertex*, Vertex*> previous;
    std::priority_queue<VertexDistance, std::vector<VertexDistance>, std::greater<VertexDistance>> queue;

    distances[start] = 0.0;
    queue.push({start, 0.0});

    while (!queue.empty()) {
        Vertex* current = queue.top().vertex;
        queue.pop();

        if (current == goal) break;

        for (Edge* edge : current->edges) {
            Vertex* neighbor = edge->destination;
            double newDist = distances[current] + edge->length;

            if (distances.find(neighbor) == distances.end() || newDist < distances[neighbor]) {
                distances[neighbor] = newDist;
                previous[neighbor] = current;
                queue.push({neighbor, newDist});
            }
        }
    }

    // Reconstruct path
    std::vector<Vertex*> path;
    for (Vertex* at = goal; at != nullptr; at = previous[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());
    return path;
}
