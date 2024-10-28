#ifndef DIJKSTRAALGORITHM_HPP
#define DIJKSTRAALGORITHM_HPP

#include "PathfindingAlgorithm.hpp"
#include <unordered_map>
#include <queue>

class DijkstraAlgorithm : public PathfindingAlgorithm {
public:
    std::vector<Vertex*> findPath(Vertex* start, Vertex* goal) override;
};

#endif // DIJKSTRAALGORITHM_HPP
