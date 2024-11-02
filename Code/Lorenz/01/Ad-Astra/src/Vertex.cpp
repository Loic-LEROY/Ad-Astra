#include "Vertex.hpp"

Vertex::Vertex(int id, double longitude, double latitude)
    : id(id), longitude(longitude), latitude(latitude), x(0), y(0) {
    // x and y can be calculated later if needed
}
