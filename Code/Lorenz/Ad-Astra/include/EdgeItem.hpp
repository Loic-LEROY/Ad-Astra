#ifndef EDGEITEM_HPP
#define EDGEITEM_HPP

#include <QGraphicsLineItem>
#include "Edge.hpp"

class EdgeItem : public QGraphicsLineItem {
public:
    EdgeItem(Edge* edge);
    Edge* getEdge() const;

private:
    Edge* edge;
};

#endif
