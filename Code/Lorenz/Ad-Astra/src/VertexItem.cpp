#include "VertexItem.hpp"
#include <QGraphicsSceneHoverEvent>
#include <QGraphicsSceneMouseEvent>
#include <QPen>

VertexItem::VertexItem(Vertex* vertex, double size)
    : QGraphicsEllipseItem(vertex->x - size / 2, vertex->y - size / 2, size, size),
    vertex(vertex), state(Normal) {
    setAcceptHoverEvents(true);
    setAcceptedMouseButtons(Qt::LeftButton);
    setPen(Qt::NoPen); // Remove outline
    updateAppearance();
}

Vertex* VertexItem::getVertex() const {
    return vertex;
}

void VertexItem::setState(VertexState newState) {
    state = newState;
    updateAppearance();
}

VertexItem::VertexState VertexItem::getState() const {
    return state;
}

void VertexItem::updateAppearance() {
    switch (state) {
    case Normal:
        setBrush(Qt::white);
        break;
    case Hovered:
        setBrush(Qt::yellow);
        break;
    case Start:
        setBrush(Qt::green);
        break;
    case End:
        setBrush(Qt::red);
        break;
    case Path:
        setBrush(Qt::blue);
        break;
    }
}

void VertexItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event) {
    if (state == Normal) {
        setState(Hovered);
    }
    QGraphicsEllipseItem::hoverEnterEvent(event);
}

void VertexItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
    if (state == Hovered) {
        setState(Normal);
    }
    QGraphicsEllipseItem::hoverLeaveEvent(event);
}

void VertexItem::mousePressEvent(QGraphicsSceneMouseEvent* event) {
    emit vertexClicked(vertex);
    QGraphicsEllipseItem::mousePressEvent(event);
}
