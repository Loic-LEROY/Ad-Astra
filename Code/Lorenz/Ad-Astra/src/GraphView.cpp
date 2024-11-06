#include "GraphView.hpp"
#include <QWheelEvent>
#include <QtMath>

GraphView::GraphView(QWidget* parent)
    : QGraphicsView(parent) {
    setRenderHint(QPainter::Antialiasing);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorUnderMouse);
    setInteractive(true);
}

void GraphView::wheelEvent(QWheelEvent* event) {
    const double scaleFactor = 1.15;
    if (event->angleDelta().y() > 0) {
        // Zoom in
        scale(scaleFactor, scaleFactor);
    } else {
        // Zoom out
        scale(1.0 / scaleFactor, 1.0 / scaleFactor);
    }
}
