#include "CustomGraphicsView.hpp"

CustomGraphicsView::CustomGraphicsView(QWidget* parent)
    : QGraphicsView(parent) {
    // Optional: Enable antialiasing for smoother rendering
    setRenderHint(QPainter::Antialiasing);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setStyleSheet("background-color: 0x232426;");
}

void CustomGraphicsView::wheelEvent(QWheelEvent* event) {
    // Zoom in or out depending on the scroll direction
    if (event->angleDelta().y() > 0) {
        scale(zoomFactor, zoomFactor);
    } else {
        scale(1.0 / zoomFactor, 1.0 / zoomFactor);
    }
}
