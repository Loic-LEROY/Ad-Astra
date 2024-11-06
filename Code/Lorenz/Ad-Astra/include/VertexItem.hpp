#ifndef VERTEXITEM_HPP
#define VERTEXITEM_HPP

#include <QObject>
#include <QGraphicsEllipseItem>
#include "Vertex.hpp"

class VertexItem : public QObject, public QGraphicsEllipseItem {
    Q_OBJECT
public:
    enum VertexState {
        Normal,
        Hovered,
        Start,
        End,
        Path
    };

    VertexItem(Vertex* vertex, double size = 3.0);

    Vertex* getVertex() const;

    void setState(VertexState state);
    VertexState getState() const;

signals:
    void vertexClicked(Vertex* vertex);

protected:
    void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

private:
    Vertex* vertex;
    VertexState state;

    void updateAppearance();
};

#endif
