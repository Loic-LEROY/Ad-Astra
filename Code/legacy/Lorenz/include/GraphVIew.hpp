#ifndef GRAPHVIEW_HPP
#define GRAPHVIEW_HPP

#include <QGraphicsView>

class GraphView : public QGraphicsView {
    Q_OBJECT
public:
    explicit GraphView(QWidget* parent = nullptr);

protected:
    void wheelEvent(QWheelEvent* event) override;
};

#endif
