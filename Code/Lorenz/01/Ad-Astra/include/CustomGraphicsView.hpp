#ifndef CUSTOMGRAPHICSVIEW_HPP
#define CUSTOMGRAPHICSVIEW_HPP

#include <QGraphicsView>
#include <QWheelEvent>

class CustomGraphicsView : public QGraphicsView {
    Q_OBJECT

public:
    explicit CustomGraphicsView(QWidget* parent = nullptr);

protected:
    void wheelEvent(QWheelEvent* event) override;

private:
    const double zoomFactor = 1.15; // Adjust the zoom sensitivity
};

#endif // CUSTOMGRAPHICSVIEW_HPP
