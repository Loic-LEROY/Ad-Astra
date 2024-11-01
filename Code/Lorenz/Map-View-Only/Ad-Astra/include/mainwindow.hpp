#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>

#include "Graph.hpp"
#include "CustomGraphicsView.hpp"


class MainWindow : public QMainWindow {
    Q_OBJECT
    enum AlgorithmType { DIJKSTRA, ASTAR, BFS };

public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void selectStartVertex();
    void selectEndVertex();
    void findPath();
    void selectAlgorithm();


private:
    Graph graph;
    CustomGraphicsView* graphicsView;
    QGraphicsScene* scene;
    Vertex* startVertex;
    Vertex* endVertex;
    AlgorithmType currentAlgorithm;

    void loadGraph();
    void displayGraph();
    void drawPath(const std::vector<Vertex*>& path);

};

#endif // MAINWINDOW_HPP
