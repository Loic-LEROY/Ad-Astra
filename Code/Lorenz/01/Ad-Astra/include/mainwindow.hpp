#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>

#include "Graph.hpp"
#include "CustomGraphicsView.hpp"


class MainWindow : public QMainWindow {
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void selectStartVertex();
    void selectEndVertex();
    void findPath();
    void selectAlgorithm();


private:

    void loadGraph();
    void displayGraph();
    void drawPath(const std::vector<Vertex*>& path);
    
    void createActions();
    void createMenus();
    
    // Menu actions
    QAction *selectStartAction;
    QAction *selectEndAction;
    QAction *selectAlgorithmAction;
    QAction *findPathAction;

    CustomGraphicsView* graphicsView;
    QGraphicsScene* scene;

    // Graph data
    Graph graph;
    Vertex* startVertex;
    Vertex* endVertex;

    enum AlgorithmType { DIJKSTRA, ASTAR, BFS };
    AlgorithmType currentAlgorithm;

    
};

#endif // MAINWINDOW_HPP
