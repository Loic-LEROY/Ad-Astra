#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <unordered_map>
#include <list>
#include <set>
#include <chrono>
#include "commify.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MapViewer; }
QT_END_NAMESPACE

class MapViewer : public QMainWindow {
    Q_OBJECT

public:
    MapViewer(QWidget *parent = nullptr);
    ~MapViewer();

    void addVertex(int id, double x, double y);
    void addEdge(int v1, int v2, double weight);
    void renderGraph();
    void renderPath(const std::list<int>& path, const std::set<int>& openSet, const std::set<int>& closedSet);
    Vertex getVertex(int id) const;

private slots:
    void on_findPathButton_clicked();
    void on_loadGraphButton_clicked();

private:
    Ui::MapViewer *ui;
    QGraphicsScene *scene;
    std::unordered_map<int, QGraphicsEllipseItem*> vertices;
    std::unordered_map<int, std::list<std::pair<int, double>>> adj;

    std::list<int> bfs(int start, int end);
    std::list<int> dijkstra(int start, int end);
    std::list<int> astar(int start, int end, std::set<int>& openSet, std::set<int>& closedSet);
    double computeDistance(const Vertex& v1, const Vertex& v2);
    void convertToCartesian(double lon, double lat, double& x, double& y);
    double heuristic(const Vertex& v1, const Vertex& v2);
};

#endif // MAPVIEWER_H