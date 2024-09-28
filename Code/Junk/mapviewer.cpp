#include "mapviewer.h"
#include "ui_mapviewer.h"
#include <iostream>
#include <chrono>
#include <QFileDialog>
#include <QTextStream>

MapViewer::MapViewer(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MapViewer)
    , scene(new QGraphicsScene(this))
{
    ui->setupUi(this);
    ui->graphicsView->setScene(scene);
}

MapViewer::~MapViewer() {
    delete ui;
}

void MapViewer::addVertex(int id, double x, double y) {
    QGraphicsEllipseItem* vertex = scene->addEllipse(x - 2, y - 2, 4, 4, QPen(), QBrush(Qt::SolidPattern));
    vertices[id] = vertex;
}

void MapViewer::addEdge(int v1, int v2, double weight) {
    if (vertices.find(v1) != vertices.end() && vertices.find(v2) != vertices.end()) {
        QGraphicsLineItem* edge = scene->addLine(vertices[v1]->rect().center().x(), vertices[v1]->rect().center().y(),
                                                 vertices[v2]->rect().center().x(), vertices[v2]->rect().center().y());
        adj[v1].emplace_back(v2, weight);
        adj[v2].emplace_back(v1, weight); // Assuming undirected graph
    }
}

void MapViewer::renderGraph() {
    // Example vertices and edges
    addVertex(1, 100, 100);
    addVertex(2, 200, 200);
    addEdge(1, 2, 100);
}

void MapViewer::renderPath(const std::list<int>& path, const std::set<int>& openSet, const std::set<int>& closedSet) {
    // Clear previous path
    scene->clear();

    // Render open set
    for (int v : openSet) {
        vertices[v]->setBrush(QBrush(Qt::blue));
    }

    // Render closed set
    for (int v : closedSet) {
        vertices[v]->setBrush(QBrush(Qt::purple));
    }

    // Render path
    for (auto it = path.begin(); it != std::prev(path.end()); ++it) {
        int v1 = *it;
        int v2 = *std::next(it);
        scene->addLine(vertices[v1]->rect().center().x(), vertices[v1]->rect().center().y(),
                       vertices[v2]->rect().center().x(), vertices[v2]->rect().center().y(), QPen(Qt::red, 2));
    }
}

void MapViewer::on_findPathButton_clicked() {
    int start = ui->startPointInput->text().toInt();
    int end = ui->endPointInput->text().toInt();
    QString algorithm = ui->algorithmComboBox->currentText();

    std::list<int> path;
    std::set<int> openSet, closedSet;
    auto startTime = std::chrono::high_resolution_clock::now();

    if (algorithm == "BFS") {
        path = bfs(start, end);
    } else if (algorithm == "Dijkstra") {
        path = dijkstra(start, end);
    } else if (algorithm == "A-Star") {
        path = astar(start, end, openSet, closedSet);
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

    renderPath(path, openSet, closedSet);

    // Calculate path length
    double pathLength = 0;
    for (auto it = path.begin(); it != std::prev(path.end()); ++it) {
        int v1 = *it;
        int v2 = *std::next(it);
        pathLength += computeDistance(vertices[v1], vertices[v2]);
    }

    // Print path and time to console
    std::cout << "Path: ";
    for (int v : path) {
        std::cout << v << " ";
    }
    std::cout << "\nINFO: path calculated in " << Commify(duration.count()) << "us\n";
    std::cout << "Number of vertices in path: " << path.size() << "\n";
    std::cout << "Length of path: " << pathLength << " km\n";
    std::cout << "Total number of searched vertices: " << openSet.size() + closedSet.size() << "\n";
}

void MapViewer::on_loadGraphButton_clicked() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Graph File"), "", tr("CSV Files (*.csv);;All Files (*)"));
    if (fileName.isEmpty()) {
        return;
    }

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        return;
    }

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList fields = line.split(',');

        if (fields.size() >= 3) {
            int id = fields[0].toInt();
            double lon = fields[1].toDouble();
            double lat = fields[2].toDouble();
            addVertex(id, lon, lat);
        }

        if (fields.size() >= 5) {
            int v1 = fields[0].toInt();
            int v2 = fields[1].toInt();
            double weight = fields[4].toDouble();
            addEdge(v1, v2, weight);
        }
    }
}

std::list<int> MapViewer::bfs(int start, int end) {
    // Implement BFS algorithm
    return {};
}

std::list<int> MapViewer::dijkstra(int start, int end) {
    // Implement Dijkstra algorithm
    return {};
}

std::list<int> MapViewer::astar(int start, int end, std::set<int>& openSet, std::set<int>& closedSet) {
    auto cmp = [](const std::pair<double, int>& left, const std::pair<double, int>& right) { return left.first > right.first; };
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, decltype(cmp)> active_queue(cmp);
    std::unordered_map<int, double> g_score;
    std::unordered_map<int, double> f_score;
    std::unordered_map<int, int> parent;

    for (auto& [id, vertex] : vertices) {
        g_score[id] = std::numeric_limits<double>::max();
        f_score[id] = std::numeric_limits<double>::max();
    }
    g_score[start] = 0;
    f_score[start] = heuristic(vertices[start], vertices[end]);
    active_queue.emplace(f_score[start], start);
    openSet.insert(start);

    while (!active_queue.empty()) {
        int current = active_queue.top().second;
        active_queue.pop();
        openSet.erase(current);
        closedSet.insert(current);

        if (current == end) {
            break;
        }

        for (const auto& [next, weight] : adj[current]) {
            double tentative_g = g_score[current] + weight;
            if (tentative_g < g_score[next]) {
                parent[next] = current;
                g_score[next] = tentative_g;
                f_score[next] = g_score[next] + heuristic(vertices[next], vertices[end]);
                active_queue.emplace(f_score[next], next);
                openSet.insert(next);
            }
        }
    }

    std::list<int> path;
    for (int v = end; v != start; v = parent[v]) {
        path.push_front(v);
    }
    path.push_front(start);
    return path;
}

double MapViewer::computeDistance(const Vertex& v1, const Vertex& v2) {
    return std::sqrt(std::pow(v1.x - v2.x, 2) + std::pow(v1.y - v2.y, 2));
}

void MapViewer::convertToCartesian(double lon, double lat, double& x, double& y) {
    // Local Mercator projection
    x = lon * 20037508.34 / 180;
    y = std::log(std::tan((90 + lat) * M_PI / 360)) / (M_PI / 180);
    y = y * 20037508.34 / 180;
}

double MapViewer::heuristic(const Vertex& v1, const Vertex& v2) {
    return computeDistance(v1, v2);
}