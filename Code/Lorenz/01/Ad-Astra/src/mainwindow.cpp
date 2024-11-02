#include "mainwindow.hpp"
#include "DijkstraAlgorithm.hpp"

#include <QMessageBox>
#include <QVBoxLayout>
#include <QInputDialog>
#include <QElapsedTimer>
#include <QCoreApplication>
#include <QMenuBar>
#include <QMenu>
#include <QAction>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), graphicsView(new CustomGraphicsView(this)), scene(new QGraphicsScene(this)), startVertex(nullptr), endVertex(nullptr), currentAlgorithm(DIJKSTRA) {
    setCentralWidget(graphicsView);
    graphicsView->setScene(scene);

    setCentralWidget(graphicsView);
    graphicsView->setScene(scene);

    loadGraph();
    displayGraph();

    // Create the menu bar and actions
    createActions();
    createMenus();
}

void MainWindow::loadGraph() {
    if (!graph.loadFromFile("C:/Users/cazau/OneDrive/Documents/Ad-Astra/resources/washington.txt")) {
        QMessageBox::critical(this, "Error", "Failed to load graph data.");
    } else {
        QMessageBox::information(this, "Success", "Graph data loaded successfully.");
    }
}

void MainWindow::displayGraph() {
    // Determine the scaling factors
    double minLon = std::numeric_limits<double>::max();
    double maxLon = std::numeric_limits<double>::lowest();
    double minLat = std::numeric_limits<double>::max();
    double maxLat = std::numeric_limits<double>::lowest();

    for (const auto& pair : graph.vertices) {
        Vertex* v = pair.second;
        if (v->longitude < minLon) minLon = v->longitude;
        if (v->longitude > maxLon) maxLon = v->longitude;
        if (v->latitude < minLat) minLat = v->latitude;
        if (v->latitude > maxLat) maxLat = v->latitude;
    }

    double lonRange = maxLon - minLon;
    double latRange = maxLat - minLat;

    // Map vertices to scene coordinates
    for (const auto& pair : graph.vertices) {
        Vertex* v = pair.second;
        v->x = ((v->longitude - minLon) / lonRange) * 800; // Width of scene
        v->y = ((maxLat - v->latitude) / latRange) * 600; // Height of scene (inverted y-axis)
        scene->addEllipse(v->x-1, v->y-1, 2, 2, QPen(QColor(0x4e5051)), QBrush(QColor(0x4e5051)));
    }

    // Draw edges
    for (Edge* e : graph.edges) {
        scene->addLine(e->source->x, e->source->y, e->destination->x, e->destination->y, QPen(QColor(0xf1f6fc)));
    }
}

void MainWindow::selectStartVertex() {
    bool ok;
    int id = QInputDialog::getInt(this, "Select Start Vertex", "Vertex ID:", 0, 0, INT_MAX, 1, &ok);
    if (ok && graph.vertices.count(id)) {
        startVertex = graph.vertices[id];
        QMessageBox::information(this, "Start Vertex Selected", "Start vertex set.");
    } else {
        QMessageBox::warning(this, "Invalid Vertex", "The vertex ID is invalid.");
    }
}

void MainWindow::selectEndVertex() {
    bool ok;
    int id = QInputDialog::getInt(this, "Select End Vertex", "Vertex ID:", 0, 0, INT_MAX, 1, &ok);
    if (ok && graph.vertices.count(id)) {
        endVertex = graph.vertices[id];
        QMessageBox::information(this, "End Vertex Selected", "End vertex set.");
    } else {
        QMessageBox::warning(this, "Invalid Vertex", "The vertex ID is invalid.");
    }
}

void MainWindow::findPath() {
    if (!startVertex || !endVertex) {
        QMessageBox::warning(this, "Vertices Not Selected", "Please select both start and end vertices.");
        return;
    }

    PathfindingAlgorithm* algorithm = nullptr;

    switch (currentAlgorithm) {
        case DIJKSTRA:
            algorithm = new DijkstraAlgorithm();
            break;
        // case ASTAR:
        //     algorithm = new AStarAlgorithm();
        //     break;
        // case BFS:
        //     algorithm = new BFSAlgorithm();
        //     break;
    }

    if (algorithm) {
        QElapsedTimer timer;
        timer.start();
        std::vector<Vertex*> path = algorithm->findPath(startVertex, endVertex);
        qint64 elapsed = timer.elapsed();
        delete algorithm;

        if (path.empty()) {
            QMessageBox::information(this, "No Path Found", "No path exists between the selected vertices.");
        } else {
            drawPath(path);
            QMessageBox::information(this, "Path Found", QString("Path found in %1 ms").arg(elapsed));
        }
    }
}

void MainWindow::drawPath(const std::vector<Vertex*>& path) {
    // Clear previous path (if any)
    scene->clear();
    displayGraph(); // Redraw the graph

    // Draw the path
    for (size_t i = 0; i < path.size() - 1; ++i) {
        scene->addLine(
            path[i]->x, path[i]->y,
            path[i + 1]->x, path[i + 1]->y,
            QPen(Qt::red, 2)
        );
    }
}

void MainWindow::selectAlgorithm() {
    QStringList algorithms = {"Dijkstra", "A*", "BFS"};
    bool ok;
    QString selected = QInputDialog::getItem(this, "Select Algorithm", "Algorithm:", algorithms, 0, false, &ok);

    if (ok && !selected.isEmpty()) {
        if (selected == "Dijkstra") {
            currentAlgorithm = DIJKSTRA;
        } else if (selected == "A*") {
            currentAlgorithm = ASTAR;
        } else if (selected == "BFS") {
            currentAlgorithm = BFS;
        }
        QMessageBox::information(this, "Algorithm Selected", "Algorithm set to " + selected);
    }
}

void MainWindow::createActions() {
    // Action to select the start vertex
    selectStartAction = new QAction(tr("&Select Start Vertex"), this);
    connect(selectStartAction, &QAction::triggered, this, &MainWindow::selectStartVertex);

    // Action to select the end vertex
    selectEndAction = new QAction(tr("&Select End Vertex"), this);
    connect(selectEndAction, &QAction::triggered, this, &MainWindow::selectEndVertex);

    // Action to select the algorithm
    selectAlgorithmAction = new QAction(tr("Select &Algorithm"), this);
    connect(selectAlgorithmAction, &QAction::triggered, this, &MainWindow::selectAlgorithm);

    // Action to find the path
    findPathAction = new QAction(tr("&Find Path"), this);
    connect(findPathAction, &QAction::triggered, this, &MainWindow::findPath);
}

void MainWindow::createMenus() {
    // Create the menu bar
    QMenuBar *menuBar = this->menuBar();

    // Create a menu for actions
    QMenu *actionMenu = menuBar->addMenu(tr("&Actions"));

    // Add actions to the menu
    actionMenu->addAction(selectStartAction);
    actionMenu->addAction(selectEndAction);
    actionMenu->addAction(selectAlgorithmAction);
    actionMenu->addSeparator();
    actionMenu->addAction(findPathAction);
}