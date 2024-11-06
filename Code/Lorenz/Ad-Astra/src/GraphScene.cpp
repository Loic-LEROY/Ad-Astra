#include "GraphScene.hpp"
#include "EdgeItem.hpp"
#include <QGraphicsSceneMouseEvent>

GraphScene::GraphScene(QObject* parent)
    : QGraphicsScene(parent), graph(nullptr), startVertex(nullptr), endVertex(nullptr), startVertexLabel(nullptr),
    endVertexLabel(nullptr) {}

void GraphScene::setGraph(Graph* graph) {
    this->graph = graph;
    drawGraph();
}

void GraphScene::drawGraph() {
    if (!graph) return;

    // Clear existing items
    clear();
    vertexItems.clear();
    edgeItems.clear();

    // Determine scaling factors
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();

    for (const auto& pair : graph->vertices) {
        Vertex* v = pair.second;
        minX = std::min(minX, v->x);
        maxX = std::max(maxX, v->x);
        minY = std::min(minY, v->y);
        maxY = std::max(maxY, v->y);
    }

    double xRange = maxX - minX;
    double yRange = maxY - minY;

    // Scaling factors
    double sceneWidth = 2000.0;
    double sceneHeight = 2000.0;

    double scaleX = sceneWidth / xRange;
    double scaleY = sceneHeight / yRange;
    double scale = std::min(scaleX, scaleY);

    double offsetX = (sceneWidth - scale * xRange) / 2.0;
    double offsetY = (sceneHeight - scale * yRange) / 2.0;

    // Adjust vertices positions
    for (const auto& pair : graph->vertices) {
        Vertex* v = pair.second;

        double x = offsetX + (v->x - minX) * scale;
        double y = offsetY + (maxY - v->y) * scale; // Invert y-axis

        // Store adjusted coordinates
        v->x = x;
        v->y = y;

        VertexItem* vertexItem = new VertexItem(v);
        addItem(vertexItem);
        vertexItem->setZValue(2);
        vertexItems[v] = vertexItem;

        connect(vertexItem, &VertexItem::vertexClicked, this, &GraphScene::vertexSelected);

        // Optional: Display vertex ID
        // QGraphicsTextItem* textItem = addText(QString::number(v->id));
        // textItem->setPos(x + 4, y + 4);
        // textItem->setDefaultTextColor(Qt::blue);
    }

    // Draw edges
    for (Edge* e : graph->edges) {
        EdgeItem* edgeItem = new EdgeItem(e);
        addItem(edgeItem);
        edgeItem->setZValue(1);
        edgeItems[e] = edgeItem; // Store the mapping from Edge* to EdgeItem*
    }
}

void GraphScene::setStartVertex(Vertex* vertex) {
    if (startVertex && vertexItems.count(startVertex)) {
        if (startVertex != vertex) {
            vertexItems[startVertex]->setState(VertexItem::Normal);
            vertexItems[endVertex]->setState(VertexItem::Normal);

            if (startVertexLabel) {
                removeItem(startVertexLabel);
                delete startVertexLabel;
                startVertexLabel = nullptr;

                removeItem(endVertexLabel);
                delete endVertexLabel;
                endVertexLabel = nullptr;
            }
        }
    }
    startVertex = vertex;
    if (vertexItems.count(vertex)) {
        vertexItems[vertex]->setState(VertexItem::Start);

        // Add label for start vertex
        if (startVertexLabel) {

            removeItem(startVertexLabel);
            delete startVertexLabel;
        }
        startVertexLabel = addText(QString::number(vertex->id));
        startVertexLabel->setZValue(20);
        startVertexLabel->setDefaultTextColor(Qt::green);
        startVertexLabel->setPos(vertex->x + 5, vertex->y - 15);
    }
}

void GraphScene::setEndVertex(Vertex* vertex) {
    endVertex = vertex;
    if (vertexItems.count(vertex)) {
        vertexItems[vertex]->setState(VertexItem::End);

        // Add label for end vertex
        if (endVertexLabel) {
            removeItem(endVertexLabel);
            delete endVertexLabel;
        }
        endVertexLabel = addText(QString::number(vertex->id));
        endVertexLabel->setZValue(20);
        endVertexLabel->setDefaultTextColor(Qt::red);
        endVertexLabel->setPos(vertex->x + 5, vertex->y - 15);
    }
}

void GraphScene::clearSelection() {
    // Reset start vertex
    if (startVertex && vertexItems.count(startVertex)) {
        vertexItems[startVertex]->setState(VertexItem::Normal);
    }
    if (startVertexLabel) {
        removeItem(startVertexLabel);
        delete startVertexLabel;
        startVertexLabel = nullptr;
    }
    startVertex = nullptr;

    // Reset end vertex
    if (endVertex && vertexItems.count(endVertex)) {
        vertexItems[endVertex]->setState(VertexItem::Normal);
    }
    if (endVertexLabel) {
        removeItem(endVertexLabel);
        delete endVertexLabel;
        endVertexLabel = nullptr;
    }
    endVertex = nullptr;

    // Clear path highlights
    clearPathHighlight();
}

void GraphScene::highlightPath(const std::vector<Vertex*>& path) {
    clearPathHighlight(); // Clear any existing path highlighting

    for (size_t i = 0; i < path.size() - 1; ++i) {
        Vertex* u = path[i];
        Vertex* v = path[i + 1];

        // Find the edge between u and v
        Edge* edge = nullptr;
        for (Edge* e : u->edges) {
            if (e->destination == v) {
                edge = e;
                break;
            }
        }
        if (edge) {
            EdgeItem* edgeItem = edgeItems[edge];
            if (edgeItem) {
                edgeItem->setPen(QPen(Qt::blue, 2));
                highlightedItems.push_back(edgeItem);
                edgeItem->setZValue(10);
            }
        }

        // Highlight the vertices if they are not start or end vertices
        VertexItem* vertexItemU = vertexItems[u];
        if (vertexItemU && u != startVertex && u != endVertex) {
            vertexItemU->setState(VertexItem::Path);
            highlightedItems.push_back(vertexItemU);
        }
        VertexItem* vertexItemV = vertexItems[v];
        if (vertexItemV && v != startVertex && v != endVertex) {
            vertexItemV->setState(VertexItem::Path);
            highlightedItems.push_back(vertexItemV);
        }
    }
}


void GraphScene::clearPathHighlight() {
    // Reset the appearance of highlighted items
    for (QGraphicsItem* item : highlightedItems) {
        EdgeItem* edgeItem = dynamic_cast<EdgeItem*>(item);
        if (edgeItem) {
            edgeItem->setPen(QPen(Qt::gray));
            edgeItem->setZValue(1);
        }
        VertexItem* vertexItem = dynamic_cast<VertexItem*>(item);
        if (vertexItem) {
            if (vertexItem->getState() == VertexItem::Path) {
                vertexItem->setState(VertexItem::Normal);
                vertexItem->setZValue(2);
            }
        }
    }
    highlightedItems.clear();
}

