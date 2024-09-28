# Graph Search Algorithms Project

![Banner](banner.jpg)

## Overview
This project implements and visualizes various graph search algorithms, including Breadth-First Search (BFS), Dijkstra's algorithm, and A-Star (A*) algorithm. The project is built using Qt for the graphical user interface and C++ for the algorithm implementations.

## Features
- **Graph Visualization**: Display a graph with vertices and edges using Cartesian coordinates.
- **Path Finding**: Compute and display the shortest path between two vertices using BFS, Dijkstra, or A-Star algorithms.
- **User Interface**: Select start and end vertices, choose the algorithm, and visualize the path.
- **Performance Measurement**: Measure and display the computation time for each algorithm.
- **Responsive UI**: Supports zooming and panning for fluid interaction.

## Requirements
- Qt 5 or higher
- C++11 or higher

## Installation
1. **Clone the repository**:
    ```sh
    git clone https://github.com/yourusername/graph_search_algorithms.git
    cd graph_search_algorithms
    ```

2. **Open the project in Qt Creator**:
    - Open `graph_search_algorithms.pro` or `CMakeLists.txt` in Qt Creator.

3. **Build the project**:
    - Click on the "Build" button in Qt Creator.

4. **Run the project**:
    - Click on the "Run" button in Qt Creator.

## Usage
1. **Load Graph Data**:
    - Click on the "Load Graph" button to load a graph from a CSV file. The CSV file should contain vertex and edge data.

2. **Select Start and End Vertices**:
    - Enter the IDs of the start and end vertices in the respective input fields.

3. **Choose Algorithm**:
    - Select one of the algorithms (BFS, Dijkstra, A-Star) from the dropdown menu.

4. **Find Path**:
    - Click on the "Find Path" button to compute and display the shortest path.

5. **View Results**:
    - The computed path, open set, and closed set will be displayed on the graph.
    - The path details, including computation time, number of vertices in the path, length of the path, and total number of searched vertices, will be printed to the console.

## File Structure
graph_search_algorithms/ ├── src/ │ ├── main.cpp │ ├── mapviewer.cpp │ ├── mapviewer.h │ ├── mapviewer.ui │ ├── commify.h │ └── ... (other necessary files) ├── resources/ │ └── example_graph.csv ├── CMakeLists.txt ├── graph_search_algorithms.pro └── README.md


## Graph Data Format
The CSV file should contain the following columns:
- Vertex ID, Longitude, Latitude
- Edge Start Vertex ID, Edge End Vertex ID, Weight (optional)

Example:
1, -77.0365, 38.8977 2, -77.0091, 38.8895 1, 2, 100


## Algorithms
### Breadth-First Search (BFS)
- Explores all vertices at the present depth level before moving on to vertices at the next depth level.
- Suitable for unweighted graphs.

### Dijkstra's Algorithm
- Finds the shortest path from a start vertex to all other vertices in a weighted graph.
- Uses a priority queue to explore the vertex with the smallest known distance.

### A-Star (A*) Algorithm
- Uses a heuristic to improve the efficiency of the shortest path search.
- Combines the actual distance from the start vertex with an estimated distance to the end vertex.

## Performance Measurement
- The computation time for each algorithm is measured using `std::chrono` and displayed in microseconds.
- The `Commify` class is used to format the time with comma separators for better readability.

## Example Output
Path: 1 2 INFO: path calculated in 95,871us Number of vertices in path: 2 Length of path: 100 km Total number of searched vertices: 2


## Contributors
- Name1
- Name2
- Name3

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgements
- Qt for the graphical user interface.
- C++ Standard Library for data structures and algorithms.
- [cppreference.com](http://en.cppreference.com/w/cpp/chrono) for documentation on `std::chrono`.

## Contact
For any questions or suggestions, please contact us at [your-email@example.com].