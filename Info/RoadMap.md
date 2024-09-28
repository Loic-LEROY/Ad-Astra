### **Phase 1: Setup & Foundations**
Objective: Get familiar with C++ basics, tools, and the project structure. Divide simple tasks to build confidence in the language and establish a common understanding.

#### Key Tasks:
1. **Set Up Development Environment** (everyone):
   - Install C++ compilers (GCC/Clang), set up Visual Studio Code, Qt Creator for the UI, and install necessary libraries (STL for algorithms, Qt for UI).
   - Explore small C++ programs to get familiar with syntax (focus on basics: classes, containers like `vector`, `set`, `queue`, `priority_queue`).

2. **Basic Graph Class Implementation** (Person A and B):
   - Implement the `Graph`, `Vertex`, and `Edge` classes. Use the CSV graph file format from the project guidelines.
   - Person A (who is stronger in programming) can focus on implementing the file-reading system to load the graph structure into memory.
   - Person B can implement basic methods for adjacency list creation, and helper functions like adding vertices and edges.

3. **C++ Learning** (Person C):
   - Focus on learning the fundamentals of C++: classes, pointers, dynamic memory, and basic I/O. Work with small graph programs to build familiarity.

#### Milestones:
- Everyone should be comfortable with C++ classes and containers by the end of this phase.
- Basic graph structure in place and the ability to load a graph from a file.

---

### **Phase 2: Algorithm Implementations**
Objective: Implement and test BFS and Dijkstra algorithms. Continue leveling up skills with more complex concepts.

#### Key Tasks:
1. **Breadth-First Search (BFS)**:
   - **Person A** will implement BFS, focusing on exploring graph traversal.
   - **Person C** will assist by writing test cases, running simple BFS examples, and verifying results.

2. **Dijkstra’s Algorithm**:
   - **Person B** can start implementing Dijkstra’s algorithm using a priority queue (heap).
   - **Person A** can review and test, helping ensure correctness and efficiency.

3. **Intermediate C++ Skills for Person C**:
   - Person C should now focus on working with more complex STL containers like `priority_queue` and learning about time complexity of algorithms.

#### Milestones:
- BFS and Dijkstra algorithms implemented and tested on small graphs.
- Person C has an intermediate understanding of C++ and can contribute to smaller coding tasks like writing helper functions or running tests.

---

### **Phase 3: A-Star Algorithm & Testing**
Objective: Implement A-Star, focus on improving algorithms, and start testing on real-world data.

#### Key Tasks:
1. **A-Star Algorithm** (Person A and B):
   - Both **Person A and B** can collaborate to implement A-Star, with Person A focusing on integrating the heuristic for the shortest path, while Person B ensures the priority queue works correctly and efficiently.

2. **Testing and Debugging** (Person C):
   - Person C can help test the algorithms using the graph datasets, ensuring that the algorithms return the correct results. Debugging output and validating against expected results are key here.

#### Milestones:
- A-Star algorithm working and tested.
- All three algorithms (BFS, Dijkstra, A-Star) ready and functioning well on test datasets.

---

### **Phase 4: User Interface**
Objective: Build and integrate the graphical interface using Qt to display the graph and paths computed by the algorithms.

#### Key Tasks:
1. **Graph Display & UI Design** (Person A and B):
   - **Person A** can focus on integrating the algorithms with the UI, allowing the user to select start and end vertices and choose an algorithm (BFS, Dijkstra, or A-Star).
   - **Person B** can handle rendering the graph using the Cartesian coordinates, following the project’s guidelines (using Qt's graphic libraries).

2. **Time Measurement & Results Display** (Person C):
   - **Person C** can focus on implementing the time measurement for each algorithm (using `std::chrono`) and ensuring that results (path, length, time) are displayed in a clean format in the console or UI.

#### Milestones:
- A functional UI that allows users to select start and end points and displays the path visually on the map.
- Complete time and performance metrics for all algorithms displayed.

---

### **Phase 5: Final Testing & Polishing**
Objective: Polish the code, ensure stability, and prepare the project for submission.

#### Key Tasks:
1. **Final Debugging and Performance Testing** (everyone):
   - Test the application on larger graphs to ensure that the algorithms perform within the required time (less than 1 second for the worst-case path).

2. **Report Writing** (Person A and B):
   - Prepare a report with code snippets and explanations of key sections (like BFS, Dijkstra, A-Star implementations).

3. **Submission Preparation** (everyone):
   - Ensure the project builds and runs on a clean environment (no hardcoded paths or dependencies).

---

### **Task Distribution Based on Skill Levels:**
- **Person A** (most experienced): Complex algorithm implementation (A-Star), handling UI integration, and debugging.
- **Person B** (intermediate): Implement Dijkstra, assist in UI development and testing, improve code performance.
- **Person C** (beginner): Assist in testing, learning through implementing simpler functions (e.g., BFS helper functions, test cases, performance measurements).

