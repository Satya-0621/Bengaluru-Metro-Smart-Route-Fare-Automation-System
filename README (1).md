## The Metro App

Develop a C++ program for the Delhi Metro Rail App, which takes input from the user regarding the source and destination stations on the Delhi Metro, and provides information about the shortest metro route between these two stations, along with the fare. The idea will be implemented using Graph and Heap data structures. The graph will be having nodes and edges. Nodes represent a metro station that will be containing certain information regarding that station like its name, its metro corridor, and the lines which it connects. Edges (the connection between two nodes) represent the distance between the two stations and the cost of each edge will be equal to the distance between the two of its connecting stations(nodes). The program uses graph and heap data structures to represent and calculate the metro routes and fares efficiently, primarily relying on the Dijkstra algorithm.Some Standard Template Library (STL) is used during development like vector,unordered_map,priority_queue,stack,etc.

## Algorithm Steps:

1. Define a Graph data structure in C++ to represent the Delhi Metro network. Each metro station is a node, containing information like its name, metro corridor, and the lines it connects. The connections between stations are represented as edges, with the edge weight representing the distance between stations.

2. Implement a Heap data structure (e.g., unordered_map) in C++ to efficiently manage the nodes during the Dijkstra algorithm-based shortest path calculations.

3. Take user input for the source and destination metro stations in C++.

4. Use Dijkstra's algorithm in C++ to find the shortest path between the source and destination stations.

5. Calculate the fare for the determined shortest path in C++ based on the total distance between stations. The fare calculation can be based on a predefined fare structure for the Delhi Metro.

6. Display the shortest metro route between the source and destination stations in C++, including the stations to be traversed.

7. Present the total fare for the journey in C++.

8. Provide a metro map in C++ to help commuters navigate and understand the metro network.

9. Ensure that the C++ application can run on various Integrated Development Environments (IDEs) like Visual Studio, Code::Blocks, or other popular C++ IDEs and can be easily used by individuals with elementary knowledge of C++ programming.
