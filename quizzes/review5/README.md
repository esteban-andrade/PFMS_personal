Quiz 4
======

Part A
------

1) TASK: A class representing an undirected, weighted graph has been partially defined in [graph.h](./a/graph.h). Use your knowledge of the `std::map` STL to complete the methods `addVertex()`, `hasVertex()`, and `getVertices()`.

2) TASK: Compile the program, execute, and check that the program outputs all the graph's vertices as expected.

3) TASK: Complete the method `bfs()` in [graph.cpp](./a/graph.cpp) so that it performs a breadth first search on the entire graph. Check that the output of the BFS is as expected.

4) QUESTION: What is the worst case computational complexity of BFS? 
O(V+E)
5) QUESTION: Which algorithm(s) can find the shortest path between two vertices in a graph that has weights on edges? 
Dijkstra's algorithm 
Part B
------

1) TASK: In [main.cpp](./b/main.cpp) add the appropriate thread instruction(s) to ensure that `main()` waits for the threads to finalise.

2) TASK: Control access to the `std::queue` in `TDataBuffer` by adding an appropriate thread mechanism to the struct.

3) TASK: Use the mechanism in threads `fibonacci` and `printToTerminal`.

4) TASK: Implement the mechanism to make the `consumer` thread wait for new data?

5) TASK: Rather than making `TDataBuffer` variable `sequence` global, change it's scope so that both threads had access to this shared resource
