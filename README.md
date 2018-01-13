# SokobanSolver
This is a solver for the Sokoban game.
I made it, when I was in university, as part of a project in the class: Introduction to artificial intelligence.

In the solver the 3 methods are implemented: breadth-first, depth-first and A*.
The solve-time improves drastically, if executed on Linux, as opposed to Windows.

The solver is implemented using c++'s hash tables, unordered_map for the closed_set. For the A*, c++'s priority_queue is used. For uninformed search (BFS and DFS), linked_lists are used. 
