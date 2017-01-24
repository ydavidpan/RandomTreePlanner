# RandomTreePlanner

# robotic project

The Random Tree algorithm is a simple sampling-based method that grows a tree in the configuration space of the robot. The algorithm is loosely based on a random walk of the configuration space and operates using the following strategy:

1. Select a random configuration a from the existing Random Tree.
2. Sample a random configuration b in the configuration space. With a small probability select the goal configuration instead of a random one.
3. Check whether the straight-line path between a and b in the C-space is valid (i.e., collision free). If the path is valid, add the path to the Random Tree
4. Repeat steps 1-3 until the goal state is added to the tree. Extract the final motion plan from the tree. The tree is initialized with the starting configuration of the robot.
