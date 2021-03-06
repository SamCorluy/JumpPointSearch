# Jump Point Search
A look into the works of the Jump Point Search algorithm
## Why?
Since I started studying game development, path finding has always intrigued me, so once i learned about an A* optimization I just had to dive in deeper and get to know as much as I could about it.
## What?
This algorithm is an optimization to the A* pathfinding algorithm that works with uniform-cost grids, meaning that every node has 8 neighbors and that the cost of every horizontal/vertical path is 1 and every diagonal path has a cost of sqrt(2). Another important note is that every neighbor is either traversable, or not. The algorithm works in such a way that it can make jumps over straight lines rather than take small steps, in most cases the jump point search algorithm will wield results much faster.
## Visual representation:
Jump point search:

![image](https://user-images.githubusercontent.com/96989324/150380782-fc30cc3b-863a-417c-bd5c-f8d459f89fc1.png)

A*:

![image](https://user-images.githubusercontent.com/96989324/150380936-667bc0ea-d21c-4e42-abfa-a865dedb4d43.png)


## How?
This optimization uses two new algorithms:
- An algorithm to Identify it's successors:

  this function requires 3 inputs:
  * the current node
  * the start node
  * the goal node

  The algorithm loops over all neighbors applying the jump algorithm, if it determines that a neighbor is a jump point, it adds this neighbor to the successorsList
    
- An algorithm to jump:

  this function requires 4 inputs:
  * the initial node
  * the direction
  * the start node
  * the goal node

  a new node gets calculated by adding a direction to the initial node, if this node is an obstacle or not part of the grid, null will be returned, if this node is the goal node however, the goalnode will be returned. If neither of these are the case, the node's neighbors get checked to make sure it isn't a forced neighbor(a forced neighbor is directly linked to a blocked node), if it turns out to be a forced neighbor, the node gets returned. and finally, if the function still hasn't returned a value, the function will call itself, replacing the initial node with the calculated node

the first algorithm is then used to select your next node instead of looking through adjacent nodes like A* does.
## Other A* optimizations?
* Orthogonal jump point search
## Implementation
Since I already had to make an A* implementation in our school's framework, I decided to use that as a base. However, since I highly underestimated the work that goes into implementing such an algorithm, I sadly didn't have the time to complete it.
## Conclusion
All in all the JPS algorithm is a neat little optimization to the A* pathfinding system, wielding similar results at faster speeds. The drawback being of course that it relies on uniform-cost grids, if you would want to optimize your A* algorithm for any other grid you would need to find a different algorithm to use!
