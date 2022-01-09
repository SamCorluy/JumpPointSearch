# Jump Point Search
A look into the works of the Jump Point Search algorithm
## Why?
since i have started studying game development, path finding has always intrigued me, so once i learned about an A* optimization I just had to dive in deeper and get to know as much as I could about it.
## How it works
This algorithm is an optimization to the A* pathfinding algorithm that works with uniform-cost grids, meaning that every node has 8 neighbors and that the cost of every horizontal/vertical path is 1 and every diagonal path has a cost of sqrt(2). The algorithm works in such a way that it can make jumps over straight lines rather than take small steps, in most cases the jump point search algorithm will wield results much faster.
