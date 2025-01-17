# Green-Route-Optimizer

Creating a Green Route Optimizer involves calculating the most eco-friendly driving routes by minimizing carbon emissions. This program will use a graph-based approach to model a road network and find the optimal path using Dijkstra's algorithm with a custom cost function. Here’s how you can implement it in Python.

```python
import heapq

class Graph:
    def __init__(self):
        self.edges = {}

    def add_edge(self, from_node, to_node, eco_cost):
        if from_node in self.edges:
            self.edges[from_node].append((to_node, eco_cost))
        else:
            self.edges[from_node] = [(to_node, eco_cost)]

    def get_edges(self, node):
        return self.edges.get(node, [])

def dijkstra(graph, start):
    """
    Compute the shortest paths from the start node to all other nodes using Dijkstra's algorithm.
    The cost function is based on eco_cost which is an indicator of carbon emissions.
    """
    queue = [(0, start)]
    seen = set()
    min_cost = {start: 0}
    came_from = {start: None}
    
    while queue:
        (cost, node) = heapq.heappop(queue)
        
        if node in seen:
            continue

        seen.add(node)
        
        for to_node, edge_cost in graph.get_edges(node):
            new_cost = cost + edge_cost
            if to_node not in min_cost or new_cost < min_cost[to_node]:
                min_cost[to_node] = new_cost
                heapq.heappush(queue, (new_cost, to_node))
                came_from[to_node] = node

    return min_cost, came_from

def reconstruct_path(came_from, start, end):
    """Reconstruct the path from start to end using the came_from dictionary."""
    path = []
    at = end
    
    if at not in came_from:
        return []

    while at is not None:
        path.append(at)
        at = came_from[at]

    path.reverse()

    if path[0] == start:
        return path
    else:
        return []

def main():
    graph = Graph()
    
    # Add edges to the graph. The `eco_cost` here is a simplified proxy for carbon emissions.
    # Replace with real data as needed.
    graph.add_edge('A', 'B', 5)
    graph.add_edge('A', 'C', 10)
    graph.add_edge('B', 'C', 2)
    graph.add_edge('B', 'D', 3)
    graph.add_edge('C', 'D', 1)
    
    start = 'A'
    end = 'D'
    
    try:
        min_cost, came_from = dijkstra(graph, start)
        path = reconstruct_path(came_from, start, end)
        
        if path:
            print(f"The eco-friendly path from {start} to {end} is: {' -> '.join(path)} with a cost of {min_cost[end]}")
        else:
            print(f"No path exists from {start} to {end}.")
    
    except KeyError as e:
        print(f"An error occurred: {e}, this may be due to a missing node in the graph.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == '__main__':
    main()
```

### How It Works:

1. **Graph Representation**: The `Graph` class uses an adjacency list to store connected nodes and their respective eco-costs.

2. **Dijkstra's Algorithm**: This implementation of Dijkstra's algorithm finds the shortest path in terms of eco-cost, simulating carbon emissions. The node with the lowest cumulative cost is explored first.

3. **Reconstruction of the Path**: After running Dijkstra's algorithm, we use the `came_from` dictionary to backtrack from the destination node to the start, reconstructing the path.

4. **Error Handling**: The program handles missing nodes with KeyError and catches unexpected errors gracefully with a generic Exception.

This example uses fictional data and a simplified model of carbon emissions, reflected by `eco_cost`. For real-world applications, actual emission data and road information will need to be integrated into the graph model.