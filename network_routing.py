# this code is only for testing, remove before submission #
from heapq import *

class TestMinPriorityQueue:
    def __init__(self):
        self.data = []
    def make_queue(self, dist):
        for key in dist.keys():
            heappush(self.data, (dist[key], key))
    def pop_min(self):
        return heappop(self.data)
    def update_key(self, vertex, value):
        # O(n) operation only for testing purposes
        for i in range(len(self.data)):
            if self.data[i][1] == vertex:
                self.data[i] = (value, vertex)
        heapify(self.data)

###########################################################

def dijkstra(graph: dict[int, dict[int, float]], start: int, heap_type: str) -> dict[int, float]:
    dist = {}
    prev = {}
    for vertex in graph.keys():
        dist[vertex] = float('inf')
        prev[vertex] = None
    dist[start] = 0
    if heap_type == "test":
        H = TestMinPriorityQueue()
    else:
        raise NotImplementedError(f"No such heap type: {heap_type}")
    H.make_queue(dist)
    while H.data:
        u = H.pop_min()[1]
        for v in graph[u].keys():
            if dist[u] + graph[u][v] < dist[v]:
                dist[v] = dist[u] + graph[u][v]
                H.update_key(v, dist[v])
                prev[v] = u
    return (dist, prev)


def assemble_path(prev: dict[int, int], start: int, end: int) -> list[int]:
    path = []
    curr = end
    while curr != start:
        path.append(curr)
        curr = prev[curr]
    path.append(start)
    return list(reversed(path))  # TODO: check time complexity


def find_shortest_path_with_heap(
        graph: dict[int, dict[int, float]],
        source: int,
        target: int
) -> tuple[list[int], float]:
    """
    Find the shortest (least-cost) path from `source` to `target` in `graph`
    using the heap-based algorithm.

    Return:
        - the list of nodes (including `source` and `target`)
        - the cost of the path
    """
    dist, prev = dijkstra(graph, source, "test")
    return (assemble_path(prev, source, target), dist[target])



def find_shortest_path_with_array(
        graph: dict[int, dict[int, float]],
        source: int,
        target: int
) -> tuple[list[int], float]:
    """
    Find the shortest (least-cost) path from `source` to `target` in `graph`
    using the array-based (linear lookup) algorithm.

    Return:
        - the list of nodes (including `source` and `target`)
        - the cost of the path
    """
    dist, prev = dijkstra(graph, source, "test")
    return (assemble_path(prev, source, target), dist[target])
