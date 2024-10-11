# this code is only for testing, remove before submission #
from heapq import *

class TestMinPriorityQueue:
    data = []
    def make_queue(self, dist):
        for key in dist.keys():
            heappush(self.data, (dist[key], key))
    def pop_min(self):
        return heappop(self.data)
    def update_key(self, vertex, value):
        heappush(self.data, (value, vertex))
    


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
        raise ValueError(f"heap_type {heap_type} not recognized.")
    while H.data:
        u = H.pop_min()[1]
        for v in graph[u].keys():
            if dist[u] + graph[u][v] < dist[v]:
                dist[v] = dist[u] + graph[u][v]
                H.update_key(v, dist[v])
                prev[v] = u
    return (dist, prev)


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
