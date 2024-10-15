# this code is only for testing, comment before submission #
# from heapq import *

# class TestMinPriorityQueue:
#     def __init__(self) -> None:
#         self.data = []
#     def make_queue(self, dist) -> None:
#         for key in dist.keys():
#             heappush(self.data, (dist[key], key))
#     def pop_min(self) -> tuple[float, int]:
#         return heappop(self.data)
#     def decrease_key(self, vertex, value) -> None:
#         # O(n) operation only for testing purposes
#         for i in range(len(self.data)):
#             if self.data[i][1] == vertex:
#                 self.data[i] = (value, vertex)
#         heapify(self.data)

###########################################################

class ArrayMinPriorityQueue:
    def __init__(self) -> None:
        self.data = {}
    def make_queue(self, dist) -> None:
        self.data = dist.copy()  # O(n) operation
    def pop_min(self) -> tuple[float, int]:
        min_element = min(self.data, key=lambda x: self.data[x])
        return (self.data.pop(min_element), min_element)
    def decrease_key(self, vertex, value) -> None:
        self.data[vertex] = value


class HeapMinPriorityQueue:
    def __init__(self):
        self.data = []
        self.pointers = {}  # of form {vertex: index in data}
    def make_queue(self, dist) -> None:
        for key in dist.keys():
            # insert procedure
            self.data.append((dist[key], key))
            self.pointers[key] = len(self.data) - 1
            curr = len(self.data) - 1
            while self.data[curr] < self.data[self.parent(curr)]:
                self.swap(curr, self.parent(curr))
                curr = self.parent(curr)
    def pop_min(self) -> tuple[float, int]:
        self.swap(0, len(self.data) - 1)
        minimum = self.data.pop()
        curr = 0
        while (self.left(curr) < len(self.data) and self.data[curr] > self.data[self.left(curr)]) or (self.right(curr) < len(self.data) and self.data[curr] > self.data[self.right(curr)]):
            if self.right(curr) >= len(self.data) or self.data[self.left(curr)] <= self.data[self.right(curr)]:
                self.swap(curr, self.left(curr))
                curr = self.left(curr)
            elif self.left(curr) >= len(self.data) or self.data[self.left(curr)] > self.data[self.right(curr)]:
                self.swap(curr, self.right(curr))
                curr = self.right(curr)
        return minimum
    def decrease_key(self, vertex, value) -> None:
        self.data[self.pointers[vertex]] = (value, vertex)
        curr = self.pointers[vertex]
        while self.data[curr] < self.data[self.parent(curr)]:
                self.swap(curr, self.parent(curr))
                curr = self.parent(curr)
    
    def swap(self, index1: int, index2: int) -> None:
        self.data[index1], self.data[index2] = self.data[index2], self.data[index1]
        self.pointers[self.data[index1][1]], self.pointers[self.data[index2][1]] = self.pointers[self.data[index2][1]], self.pointers[self.data[index1][1]]
    def parent(self, index: int) -> int:
        return index // 2
    def left(self, index: int) -> int:
        return 2 * index
    def right(self, index: int) -> int:
        return (2 * index) + 1


def dijkstra(graph: dict[int, dict[int, float]], start: int, pq_type: str) -> dict[int, float]:
    dist = {}
    prev = {}
    for vertex in graph.keys():
        dist[vertex] = float('inf')
        prev[vertex] = None
    dist[start] = 0

    if pq_type == "array":
        H = ArrayMinPriorityQueue()
    elif pq_type == "heap":
        H = HeapMinPriorityQueue()
    # elif pq_type == "test":
    #     H = TestMinPriorityQueue()
    else:
        raise NotImplementedError(f"No such priority queue type: {pq_type}")
    
    H.make_queue(dist)
    while H.data:
        u = H.pop_min()[1]
        for v in graph[u].keys():
            if dist[u] + graph[u][v] < dist[v]:
                dist[v] = dist[u] + graph[u][v]
                H.decrease_key(v, dist[v])
                prev[v] = u
    return (dist, prev)


def assemble_path(prev: dict[int, int], start: int, end: int) -> list[int]:
    # O(V) operation, shouldn't cause any issues
    path = []
    curr = end
    while curr != start:
        path.append(curr)
        curr = prev[curr]
    path.append(start)
    return list(reversed(path))


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
    dist, prev = dijkstra(graph, source, "heap")
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
    dist, prev = dijkstra(graph, source, "array")
    return (assemble_path(prev, source, target), dist[target])

# # tests
# testQueue = TestMinPriorityQueue()
# arrayQueue = ArrayMinPriorityQueue()
# heapQueue = HeapMinPriorityQueue()
# test_data = {0: .5, 1: .4, 2: .3, 3: .2, 4: .1}
# testQueue.make_queue(test_data)
# arrayQueue.make_queue(test_data)
# heapQueue.make_queue(test_data)
# print("test:", testQueue.data)
# print("heap:", heapQueue.data)
# print("ref:", heapQueue.pointers)
# while testQueue.data:
#     print("test:", testQueue.pop_min())
#     print("heap:", heapQueue.pop_min())
