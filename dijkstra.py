## this file is the implementation of the Dijkstra's algorithm


import heapq
closed = {}
open = []

def Dijkstra(s0, sg, T):
    closed[s0] = 0
    heapq.heappush(open, (s0, 0))    
    while open:
        current_node, cost = heapq.heappop(open)
        if current_node == sg:
            return cost


