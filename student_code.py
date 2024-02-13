'''
    The idea of using min heap and how to calculate the g-value 
    between the current intersection and it's neighbour was from ChatGPT.
'''

import math
import heapq

def shortest_path(M, start, goal):
    print("shortest path called")

    frontier = [(0, start)]         # creating a min heap and adding the start node (with the lowest f-value)
    visited_nodes = set()           # set of visited nodes or explored intersections
    cost_to_each_node = {start: 0}  # dicitionary of g-values
    parent_nodes = {}               # dictionary of parent nodes

    # implementing the heuristic function (h-value)
    def heuristic_value(node):
        '''
            This function calculates the h-value (Euclidean distance) between
            an intersection and the goal.

            Args:
                    node: the intersection we want to calculate it's h-value to our
                    goal in the given map.
            
            output:
                    the estimated distance from the passed intersection to our goal.
        '''
        x1, y1 = M.intersections[node]  # x,y coordinates of the intersection
        x2, y2 = M.intersections[goal]  # x,y coordinates of the goal
        return math.sqrt(((x2 - x1) ** 2) + ((y2 - y1) ** 2))
    
    # implementing the shortest path length between two nodes (g-value)
    def path_node_neighbour(node_1, node_2):
        '''
            This function calculates the g-value (Euclidean distance) between
            two intersections.

            Args:
                    node: two intersections
            
            output:
                    the shortest path length between both passed intersections.
        '''
        x1 = M.intersections[node_1][0]     # x-coordinate of the current intersecion
        y1 = M.intersections[node_1][1]     # y-coordinate of the current intersecion
        x2 = M.intersections[node_2][0]     # x-coordinate of it's neighbour    
        y2 = M.intersections[node_2][1]     # y-coordinate of it's neighbour
        return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))

    # while the frontier is not empty
    while frontier:
        # selecting the node with the minimum f-value
        current_cost, current_node = heapq.heappop(frontier)

        # checking if the selected node is our goal
        if current_node == goal:
            # now we create a list for our path and starting from the goal
            path = [goal]
            # we iterate until we reach the start intersection
            while current_node != start:
                # moving to back to the parent of our current node
                current_node = parent_nodes[current_node]
                # adding that parent (the new current node) to our path list
                path.append(current_node)
            # we return our path but reversed, because our list starts from the goal and ends at the start intersection    
            return path[::-1]
        
        # after we visited the current intersection we add it to our set of visited_nodes
        visited_nodes.add(current_node)

        # expanding the current intersection and finding its neighbours
        for neighbour in M.roads[current_node]:
            # we check if that neighbour has been visited before or not
            if neighbour in visited_nodes:
                # if yes we leave it and continue in our loop
                continue

            # otherwise we calculate the cost (g-value) starting from start intersection until that neighbour through our current node.
            path_length = cost_to_each_node[current_node] + path_node_neighbour(current_node, neighbour)
            
            # if the neighbour hasn't been visited yet or the path length to that neighbour is lower than previous g-value of that neighbour
            if neighbour not in cost_to_each_node or path_length < cost_to_each_node[neighbour]:
                # now we add that neighbour and it's g-value to the g-values dictionary
                cost_to_each_node[neighbour] = path_length
                # now we calculate f = g + h
                f_value = path_length + heuristic_value(neighbour)

                # now we add that neighbour intersection to our priority queue frontier
                # because its min heap, the node with the lowest f-value will be the first tuple.
                heapq.heappush(frontier, (f_value, neighbour))
                # now we update the parent of that neighbour as the current node.
                parent_nodes[neighbour] = current_node
    
    return []