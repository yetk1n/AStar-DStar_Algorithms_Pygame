import time
import random
from copy import deepcopy

from pygame import init
from agent import Agent


#  use whichever data structure you like, or create a custom one
import queue
import heapq
from collections import deque


"""
  you may use the following Node class
  modify it if needed, or create your own
"""
class Node():
    
    def __init__(self, parent_node, level_matrix, player_row, player_column, depth, h_value, f_value):
        self.parent_node = parent_node
        self.level_matrix = level_matrix
        self.player_row = player_row
        self.player_col = player_column
        self.depth = depth
        self.h = h_value
        self.f = f_value
        
    
    def __lt__(self, other):
        return self.depth + self.h < other.depth + other.h



class PriorityQueue: 
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]


class AStarAgent(Agent):

    def __init__(self):
        super().__init__()
        
        #  g cost in A*
        #    IMPORTANT NOTE!!!
        #please fill values inside this array
        #as you perform the A* search!
        self.g_values = []
        
        
        #  a large enough value for initializing g values at the start
        self.INFINITY_COST = 2**10
        

    def find_f_value(self, current):
        return current.f
    
    #  finds apple's position in the given level matrix
    #return a tuple of (row, column)
    def find_apple_position(self, level_matrix):
        for r in range(len(level_matrix)):
            for c in range(len(level_matrix[0])):
                if (level_matrix[r][c] == "A"):
                    return (r, c)
        
        return (-1, -1)
        
    
    #  calculates manhattan distance between player and apple
    #this function assumes there is only a single apple in the level
    def heuristic(self, player_row, player_column, apple_row, apple_column):
        return abs(player_row - apple_row) + abs(player_column - apple_column)
    
        
    def get_neighbors(self, current_node, parent_node):
        loc_x = current_node.player_row - parent_node.player_row
        loc_y = current_node.player_col - parent_node.player_col
        return loc_x,loc_y

    # Check the neighbor for adding to unvisited list
    def is_valid_for_unvisited(self, unvisited, neighbor):
        for node in unvisited:
            if ((neighbor.player_row,neighbor.player_col) == (node.player_row,node.player_col) 
                    and neighbor.f >= node.f):
                return False
        return True

    def solve(self, level_matrix, player_row, player_column):
        super().solve(level_matrix, player_row, player_column)
        move_sequence = []

       

        initial_level_matrix = [list(row) for row in level_matrix] #deepcopy(level_matrix)
        
        
        level_height = len(initial_level_matrix)
        level_width = len(initial_level_matrix[0])
        
       
        #  initialize g values
        self.g_values = [ [self.INFINITY_COST]*level_width for i in range(level_height) ]
        
        #  initialize g of starting position 0
        self.g_values[player_row][player_column] = 0
        
        #  calculate heuristic value for starting position
        (apple_row, apple_column) = self.find_apple_position(initial_level_matrix)
        initial_heuristic = self.heuristic(player_row, player_column, apple_row, apple_column)
        
        print("A* solve() --- level size:", (level_height, level_width))
        print("A* solve() --- apple position:", (apple_row, apple_column))
        print("A* solve() --- initial_heuristic:", initial_heuristic)
        
        
        
        """
            YOUR CODE STARTS HERE
            fill move_sequence list with directions chars
        """
        
        # Initializing the nodes for start and goal
        starting_node = Node(None,initial_level_matrix,player_row,player_column,0,initial_heuristic,0)
        self.generated_node_count += 1
        unvisited = []
        visited = []
        unvisited.append(starting_node)
        
        while (unvisited):
            # Sort the unvisited list according to f value
            # If the f values are the same, it will sort according to depth and h value
            unvisited.sort(key=self.find_f_value) 

            #Saves the maximum node
            self.maximum_node_in_memory_count = max(self.maximum_node_in_memory_count, len(unvisited))
            
            current_node = unvisited.pop(0) #Get first node as current node
            x,y = current_node.player_row, current_node.player_col

            # Checks if the current node is on the apple or not
            if x == apple_row and y == apple_column:
                # Traverse the node list till to the starting node
                while ((current_node.player_row,current_node.player_col) != (starting_node.player_row,starting_node.player_col)):    
                    # Find the direction of the neighbor of the current node
                    loc_x, loc_y = self.get_neighbors(current_node,current_node.parent_node)
                    if (loc_x,loc_y) == (0,-1):
                        move_sequence.append('L')
                    elif (loc_x,loc_y) == (0,1):
                        move_sequence.append('R')
                    elif (loc_x,loc_y) == (-1,0):
                        move_sequence.append('U')
                    elif (loc_x,loc_y) == (1,0):
                        move_sequence.append('D')

                    current_node = current_node.parent_node
                move_sequence.reverse()
                # Sums the expanded node count
                self.expanded_node_count += len(visited)
                return move_sequence

            # Set neighbors direction
            directions = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
            # Shuffles the direction in order to make the system more real
            # Thus, it will not follow the same path every time.
            random.shuffle(directions)
            
            # Checks the neighbors
            for next in directions:
                # Check the boundaries of the matrix
                if(next[0] + 1 > len(initial_level_matrix)):
                    continue
                if(next[1] + 1 > len(initial_level_matrix[1])):
                    continue

                # Check the next direction whether it is F or not
                obstacle = initial_level_matrix[next[0]][next[1]]
                if(obstacle == 'W' or obstacle == 'P'):
                    continue

                # Check the direction if it is visited or not
                if((next[0],next[1]) in visited):
                    continue
                
                # Creating a node for neighbor
                h_neighbor = self.heuristic(next[0],next[1],apple_row,apple_column)
                g_neighbor = current_node.depth + 1
                f_neighbor = g_neighbor + h_neighbor
                neighbor = Node(current_node,initial_level_matrix,next[0],next[1],g_neighbor,h_neighbor,f_neighbor)

                # Increase the generated node count
                self.generated_node_count += 1

                 # Check if neighbor is in unvisited list and if it has a lower f value
                if(self.is_valid_for_unvisited(unvisited, neighbor)):
                    unvisited.append(neighbor)
                

            visited.append((x,y))
            
        """
            YOUR CODE ENDS HERE
            return move_sequence
        """
        return move_sequence
