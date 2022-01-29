import time
import random
from copy import deepcopy
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
    
    def __init__(self, parent_node, level_matrix, player_row, player_column, rhs, h, key):
        self.parent_node = parent_node
        self.level_matrix = level_matrix
        self.player_row = player_row
        self.player_col = player_column
        self.h = h
        self.rhs = rhs
        self.key = key

        
    
    def __lt__(self, other):
        return self.key < other.key
        

       
class PriorityQueue: 
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]


class DStarLiteAgent(Agent):

    def __init__(self):
        super().__init__()
        
        
        self.initialized = False
        
        
        #  g cost in A*, 2d array of size [height][width] 
        #    IMPORTANT NOTE!!!
        #please fill values inside this array
        #as you perform the A* search!
        self.g_values = []
        self.s_last = Node
        self.s_goal = Node
        self.s_start = Node
        #  rhs cost in D*, 2d array of size [height][width]
        #SAME AS G, FILL THESE VALUES IN YOUR CODE
        self.rhs_values = []
        self.unvisited = []
        self.k_m = 0
        #  a large enough value for initializing g values at the start
        self.INFINITY_COST = 2**10
        
    def initialize(self, initial_level_matrix, player_row, player_column, apple_row, apple_column):
        level_height = len(initial_level_matrix)
        level_width = len(initial_level_matrix[0])
        #  initialize rhs values
        self.rhs_values = [ [self.INFINITY_COST]*level_width for i in range(level_height) ]
        #  initialize g values
        self.g_values = [ [self.INFINITY_COST]*level_width for i in range(level_height) ]
        initial_heuristic = self.heuristic(player_row, player_column, apple_row, apple_column)
        self.rhs_values[apple_row][apple_column] = 0
        self.k_m = 0
        self.s_start = Node(None,initial_level_matrix,player_row,player_column,self.rhs_values[player_row][player_column],initial_heuristic,(0,0))
        self.s_goal = Node(None,initial_level_matrix,apple_row,apple_column,self.rhs_values[apple_row][apple_column],
                        initial_heuristic, 0)
        self.s_goal.key = self.calculateKey(self.s_goal,self.s_start)

        self.unvisited.append((self.s_goal,self.calculateKey(self.s_goal,self.s_start)))
        return

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
   
    # Calculates the key value
    def calculateKey(self, current, s_start):
        return (min(self.g_values[current.player_row][current.player_col], self.rhs_values[current.player_row][current.player_col]) + 
                self.heuristic(s_start.player_row,s_start.player_col,current.player_row,current.player_col) + self.k_m, 
                min(self.g_values[current.player_row][current.player_col], self.rhs_values[current.player_row][current.player_col]))
    
    # updates the nodes
    def updateVertex(self, u, s_start):
        if(self.level_matrix[u.player_row][u.player_col] == 'W' or self.level_matrix[u.player_row][u.player_col] == 'A'):
            return
        if(u != self.s_goal):
            directions = self.get_directions((u.player_row,u.player_col))
            for next in directions:
                self.rhs_values[u.player_row][u.player_col] = min(self.rhs_values[u.player_row][u.player_col], 
                self.g_values[next[0]][next[1]]+1)

        for q in range(len(self.unvisited)):
            if((u.player_row,u.player_col) == (self.unvisited[q][0].player_row,self.unvisited[q][0].player_col)):
                self.unvisited.remove(self.unvisited[q])
                break

        if(self.g_values[u.player_row][u.player_col] != self.rhs_values[u.player_row][u.player_col]):
            self.unvisited.append((u,self.calculateKey(u,s_start)))
        return

    # gets neighbor location of given position as list
    def get_directions(self, current):
        x = current[0]
        y = current[1]
        return [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

    def computeShortestPath(self, s_start, matrix):
        while(self.topKey(self.unvisited) < self.calculateKey(s_start,s_start) or self.rhs_values[s_start.player_row][s_start.player_col] != self.g_values[s_start.player_row][s_start.player_col]):
            k_old = self.topKey(self.unvisited)
            u = self.unvisited.pop(0)
            x,y = u[0].player_row, u[0].player_col
            directions = self.get_directions((x,y))

            if(k_old < self.calculateKey(u[0],s_start)):
                self.unvisited.append((u[0],self.calculateKey(u[0],s_start)))
            elif(self.g_values[x][y] > self.rhs_values[x][y]):
                self.g_values[x][y] = self.rhs_values[x][y]
                for r,c in directions:
                    # Check the next direction whether it is F or not
                    obstacle = matrix[r][c]
                    if(obstacle == 'W' or obstacle == 'A'):
                        continue

                     # Creating a node for neighbor
                    h_neighbor = self.heuristic(r,c,s_start.player_row,s_start.player_col)
                    key = self.calculateKey(u[0],s_start)
                    neighbor = Node(u,matrix,r,c,self.rhs_values[r][c],h_neighbor,key)
                    
                    self.updateVertex(neighbor,s_start)
            else:
                self.g_values[x][y] = self.INFINITY_COST
                for r,c in directions:
                    # Check the next direction whether it is F or not
                    obstacle = matrix[r][c]
                    if(obstacle == 'W' or obstacle == 'A'):
                        continue

                     # Creating a node for neighbor
                    h_neighbor = self.heuristic(r,c,s_start.player_row,s_start.player_col)
                    key = self.calculateKey(u[0],s_start)
                    neighbor = Node(u,matrix,r,c,self.rhs_values[r][c],h_neighbor,key)
                    
                    self.updateVertex(neighbor,s_start)

        return    
        
    def key_sort(self, u):
        return u[1]
    
    def topKey(self, V):
        V.sort(key=self.key_sort)
        return V[0][1]
    #sorts the list and picks the first one
    # def topKey(self, V):
    #     V = sorted(V)
    #     return V[0][1]

    def get_neighbors(self, a, b, c, d):
        loc_x = c - a
        loc_y = d - b
        return loc_x,loc_y

    def path_finding(self, a, b, apple_row, apple_column, move_sequence):
        while((a,b) != (apple_row,apple_column)):
                directions = self.get_directions((a,b))
                for next in directions:
                    # Find the direction of the neighbor of the current node
                    loc_x, loc_y = self.get_neighbors(a,b,next[0],next[1])
                    if(self.rhs_values[next[0]][next[1]] < self.rhs_values[a][b]):
                        if (loc_x,loc_y) == (0,-1):
                            move_sequence.append('L')
                        elif (loc_x,loc_y) == (0,1):
                            move_sequence.append('R')
                        elif (loc_x,loc_y) == (-1,0):
                            move_sequence.append('U')
                        elif (loc_x,loc_y) == (1,0):
                            move_sequence.append('D')
                        if(a + loc_x < len(self.rhs_values)):
                            a += loc_x
                        if(b + loc_y < len(self.rhs_values[1])):
                            b += loc_y 
                        break
        return move_sequence

    def solve(self, level_matrix, player_row, player_column, changed_row, changed_column):
        super().solve(level_matrix, player_row, player_column)
        move_sequence = []

        """
            YOUR CODE STARTS HERE
            fill move_sequence list with directions chars
        """

        initial_level_matrix = [list(row) for row in level_matrix] #deepcopy(level_matrix)
        self.print_level_matrix(initial_level_matrix)
        
        #  calculate heuristic value for starting position
        (apple_row, apple_column) = self.find_apple_position(initial_level_matrix)

       

 
        if (not self.initialized):
            #  first time calling D*lite agent solve()
            self.initialize(initial_level_matrix,player_row,player_column,apple_row,apple_column)
            self.s_last = self.s_start
            self.computeShortestPath(self.s_start, initial_level_matrix)
            print()
            self.path_finding(player_row,player_column,apple_row,apple_column,move_sequence)
            
            self.initialized = True
        else:
            #  initialization phase is already performed
            #  this means solve() is called once again because there is
            #a change detected in the map
            self.k_m += self.heuristic(player_row,player_column,self.s_last.player_row,self.s_last.player_col)
            self.s_start.player_row = player_row
            self.s_start.player_col = player_column
            self.rhs_values[changed_row][changed_column] = self.INFINITY_COST
            # self.rhs_values[player_row][player_column] = self.INFINITY_COST
            self.g_values[changed_row][changed_column] = self.INFINITY_COST
            self.g_values[player_row][player_column] = self.INFINITY_COST

            print("Solve called again because a new obstacle appeared at position:(", changed_row, ",", changed_column, ")")
            h = self.heuristic(player_row,player_column,apple_row,apple_column)
            u = Node(None,initial_level_matrix,player_row,player_column,self.rhs_values[player_row][player_column],
                h, self.calculateKey(self.s_goal,self.s_start))
            self.updateVertex(u,self.s_start)
            self.computeShortestPath(self.s_start, initial_level_matrix)
            self.path_finding(player_row,player_column,apple_row,apple_column,move_sequence)
            
        
        
        """
            YOUR CODE ENDS HERE
            return move_sequence
        """
        #move_sequence = ["U"]
        return move_sequence
    
    
    
    def on_encounter_obstacle(self):
        pass
