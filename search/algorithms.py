import heapq
import math

class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSED list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the nodes
    in the self.open list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node. 
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._cost = 0
        self._parent = None
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the self.open list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def set_parent(self, parent):
        """
        Sets the parent of a node in the search tree
        """
        self._parent = parent

    def get_parent(self):
        """
        Returns the parent of a node in the search tree
        """
        return self._parent
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def set_g(self, g):
        """
        Sets the g-value of the state
        """
        self._g = g

    def get_cost(self):
        """
        Returns the cost of a state; the cost is determined by the search algorithm
        """
        return self._cost
    
    def set_cost(self, cost):
        """
        Sets the cost of the state; the cost is determined by the search algorithm 
        """
        self._cost = cost

class Dijkstra:
    def __init__(self, map):
        """
        the constructor
        """
        self.map = map
        self.closed = {}
        self.open = []
        self.count = 0

    def get_path(self, goal_state):
        """
        get the path from the goal state to the start state by backtracking its parent pointers
        """
        path = []
        cur = goal_state
        while cur is not None:
            path.append(cur)
            cur = cur.get_parent()
        path.reverse()
        return path
    
    def get_closed(self): 
        """
        get the closed data
        """
        return self.closed
    
    def get_open(self):
        """
        get the open_data
        """
        return self.open
    
    def get_count(self):
        """
        get the number of states expanded
        """
        return self.count


    def run(self, start, goal): #run the dijkstra algorithm
        """
        run the dijkstra algorithm
        """
        self.open = []
        self.closed = {}
        self.count = 0

        if start == goal:
            return ([start], 0.0, 0)    #if start is the goal state
        
        
        best_g = {} #instead of expanding, keep track of all instances

        #start the start state
        start.set_g(0.0)
        start.set_cost(start.get_g())  # dijkstra uses g-value
        heapq.heappush(self.open, start)
        best_g[start.state_hash()] = 0.0

        while self.open:
            cur = heapq.heappop(self.open)  #pop with the least cost

            
            if cur.state_hash() in self.closed:   #check if in closed already
                continue

            self.closed[cur.state_hash()] = cur #add it into closed list 
            self.count += 1 #number of states expanded

            
            if cur == goal:   #if the current state is the goal state
                return self.get_path(cur), cur.get_g(), self.get_count()

            
            successors = self.map.successors(cur) #expand the current state
            for s in successors:
                s.set_parent(cur) #set the parent od the successors to the current state

                s.set_cost(s.get_g())   #update the gvalues of them

                s_hash = s.state_hash() #make them unique

                #if not in closed, and either not seen yet or found better gvalue
                if s_hash not in self.closed and (s_hash not in best_g or s.get_g() < best_g[s_hash]):
                    best_g[s_hash] = s.get_g()  #update the best gvalue
                    heapq.heappush(self.open, s)    #push the state to open list

        #if no path was found
        return None, -1, self.get_count()

class AStar:
    def __init__(self, map_object):
        """
        the constructor
        """
        self.map = map_object
        self.closed = {}
        self.open = []
        self.count = 0

    def octile_distance(self, s, goal):
        """
        Octile distance:
        h(s) = 10 * min(Δx, Δy) + |Δx - Δy|

        Where Δx = |s.x - goal.x|
        Δy = |s.y - goal.y|.
        """
        dx = abs(s.get_x() - goal.get_x())
        dy = abs(s.get_y() - goal.get_y())
        
        return 1.5 * min(dx, dy) + abs(dx - dy)
    
    def get_path(self, goal_state):
        """
        get the path from the goat state to the start state by backtracking its parent pointers
        """
        path = []
        cur = goal_state
        while cur is not None:
            path.append(cur)
            cur = cur.get_parent()
        path.reverse()
        return path

    def get_closed(self):
        """
        get the closed_list
        """
        return self.closed
    
    def get_open(self):
        """
        get the open_list
        """
        return self.open
    def get_count(self):
        """
        get the number of states expanded
        """
        return self.count

    def run(self, start, goal):
        self.open = []
        self.closed = {}
        self.count = 0

        if start == goal:   #if start is the goal state
            return ([start], 0.0, 0)

        best_g = {} #instead of expanding, keep track of all instances

        #start the start state
        start.set_g(0.0)
        h_start = self.octile_distance(start, goal)
        start.set_cost(start.get_g() + h_start)
        heapq.heappush(self.open, start)
        best_g[start.state_hash()] = 0.0

        while self.open:
            cur = heapq.heappop(self.open)

            if cur.state_hash() in self.closed:   #check if in closed already
                continue

            self.closed[cur.state_hash()] = cur #add it into closed list
            self.count += 1

            if cur == goal:   #if the current state is the goal state
                return self.get_path(cur), cur.get_g(), self.get_count()
            
            successors = self.map.successors(cur) #get the child states

            for s in successors:
                s.set_parent(cur) #set the parent of the child states to the current state
            
                h_val = self.octile_distance(s, goal)   #calculate the hvalue of the child states

                s.set_cost(s.get_g() + h_val)   #update the cost of the child states (f = g + h)

                s_hash = s.state_hash() #make the state unique
                
                if s_hash not in self.closed and (s_hash not in best_g or s.get_g() < best_g[s_hash]):  #if not in closed, and either not seen yet or found better gvalue
                    best_g[s_hash] = s.get_g()  #update the best gvalue
                    heapq.heappush(self.open, s)   

        #if no path was found
        return None, -1, self.get_count()