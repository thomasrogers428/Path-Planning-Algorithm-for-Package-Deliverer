#!/usr/bin/env python

# Brandon Feng, Seungjae Lee, Thomas Rogers 
# Node class to be used in Search class 

class Node: 
    
    def __init__(self, state, cost, parent): 
        
        """
        state is 4-item tuple that includes the following: 
        robot location in the form of (x,y) tuple 
        set of packages held by robot (in tuple form to allow for hasing)
        set of delivered packages (in tuple form)
        action robot is to perform 
        """
        
        self.state = state 
        self.cost = cost 
        # also stores parent node for backchaining
        self.parent = parent
        
    # getter methods for various items in state 
        
    def get_robot_location(self): 
        return self.state[0]
        
    def get_robot_packages(self): 
        return set(self.state[1])
        
    def get_delivered_packages(self): 
        return set(self.state[2])
        
    def get_action(self): 
        return self.state[3]
        
    def get_cost(self): 
        return cost 
        
    # comparator for use in priority queue 
    def __lt__(self, other):
        return self.cost < other.cost