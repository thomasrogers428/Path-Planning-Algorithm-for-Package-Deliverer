#!/usr/bin/env python

# Brandon Feng, Seungjae Lee, Thomas Rogers 
# Search node: given warehouse and recipient locations and the robot initial location, finds optimal sequence of robot actions 

from heapq import heappush, heappop # for priority queue 
import math 
from itertools import chain, combinations # used for find powersets (sets of packages)
from node import Node 
from copy import copy 

# constants for actions
TRAVEL = 0  # travel to new location 
PICKUP = 1 # pickup set of packages 

class Search: 
    
    def __init__(self, robot_capacity, init_robot_location, package_info): 
        
        # robot's package weight limit 
        self.robot_capacity = robot_capacity 
        # initial robot location: (x,y) tuple 
        self.init_robot_location = init_robot_location 
        # dictionary: key = package, value = (warehouse location, destination, weight)
        self.package_info = package_info
        
        # creating set of warehouse locations and set of recipient locations 
        self.warehouses = set() 
        self.recipients = set() 
        for package in self.package_info: 
            self.warehouses.add(self.package_info[package][0])
            self.recipients.add(self.package_info[package][1])
            
    # boolean for if goal state is reached 
    def goal_reached(self, node): 
        
        # if number of delivered packages equals total number of packages 
        if len(node.get_delivered_packages()) == len(self.package_info.keys()): 
            return True 
        return False 
        
    # euclidean distance between location1 and location2
    def get_distance(self, location1, location2):   
        
        x1 = location1[0]
        y1 = location1[1]
        x2 = location2[0]
        y2 = location2[1]
        
        distance = math.sqrt(((y2 - y1)**2) + ((x2 - x1)**2))
        return distance 
    
    # returns total weight of set of packages 
    def find_weight(self, packages): 
        
        weight = 0
        # sum up to find total weight of packages 
        for package in packages: 
            weight += self.package_info[package][2]
            
        return weight 
        
    # drops off all recipient packages held by robot 
    def deliver_packages(self, node, recipient): 
        
        # to return - updated information about delivered packages and packages held by robot 
        new_robot_packages = node.get_robot_packages()
        new_delivered_packages = node.get_delivered_packages()
        
        for package in node.get_robot_packages(): 
            # if package is meant for recipient
            if self.package_info[package][1] == recipient: 
                # remove from robot and mark as delivered 
                new_robot_packages.remove(package)
                new_delivered_packages.add(package)
                
        return new_robot_packages, new_delivered_packages 
        
    # returns set of packages stored at warehouse 
    def get_warehouse_packages(self, warehouse): 
        
        packages = set() 
        # loop through packages and add ones that belong to warehouse 
        for package in self.package_info: 
            if self.package_info[package][0] == warehouse: 
                packages.add(package)
        
        return packages 
                
    # finds all subsets of packages within packages 
    def find_all_subsets(self, packages):

        # makes use of itertools library 
        s = list(packages)
        # return as iterable 
        subsets = chain.from_iterable(combinations(s, r) for r in range(len(s)+1))
        return subsets 
        
    def get_neighbors(self, parent_node): 
        
        # list of (x,y) tuples where x = neighbor state and y = cost 
        neighbors = []
        
        # if robot is to travel 
        if parent_node.get_action() == TRAVEL: 
            
            # loop through warehouse locations
            for warehouse in self.warehouses: 
                # so robot doesn't stay in place 
                if warehouse == parent_node.get_robot_location(): 
                    continue 
                # get cost of travel - cummulative 
                neighbor_cost = parent_node.cost + self.get_distance(parent_node.get_robot_location(), warehouse) 
                # since robot is going to warehouse, it should pickup packages next 
                neighbor_action = PICKUP
                # create state - delivered packages and packages carried by robot remain the same
                neighbor_state = (warehouse, tuple(parent_node.get_robot_packages()), tuple(parent_node.get_delivered_packages()), neighbor_action)
                neighbors.append((neighbor_state, neighbor_cost))
                
            # loop through recipient locations 
            for recipient in self.recipients: 
                if recipient == parent_node.get_robot_location(): 
                    continue 
                neighbor_cost = parent_node.cost + self.get_distance(parent_node.get_robot_location(), recipient) 
                # after arriving to recipient, it should travel again 
                neighbor_action = TRAVEL 
                # assuming that robot drops off packages when travelling to recipient 
                neighbor_robot_packages, neighbor_delivered_packages = self.deliver_packages(parent_node, recipient)
                # create state 
                neighbor_state = (recipient, tuple(neighbor_robot_packages), tuple(neighbor_delivered_packages), neighbor_action)
                neighbors.append((neighbor_state, neighbor_cost))

        # if robot is to pick up set of packages 
        else: 
            # robot should travel after picking up packages 
            neighbor_action = TRAVEL 
            # get all packages at current warehouse
            all_packages = self.get_warehouse_packages(parent_node.get_robot_location())
            # find all possible sets of packages 
            for set_of_packages in self.find_all_subsets(all_packages): 
                # ignore empty set 
                if len(set_of_packages) == 0: 
                    continue 
                neighbor_robot_packages = parent_node.get_robot_packages()
                # add packages to robot 
                for package in set_of_packages: 
                    neighbor_robot_packages.add(package) 
                # find new weight 
                neighbor_weight = self.find_weight(neighbor_robot_packages)
                # ignore if exceeds robot capacity 
                if neighbor_weight > self.robot_capacity: 
                    continue 
                # create state and add to list: does not incur additional cost
                neighbor_state = (parent_node.get_robot_location(), tuple(neighbor_robot_packages), tuple(parent_node.get_delivered_packages()), neighbor_action)
                neighbors.append((neighbor_state, parent_node.cost))
                
        return neighbors 
        
    # backchaining to find path 
    def backchain(self, node): 
        
        path = []
        while node is not None: 
            # inserts at beginning of path for correct sequential order
            path.insert(0, node)
            node = node.parent 
            
        return path 
        
    # Dijkstra's algorithm implementation: returns path and total cost 
    def search(self): 
        
        # priority queue 
        q = []
        # dictionary: key = visited state, value = lowest cost 
        visited2cost = {} 
        
        # finding first action to take 
        # if robot is at warehouse, it should pick up packages 
        if self.init_robot_location in self.warehouses: 
            init_action = PICKUP 
        # otherwise, it should travel 
        else: 
            init_action = TRAVEL 
            
        # initializing queue and visited dictionary 
        state = (self.init_robot_location, (), (), init_action)
        #print(state)
        node = Node(state, 0, None)
        heappush(q, node)
        visited2cost[state] = 0

        solution_found = False 
        path = []
        
        # loop until q is empty 
        while len(q) != 0: 
            
            # pop lowest cost node from queue 
            node = heappop(q) 
            # check if node has been marked as "removed" (i.e has inefficent cost)
            if node.state in visited2cost: 
                # if so, ignore 
                if node.cost > visited2cost[node.state]: 
                    continue 

            # if goal state is reached, break loop 
            if self.goal_reached(node): 
                solution_found = True 
                break 
            
            # loop through neighbors and add to queue
            for neighbor_state, neighbor_cost in self.get_neighbors(node): 
                # if neighboring state has not been visited or have found lower cost path
                if neighbor_state not in visited2cost or neighbor_cost < visited2cost[neighbor_state]: 
                    neighbor_node = Node(neighbor_state, neighbor_cost, node)
                    # add to queue 
                    heappush(q, neighbor_node)
                    # update lowest possible cost 
                    visited2cost[neighbor_state] = neighbor_cost 
                    
        # if solution is found, backchain to find path 
        if solution_found: 
            path = self.backchain(node)
            
        return path, node.cost 
                
        



    
