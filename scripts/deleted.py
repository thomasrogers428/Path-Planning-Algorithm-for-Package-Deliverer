 # IGNORE: deleted files 
 
 # updates robot packages and weight accordingly 
    def add_package(self, package): 
        
        # adds package to robot and adds to robot weight 
        self.robot_packages.add(package)
        self.robot_weight += self.package2weight[package]
        
    def remove_package(self, package): 
        
        # removes package from robot and decreases robot weight 
        self.robot_packages.remove(package)
        self.robot_weight -= self.package2weight[package]

    # picks up packages from warehouse and updates information accordingly 
    def pickup(self, packages): 
        
        # store pertinent information to undo later if needed 
        warehouse = self.robot_location 
        picked_packages = set() 
        
        # add packages to robot, remove from warehouse 
        for package in packages: 
            self.add_package(package)
            self.warehouse2packages[warehouse].remove(package) 
            picked_packages.add(package)
            
        return warehouse, picked_packages 

    # moves robot to specified location and updates information accordingly 
    def travel(self, location): 
        
        # store pertinent information to undo later if needed 
        prev_location = self.robot_location 
        dropped_packages = set() 
        
        self.robot_location = location 
        
        if self.robot_location in self.recipient2packages: 
            # assumes that robot drops off all packages that can be dropped off if travelling to recipient 
            packages_to_drop = self.recipient2packages[location] 
            for package in packages_to_drop: 
                # drops off package if robot is carrying
                if package in self.robot_packages: 
                    self.remove_package(package)
                    dropped_packages.add(package)
                
        return prev_location, dropped_packages
        
    # undoes travel to new location 
    def undo_travel(self, old_location, dropped_packages): 
        
        # change location 
        self.robot_location = old_location 
        # robot "picks up" packages from where it dropped them off 
        for package in dropped_packages: 
            self.add_package(package)
                
    # undoes package pickup from warehouse 
    def undo_pickup(self, warehouse, packages):
        
        # add packages back to warehouse and remove from robot 
        for package in packages: 
            self.warehouse2packages[warehouse].add(package)
            self.remove_package(package)
        
    # pushes specified action 
    def push_action(self, action, parameter): 
        
        if action == PICKUP: 
            location, packages = self.pickup(parameter)
        else: 
            location, packages = self.travel(parameter)
            
        return location, packages
             
    # pops (undoes) specified action 
    def pop_action(self, action, location, packages): 
        
        if action == PICKUP: 
            self.undo_pickup(location, packages)
        else: 
            self.undo_travel(location, packages)