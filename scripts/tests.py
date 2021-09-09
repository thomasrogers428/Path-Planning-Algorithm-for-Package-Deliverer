# Brandon Feng, Seungjae Lee, Thomas Rogers 
# Testing file 

from search import Search 
from node import Node 

### Testing Search Node ###

robot_capacity = 5
init_robot_location = (0,0)
package_info = {}
package_info[1] = ((2,1), (4,4), 1)
package_info[2] = ((2,1), (10,0), 2)
package_info[3] = ((2,1), (10,0), 3)
package_info[4] = ((6,5), (15,10), 4)
package_info[5] = ((6,5), (4,4), 5)

search = Search(robot_capacity, init_robot_location, package_info)

print("\n*** testing get_neighbors() function ***\n")
test_parent_state = ((1,1), set(), set(), 0)
parent_node = Node(test_parent_state, 0, None)
test_state = ((2,1), set(), set(), 1)
test_node = Node(test_state, 1, parent_node)
neighbors = search.get_neighbors(test_node)
print("first step...")
for neighbor_state, neighbor_cost in neighbors: 
    print("state: ")
    print(neighbor_state)
    print("cost: ")
    print(neighbor_cost)
test_node = Node(neighbors[0][0], neighbors[0][1], test_node)
neighbors = search.get_neighbors(test_node)
print("second step...")
for neighbor_state, neighbor_cost in neighbors: 
    print("state: ")
    print(neighbor_state)
    print("cost: ")
    print(neighbor_cost)

print("\n*** Testing search() function ***\n")
search = Search(robot_capacity, init_robot_location, package_info)
path, cost = search.search() 
print("Path: ")
for node in path: 
    print(node.state)
print("Cost: ")
print(cost)










    
