# Path-Planning-Algorithm-for-Package-Deliverer
####Seungjae (Jason) Lee, Thomas Rogers, Brandon Feng
The package has algorithm that finds the optimal path for user-assigned set of packages and navigates while avoiding obstacle
Run the following on each terminal:

1. **roscore**
2. **rosrun stage_ros stageros \<location of deliverer package\>/world/Testmap.world**
3. First, download the attached package in src and run **catkin_make** in the workspace. Do **source catkin_ws/devel/setup.bash**. Then run, **rosrun deliverer movement.py**


Additional Notes: Packages (input for this project) can be changed in change_path() function in movement.py
