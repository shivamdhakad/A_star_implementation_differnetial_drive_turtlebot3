

export TURTLEBOT3_MODEL=waffle

# ENPM661_Project3_Phase2

Team:
Shivam Dhakad - 120263477
Modabbir Adeeb- 120314827

Github link:
https://github.com/shivamdhakad/A_star_implementation_differnetial_drive_turtlebot3

Video File, drive link: (contain 2 simulation vedio files)
https://drive.google.com/drive/folders/1zavCcopyKfl8CDx0SJVWz5sZJlF5TpN5?usp=sharing

Part 01:-

Libraries Used:-
numpy
pynput
math
matplotlib.pyplot
time
heapq
matplotlib.patches
rclpy.qos
geometry_msgs.msg
random


Instructions to run the code:
Part 1:
executable file: proj3_phase2_part1.py
run command: python3 proj3_phase2_part1.py

Part1 Test case1:
Inputs to the terminal:
Enter clearance of robot : 10
Enter the Low RPM: 5
Enter the High RPM: 9
Enter starting x , y ,orientation seperated by spaces in mm : 500 1000 0
Enter goal x coordinates and y coordinates in mm: 5500 1000 

#####################################################################################
Part 2:

main executable files:
competition_world.launch.py
turtlebot_path_follower.py
ros pacakage: turtlebot3_project3

commands:
a)ros2 launch turtlebot3_project3 competition_world.launch.py
b)ros2 run turtlebot3_project3 turtlebot_path_follower.py

Instruction to run the code:
1)place the ros package in src folder
2)colcon build 
3)source install/setup.bash 
4)ros2 launch turtlebot3_project3 competition_world.launch.py
5)ros2 run turtlebot3_project3 turtlebot_path_follower.py   (run this command in seprate terminal after sourcing install/setup.bash and ros ) 
6) If ros2 run command does not execute and you get error: no executable found then go into script folder and there run this command to lacunh the python file:python3 turtlebot_path_follower.py

 
Part2 Test Case 1 :-
Enter clearance of robot in mm: 10
Enter the Low RPM: 5
Enter the High RPM: 9
Enter Goal node x coordinte, y coordinate seperated by spaces in mm 3000 1700



      

    
