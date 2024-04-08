# A_star_implementation_differnetial_drive_turtlebot3

## ENPM661_Project3_Phase2

Team:
Shivam Dhakad - 120263477
Modabbir Adeeb- 120314827

- Github link:
  https://github.com/shivamdhakad/A_star_implementation_differnetial_drive_turtlebot3

- Video File, drive link: (contain 2 simulation vedio files)
  https://drive.google.com/drive/folders/1zavCcopyKfl8CDx0SJVWz5sZJlF5TpN5?usp=sharing

### Part 01:-
Libraries Used:-\
numpy\
math\
matplotlib.pyplot\
time\
heapq\
matplotlib.patches\
rclpy.qos\
geometry_msgs.msg\


- Instructions to run the code:

-Part 1:
executable file run command: python3 part1.py\

test case1:
Inputs to the terminal:
Enter clearance of robot in mm: 10
Enter the Low RPM: 5
Enter the High RPM: 9
Enter starting x coordintes, y coordinates and orientation seperated by spaces in mm : 500 1000 0
Enter goal x coordinates and y coordinates in mm: 5000 1000 0

- Part 2:
instruction to run the code:

ros pacakage: turtlebot3_project3
commands:
place the ros package in src folder
colcon build 
source install/setup.bash 
ros2 launch turtlebot3_project3 competition_world.launch

after these steps, in terminal go into the ros package folder 
and run: python3 turtlebot_path_follower.py    (run this command in terminal where the file is saved)


The terminal will ask for user input and enter values as shown below  \
Test Case 1 :- \
Enter clearance of robot in mm: 50\
Enter the Low RPM: 5\
Enter the High RPM: 9\
Enter Goal node x coordinte, y coordinate seperated by spaces in mm 3000 1700



      

    
