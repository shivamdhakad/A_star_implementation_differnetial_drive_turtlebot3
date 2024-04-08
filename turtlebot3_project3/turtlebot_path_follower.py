#!/usr/bin/env python3
# import rospy
from geometry_msgs.msg import Twist
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import heapq
from math import dist
import matplotlib.patches as patches

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node as ROSNode
from geometry_msgs.msg import Twist
from rclpy.time import Time
from rclpy.clock import Clock
from rclpy.qos import QoSProfile
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import heapq
from math import dist
import matplotlib.patches as patches


#Turning on interactive mode on Matplotlib
plt.ion()


#Class node is defined to constitute a node in the graph
class Node:
    
    #Node class attributes are initialized
    def __init__(self, x, y, parent, current_theta, change_theta, UL, UR, c2c, c2g, total_cost):
        self.x = x
        self.y = y
        self.parent = parent
        self.current_theta = current_theta
        self.change_theta = change_theta
        self.UL = UL
        self.UR = UR
        self.c2c = c2c
        self.c2g = c2g
        self.total_cost = total_cost
    
    #Less-than operator for Node class objects is defined based on their total cost    
    def __lt__(self, other):

        return self.total_cost < other.total_cost

#Defined a Function to plot dubins curves
def plot_curve(Xi, Yi, Thetai, UL, UR,c, plot, Nodes_list, Path_list):
    
    
    #Xi, Yi,Thetai: coordinates of input points
    #X_start, Y_start: coordinates of start point for plot function
    #X_end, Y_end, Theta_end: coordinates and orientation of the end point
    #UL, UR:  Control inputs
    #c: Clearance value of the obstacle space
    #plot: Flag to indicate if the curve should be plotted or not
    #Nodes_list: Nodes in the curve are stored in this list
    #Path_list: All the points in the path are stored in this list
    
    t = 0
    r = 0.033
    L = 0.160
    dt = 0.8
    cost = 0
    X_end = Xi
    Y_end= Yi
    Theta_end = 3.14 * Thetai / 180

    while t < 1:
        t = t + dt
        X_start = X_end
        Y_start = Y_end
        X_end += r*0.5 * (UL + UR) * math.cos(Theta_end) * dt
        Y_end += r*0.5 * (UL + UR) * math.sin(Theta_end) * dt
        Theta_end += (r / L) * (UR - UL) * dt

        #If loop to Check if the current move are valid or not
        if  move_validity(X_end,  Y_end, r, c):
            
            #If it is not plotting than update the path list and nodes, and add the cost
            if plot == 0:
                c2g = dist((X_start, Y_start), (X_end,  Y_end))
                cost = cost + c2g
                Nodes_list.append((X_end, Y_end))
                Path_list.append((X_start, Y_start))

            #If it is plotting, that draw the curve
            if plot == 1:
                plt.plot([X_start, X_end], [Y_start, Y_end], color="red")
        #Return None if the move is invalid
        else:
            return None
    #converting angle to radian
    Theta_end = 180 * (Theta_end) / 3.14
    return [X_end, Y_end, Theta_end, cost, Nodes_list, Path_list]


#An UID is generated for each node based on its position
def UID(node):
    UID = 1000*node.x + 111*node.y 
    return UID

#Defined the function to implement A star search algorithm for mobile robot motion planning
def a_star(start_node, goal_node, rpm1, rpm2, radius, clearance):

    
    #Check to see if the Goal node is reached 
    if check_goal(start_node, goal_node):
        return 1,None,None
    
    #Goal and start nodes are set
    start_node = start_node
    start_node_id = UID(start_node)
    goal_node = goal_node

    #To keep track of explored nodes list and dictionaries are initiated
    #Explored nodes are stored in Nodes_list list
    Nodes_list = []
    #Final path from start to goal node is stored in Path_list list
    Path_list = []

    #Open nodes are stored in open_node dictionary
    open_node = {}  # Dictionary to store all the open nodes
    #Closed nodes are stored in closed_node dictionary
    closed_node = {} 
    
    #Start node is added to the open nodes dictionary
    open_node[start_node_id] = start_node
    #store nodes based on their total cost in priority list
    priority_list = []
    
    #Every possible moves of the robot is defined
    moves = [[rpm1, 0], 
             [0, rpm1], 
             [rpm1, rpm1], 
             [0, rpm2], 
             [rpm2, 0], 
             [rpm2, rpm2], 
             [rpm1, rpm2],
             [rpm2, rpm1]]

    #Start node is pushed into the priority queue with total cost
    heapq.heappush(priority_list, [start_node.total_cost, start_node])

    #Run A* algorithm 
    while (len(priority_list) != 0):

        #node with the minimum cost is poped from the priority queue
        current_nodes = (heapq.heappop(priority_list))[1]
        current_id = UID(current_nodes)

        #Check weather poped node is the goal node
        if check_goal(current_nodes, goal_node):
            goal_node.parent = current_nodes.parent
            goal_node.total_cost = current_nodes.total_cost
            print("Goal Node found")
            return 1, Nodes_list, Path_list
        
        # Add the popped node to the closed nodes dictionary
        if current_id in closed_node:  
            continue
        else:
            closed_node[current_id] = current_nodes
        
        del open_node[current_id]
        
        #All the possible moves are looped through
        for move in moves:
            #After applying the move compute the next state
            action = plot_curve(current_nodes.x, current_nodes.y, current_nodes.current_theta, move[0], move[1],
                            clearance, 0, Nodes_list, Path_list)
           
            #Check to see if the move is valid
            if (action != None):

                #Coordinates and the angle are rounded off to the nearest integer
                angle = action[2]
                
                theta_lim = 15
                x = (round(action[0] * 10) / 10)
                y = (round(action[1] * 10) / 10)
                theta = (round(angle / theta_lim) * theta_lim)
                
                #The cost to move to the new node and orientation is calculated
                current_theta = current_nodes.change_theta - theta
                c2g = dist((x,y), (goal_node.x, goal_node.y))
                new_node = Node(x, y, current_nodes, theta, current_theta, move[0], move[1], current_nodes.c2c+action[3], c2g, current_nodes.c2c+action[3]+c2g)

                new_node_id = UID(new_node)
                
                #Check to see if the new node is not visited prior and is valid valid
                if not move_validity(new_node.x, new_node.y, radius, clearance):
                    continue
                elif new_node_id in closed_node:
                    continue

                #If the node info already exists in the open list update it
                if new_node_id in open_node:
                    if new_node.total_cost < open_node[new_node_id].total_cost:
                        open_node[new_node_id].total_cost = new_node.total_cost
                        open_node[new_node_id].parent = new_node

                #If the new node doesn't already exist add it to the open list        
                else:
                    open_node[new_node_id] = new_node
                    heapq.heappush(priority_list, [ open_node[new_node_id].total_cost, open_node[new_node_id]])
            
    return 0, Nodes_list, Path_list

#Defined a function to checks if the node is obstacle or not 
def WS(x, y, radius, clearance):
    
    #Total buffer space is defined
    total_space = radius + clearance

    #Check for the two rectangular obstacle
    obstacle2 = (x >= 1.5 - total_space) and (x <= 1.75 + total_space) and (y >= 1.00 - total_space)
    obstacle3 = (x >= 2.5 - total_space) and (x <= 2.75 + total_space) and (y <= 1.00 + total_space)

    # Check for the Circular obstacle 
    obstacle1 = ((np.square(x - 4.2)) + (np.square(y - 1.2)) <= np.square(0.6 + total_space))
    
   
    #Define the Borders   
    border1 = (x <= 0 + total_space)     
    border2 = (x >= 5.99 - total_space)
    border3 = (y <= 0 + total_space)
    border4 = (y >= 1.99 - total_space)

    #Check if there are any obstacle or border present
    if obstacle1 or obstacle2 or obstacle3 or border1 or border2 or border3 or border4:
        return True
    else:
        return False
    
#Define a fucntion checks if the move is valid based on absense of obstacle
def move_validity(x,y, r,c):
    
    #Check for obstacle
    if WS(x, y, r, c):
        return False
    else:
        return True

#Defined the fucntion to checks if the goal has been reached or not
def check_goal(current, goal):

    #Distance between the current and goal nodes is calculated
    dt = dist((current.x, current.y), (goal.x, goal.y))

    #Check if the goal has been reached
    if dt < 0.15:
        return True
    else:
        return False
    


# Defined the function to generate the path of the robot
def back_track(goal_node):  

    x_path = []
    y_path = []
    theta_path = []
    x_path.append(goal_node.x)
    y_path.append(goal_node.y)
    theta_path.append(goal_node.current_theta)
    parent_node = goal_node.parent
    ul_list=[goal_node.UL]
    ur_list=[goal_node.UR]

    #Backtrack to the start node
    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        theta_path.append(parent_node.current_theta)
        ul_list.append(parent_node.UL)
        ur_list.append(parent_node.UR)
        parent_node = parent_node.parent
        
    #Order of the path is reversed so it starts from the beginning    
    x_path.reverse()
    y_path.reverse()
    theta_path.reverse()
    ul_list.reverse()
    ur_list.reverse()

    x = np.asarray(x_path)
    y = np.asarray(y_path)
    theta = np.array(theta_path)
    return x, y, theta, ul_list, ur_list


#Using the A* algorithm this script finds a path between the start and goal positions of a robot.
#Robot's starting position, goal position, RPMs, robot radius, and clearance are taken from user input.
#Using matplotlib patches and circles the obstacle is defined.
#Start/goal positions and path are plotted on a graph.
#The runtime of the A*algorithm is printed.
if __name__ == '__main__':

    #Width, height, robot radius, clearance, and RPMs are defined.
    width = 6
    height = 2
    robot_radius  = 0.138
    # robot_radius  = 0.050
    # clearance = int(input("Enter clearance of robot: "))
    # clearance = clearance/100
    clearance = 15/100
    clearance = float(clearance)

    # RPM1 = int(input("Enter Low RPM: "))
    # RPM2 = int(input("Ente High RPM: "))
    RPM2 = 10
    RPM1 = 6
    #User input starting x-coord and y-coord of the robot.
    # start_x = int(input("Enter starting X co-ordinate: "))
    # start_y = int(input("Enter starting Y co-ordinate: "))
    # start_x = start_x/100

    # start_y = start_y/100
    start_x = 50/100
    start_y = 100/100

    start_x = float(start_x)
    start_y = float(start_y)

    #User input the starting orientation of the robot.
    # start_theta = input("Enter Orientation of the robot at start node: \n")
    # start_theta = int(start_theta)
    start_theta = int(0)

    #Starting orientation is rounded off to the nearest multiple of 30.
    number = int(start_theta)
    remainder = number % 30
    if remainder < 15:
      start_theta = number - remainder
    else:
      start_theta = number + (30 - remainder)
    
    #user input the goal x-coord and y-coord of the robot.
    goal_x = int(input("Enter goal X co-ordinates: "))
    goal_y = int(input("Enter goal Y co-ordinates: "))
    # goal_x = goal_x/100
    # goal_y = goal_y/100
    goal_x = goal_x/1000
    goal_y = goal_y/1000
    goal_x = float(goal_x)
    goal_y = float(goal_y)

    #if the start and/or goal positions are in the obstacle space it will error out
    if not move_validity(start_x, start_y, robot_radius, clearance):
        print("Start node is either invalid or in obstacle space")
        exit(-1)
        
    if not move_validity(goal_x, goal_y, robot_radius, clearance):
        print("Goal node is either invalid or in obstacle space")
        exit(-1)
    

    #Current time for runtime calculation
    timer_start = time.time()

    #Distance between the starting and goal positions is calculated
    c2g = dist((start_x,start_y), (goal_x, goal_y))
    total_cost =  c2g

    #Node object for the starting and goal positions is created
    start_node = Node(start_x, start_y,-1,start_theta,0,0,0,0,c2g,total_cost)
    goal_node = Node(goal_x, goal_y, -1,0,0,0,0,c2g,0,total_cost)

    #Run the A* algorithm to find a path between the starting and goal positions
    flag, Nodes_list, Path_list = a_star(start_node, goal_node,RPM1,RPM2,robot_radius,clearance)
                    
    #If flag is equal to 1, backtrack from goal to start node and get the path coordinates
    x_path, y_path, theta_path, ul_list, ur_list = back_track(goal_node)
    print("uL:",ul_list)
    print("ru_list:",ur_list)

# # Initialize the node
# rospy.init_node('turtlebot3_publisher', anonymous=True)

# # Create a publisher object
# pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# # Set the publishing rate
# rate = rospy.Rate(10) # 10 Hz

# # Set the start time
# start_time = rospy.Time.now()

# # # Prompt the user to input new velocity values and publish them one after the other at an interval of one second
# # while not rospy.is_shutdown():
# #     # Prompt the user to input new velocity values
# #     # print("Enter linear and angular velocity values (e.g., 0.1 0.5):")
# #     # vel_values = input().split()
# #     # linear_vel = float(vel_values[0])
# #     # angular_vel = float(vel_values[1])

# #     vel_msg = Twist()
# #     vel_msg.linear.x = random.uniform(0.0, 0.3) # generate a random linear velocity between 0.0 and 1.0
# #     vel_msg.angular.z = random.uniform(0.0, 0.3) # generate a random angular velocity between 0.0 and 1.0

# #     # # Define the velocity message
# #     # vel_msg = Twist()
# #     # vel_msg.linear.x = linear_vel
# #     # vel_msg.angular.z = angular_vel

# #     # Publish the velocity message for one second
# #     while (rospy.Time.now() - start_time).to_sec() < 1.0 and not rospy.is_shutdown():
# #         pub.publish(vel_msg)
# #         rate.sleep()

# #     start_time = rospy.Time.now()

# # Define the list of velocities for linear and angular

# U_L = ul_list
# U_R = ur_list
# linear_velocities = []
# angular_velocities = []

# r=0.033
# L= 0.160

# for i in range(len(U_L)):
#     U_L[i] = U_L[i] * 0.10472 *10
#     U_R[i] = U_R[i] * 0.10472 *10 
#     linear_velocity = r/2 * (U_L[i] + U_R[i])
#     linear_velocities.append(linear_velocity)

# for i in range(len(U_L)):
#     U_L[i] = U_L[i] * 0.10472 *10
#     U_R[i] = U_R[i] * 0.10472 *10
#     angular_velocity = -(r/L* (U_L[i] - U_R[i]))
#     angular_velocities.append(angular_velocity)


# # Publish the velocities from the lists one after the other at an interval of one second
# for i in range(len(linear_velocities)):
#     linear_vel = linear_velocities[i]
#     angular_vel = angular_velocities[i]

#     # Define the velocity message
#     vel_msg = Twist()
#     vel_msg.linear.x = linear_vel
#     vel_msg.angular.z = angular_vel 

#     # Publish the velocity message for one second
#     for j in U_L:
#         for k in U_R:
#             if j == k:
#                 while (rospy.Time.now() - start_time).to_sec() < 1.52 and not rospy.is_shutdown():
#                     pub.publish(vel_msg)
#                     rate.sleep()
            
#             else:
#                 while (rospy.Time.now() - start_time).to_sec() < 0.136 and not rospy.is_shutdown(): 
#                     pub.publish(vel_msg)
#                     rate.sleep()


    

#     # while (rospy.Time.now() - start_time).to_sec() < 1 and not rospy.is_shutdown():
#     #     pub.publish(vel_msg)
#     #     rate.sleep()

#     start_time = rospy.Time.now()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import random

# class Turtlebot3Publisher(Node):

#     def __init__(self):
#         super().__init__('turtlebot3_publisher')

#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer = self.create_timer(0.1, self.publish_velocity)  # Publish at 10 Hz

#         self.start_time = self.get_clock().now()

#         # Define lists for velocities (assuming ul_list and ur_list are defined elsewhere)
#         self.U_L = ul_list
#         self.U_R = ur_list
#         self.linear_velocities = []
#         self.angular_velocities = []

#         # Populate velocity lists (calculations remain the same)
#         r = 0.033
#         L = 0.160
#         for i in range(len(self.U_L)):
#             self.linear_velocities.append(r/2 * (self.U_L[i] + self.U_R[i]))
#             self.angular_velocities.append(-(r/L) * (self.U_L[i] - self.U_R[i]))

#         self.velocity_index = 0

#     def publish_velocity(self):
#         linear_vel = self.linear_velocities[self.velocity_index]
#         angular_vel = self.angular_velocities[self.velocity_index]

#         vel_msg = Twist()
#         vel_msg.linear.x = linear_vel
#         vel_msg.angular.z = angular_vel
#         print("linear_vel:",linear_vel,"angular_vel:",angular_vel)

#         self.publisher.publish(vel_msg)

#         # Update velocity index and handle timing
#         current_time = self.get_clock().now()
#         elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Time in seconds

#         if elapsed_time >= 1.0:
#             self.velocity_index += 1
#             self.start_time = current_time

#             if self.velocity_index >= len(self.linear_velocities):
#                 self.velocity_index = 0

# def main(args=None):
#     rclpy.init(args=args)
#     node = Turtlebot3Publisher()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class Turtlebot3Publisher(Node):

    def __init__(self):
        super().__init__('turtlebot3_publisher')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # Publish at 10 Hz

        self.start_time = self.get_clock().now()

        # Define lists for velocities (assuming ul_list and ur_list are defined elsewhere)
        self.U_L = ul_list
        self.U_R = ur_list
        self.linear_velocities = []
        self.angular_velocities = []

        # Populate velocity lists (calculations remain the same)
        r = 0.033
        # L = 0.160
        L = 0.287
        for i in range(len(self.U_L)):
            self.linear_velocities.append(r/2 * (self.U_L[i] + self.U_R[i]))
            self.angular_velocities.append(-(r/L) * (self.U_L[i] - self.U_R[i]))

        self.velocity_index = 0

    def publish_velocity(self):
        if self.velocity_index >= len(self.linear_velocities):
            self.stop_publishing()
            return

        linear_vel = self.linear_velocities[self.velocity_index]
        angular_vel = self.angular_velocities[self.velocity_index]

        vel_msg = Twist()
        vel_msg.linear.x = linear_vel* (1.3)
        vel_msg.angular.z = angular_vel*(1.1)
        print("linear_vel:",linear_vel,"angular_vel:",angular_vel)

        self.publisher.publish(vel_msg)

        # Update velocity index and handle timing
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Time in seconds

        if elapsed_time >= 1.0:
            self.velocity_index += 1
            self.start_time = current_time

    def stop_publishing(self):
        # Stop timer and potentially perform cleanup actions (optional)
        self.timer.cancel()
        print("Finished publishing velocities from all settlements in ul_list!")

def main(args=None):
    time.sleep(3.0)
    rclpy.init(args=args)
    node = Turtlebot3Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()