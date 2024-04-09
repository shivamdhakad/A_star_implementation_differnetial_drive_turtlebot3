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


# Intialize Node class 
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

#Function to plot non_holnomic action curves for the robot
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

    Nodes_list = [] # save explored nodes
    Path_list = [] # save waypoint-nodes for final path
    open_node = {}  # open node dictionary
    closed_node = {}  # closed node dictionary
    # append start node to open list 
    open_node[start_node_id] = start_node
    # priority list to save nodes to be explored
    priority_list = []
    # turtlebot action moves    
    moves = [[rpm1, 0],[0, rpm1],[rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2],[rpm2, rpm1]]
    heapq.heappush(priority_list, [start_node.total_cost, start_node])

    # iterate the action sequece till the priority queue is empty
    while (len(priority_list) != 0):

        current_nodes = (heapq.heappop(priority_list))[1]
        current_id = UID(current_nodes)
        # check if goal node is reached
        if check_goal(current_nodes, goal_node):
            goal_node.parent = current_nodes.parent
            goal_node.total_cost = current_nodes.total_cost
            print("Goal Node reached")
            return 1, Nodes_list, Path_list
        # Add the popped node to the closed nodes dictionary
        if current_id in closed_node:  
            continue
        else:
            closed_node[current_id] = current_nodes
        
        del open_node[current_id]
        
        #All the possible moves are looped through
        for move in moves:
            # implement action_moves in a loop in sequence
            action = plot_curve(current_nodes.x, current_nodes.y, current_nodes.current_theta, move[0], move[1],
                            clearance, 0, Nodes_list, Path_list)
           
            #Check the node generated is valid
            if (action != None):

                #round-off the node coordinates to integer value
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

#Check function: To check generated node is not in obstacle space 
def WS(x, y, radius, clearance):
    
    #Total buffer space 
    total_space = radius + clearance
    # total_space = 0.05 + clearance
    #rectangular obstacle space
    obstacle2 = (x >= 1.5 - total_space) and (x <= 1.75 + total_space) and (y >= 1.00 - total_space)
    obstacle3 = (x >= 2.5 - total_space) and (x <= 2.75 + total_space) and (y <= 1.00 + total_space)

    #circular obstacle space
    obstacle1 = ((np.square(x - 4.2)) + (np.square(y - 1.2)) <= np.square(0.6 + total_space))
      
    #Define the Borders of the obstacle space  
    border1 = (x <= 0 + total_space)     
    border2 = (x >= 5.99 - total_space)
    border3 = (y <= 0 + total_space)
    border4 = (y >= 1.99 - total_space)

    # border obstacle space
    if obstacle1 or obstacle2 or obstacle3 or border1 or border2 or border3 or border4:
        return True
    else:
        return False
    
#Check function: to check node is validity in obstacle space
def move_validity(x,y, r,c):
    
    #Check for obstacle
    if WS(x, y, r, c):
        return False
    else:
        return True


# check function: to check if the current node is 15 eculidian distance from goal node
def check_goal(current, goal):

    #Distance between the current and goal nodes is calculated
    dt = dist((current.x, current.y), (goal.x, goal.y))

    #Check if the goal has been reached
    if dt < 0.15:
        return True
    else:
        return False
    
    
# backtracking function to genrate path node coordinates 
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
        
    #reverse the list order    
    x_path.reverse()
    y_path.reverse()
    theta_path.reverse()
    ul_list.reverse()
    ur_list.reverse()

    x = np.asarray(x_path)
    y = np.asarray(y_path)
    theta = np.array(theta_path)
    return x, y, theta, ul_list, ur_list


if __name__ == '__main__':

    #Width, height, robot radius, clearance, and RPMs are defined.
    width = 6
    height = 2
    robot_radius  = 0.140
    clearance = int(input("Enter clearance of robot (5-15): "))
    clearance = clearance/100
    # clearance = 15/100
    # clearance = float(clearance)

    RPM1 = int(input("Enter Low RPM (5-12): "))
    RPM2 = int(input("Ente High RPM (5-12): "))
    # RPM2 = 10
    # RPM1 = 6
    #User input starting x-coord and y-coord of the robot.
    start_x = 500/1000
    start_y = 1000/1000

    start_x = float(start_x)
    start_y = float(start_y)

    #User input the starting orientation of the robot.
    start_theta = int(0)

    #Starting orientation is rounded off to the nearest multiple of 30.
    number = int(start_theta)
    remainder = number % 30
    if remainder < 15:
      start_theta = number - remainder
    else:
      start_theta = number + (30 - remainder)
    
    #user input the goal x-coord and y-coord of the robot.
    goal_x = int(input("Enter goal X co-ordinates in mm: "))
    goal_y = int(input("Enter goal Y co-ordinates in mm: "))
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
    

    # start timer 
    timer_start = time.time()

    # calculate heuristic cost for start node
    c2g = dist((start_x,start_y), (goal_x, goal_y))
    total_cost =  c2g
    # intitallize start node
    start_node = Node(start_x, start_y,-1,start_theta,0,0,0,0,c2g,total_cost)
    goal_node = Node(goal_x, goal_y, -1,0,0,0,0,c2g,0,total_cost)
    # call a star search algorithm
    flag, Nodes_list, Path_list = a_star(start_node, goal_node,RPM1,RPM2,robot_radius,clearance)
    # call backtracking function
    x_path, y_path, theta_path, ul_list, ur_list = back_track(goal_node)
    print("U_L:",ul_list)
    print("U_R:",ur_list)


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

        # Define lists for rpm lists
        self.U_L = ul_list
        self.U_R = ur_list
        self.linear_velocities = []
        self.angular_velocities = []
        # calculate linear and angular velocities
        # r: wheel radius, L : wheel perpendicular distance
        r = 0.033 
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
        vel_msg.linear.x = linear_vel* (1.25)
        vel_msg.angular.z = angular_vel*(1.3)
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
        print("Finished publishing velocities!")

def main(args=None):
    time.sleep(3.0)
    rclpy.init(args=args)
    node = Turtlebot3Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()