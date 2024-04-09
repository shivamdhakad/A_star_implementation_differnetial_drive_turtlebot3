#ENPM661 : PROJECT3 PHASE2
# TurtleBot: a_star implimentation

import numpy as np
import math
import matplotlib.pyplot as plt
import time
import heapq
from math import dist
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap

# Intiallize Node class
class Node:
    def __init__(self, x, y, parent, c_t, n_t, u_left, u_right, c_2_come, c_2_goal, cost):
        # Initializing the attributes of the Node class
        self.x = x
        self.y = y
        self.parent = parent
        self.c_t = c_t      #current node theta
        self.n_t = n_t      # new node theta
        self.u_left= u_left     #left wheel rpm
        self.u_right = u_right      #right wheel rpm
        self.c_2_come = c_2_come
        self.c_2_goal = c_2_goal
        self.cost = cost

    def __lt__(self, other): # node.cost caparing check
         return (self.c_2_come + self.c_2_goal) < (other.c_2_come + other.c_2_goal)


# X_i, Y_i,theta_i : current node goal_coordinates
# u_lefteftand u_right: left and right wheel rpm
# c: move actions from execute_function()
# plot: boolean value
# nodes: current explored node list 
# list_path: neighbhour nodes of current node-list 

# function to plot non-holonomic direction curve for the robot
def curves(x_i, y_i, Thetai, u_left, u_right, c, plot, nodes, list_path): 
    t = 0
    r = 0.033  # waffle wheel-radius   
    L = 0.287
    dt = 0.1
    cost = 0
    x_goal = x_i
    y_goal = y_i
    end_orientation = math.radians(Thetai)  # gefree to radians
    #calculating next new node coordinates and theta
    while t < 1:
        t += dt
        x_start = x_goal
        y_start = y_goal
        x_goal += r * 0.5 * (u_left+ u_right) * math.cos(end_orientation) * dt
        y_goal += r * 0.5 * (u_left+ u_right) * math.sin(end_orientation) * dt
        end_orientation += (r / L) * (u_right - u_left) * dt
        #if the goal lies in plot, calculate cost and append and then plot the path
        if movement_validty(x_goal, y_goal, r, c):
            if plot == 0:
                c_2_goal = dist((x_start, y_start), (x_goal, y_goal))
                cost += c_2_goal
                nodes.append((x_goal, y_goal))
                list_path.append((x_start, y_start))
            elif plot == 1:
                plt.plot([x_start, x_goal], [y_start, y_goal], color="red")
        else:
            return None
    # theta radian to degrees
    end_orientation = math.degrees(end_orientation)  
    return [x_goal, y_goal, end_orientation, cost, nodes, list_path]


# Generateing unique UID for each node in the dictionary
def UID(node):
    UID = 1000*node.x + 111*node.y 
    return UID

# A star search algorithm
def A_star(start_node, goal_node, rpm1, rpm2, radius, clearance):

    if goal_range_check(start_node, goal_node):
        return 1, None, None
    #define nodes, path, open list and closed lists
    nodes = []  # exlpored node list 
    list_path = [] # list to saveing node for backtracking function for rpm valuse
    closed_list = {}  # closed node list
    open_list = {}  # open node dictionary
    open_list[UID(start_node)] = start_node 
    pr_list = []    
    # action set for non-holonomic turtlebot
    moves = [[rpm1, 0], [0, rpm1], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]

    heapq.heappush(pr_list, [start_node.cost, start_node])
    # node exploration loop
    while len(pr_list) != 0:  # iterate the loop until the priority list is empty

        closed_nodes = heapq.heappop(pr_list)[1]
        c_id = UID(closed_nodes)
        
        #check condition if the goal is reached
        if goal_range_check(closed_nodes, goal_node):
            goal_node.parent = closed_nodes.parent
            goal_node.cost = closed_nodes.cost
            print("Goal_Node reached")
            return 1, nodes, list_path
        #append the node to closed list and pop from open list
        if c_id in closed_list:  
            continue
        else:
            closed_list[c_id] = closed_nodes
        del open_list[c_id]

        #define action set for current nodes and plot the curves
        for move in moves:
            action = curves(closed_nodes.x, closed_nodes.y, closed_nodes.c_t, move[0], move[1],
                            clearance, 0, nodes, list_path)
            
            #round of the coordinates to integer values
            if action is not None:
                angle = action[2]
                x = round(action[0] * 10) / 10
                y = round(action[1] * 10) / 10
                #nearest value which is multiple of 30
                theta = round(angle / 15) * 15
                
                #calculate cost to goal, total cost and make new nodes
                c_t = closed_nodes.n_t - theta
                c_2_goal = dist((x,y), (goal_node.x, goal_node.y))
                new_node = Node(x, y, closed_nodes, theta, c_t, move[0], move[1], closed_nodes.c_2_come+action[3], c_2_goal, closed_nodes.c_2_come+action[3]+c_2_goal)
                new_node_id = UID(new_node)
                #if the new node is a valid node, add the node in the closed list 
                if not movement_validty(new_node.x, new_node.y, radius, clearance):
                    continue
                elif new_node_id in closed_list:
                    continue
                #if in open list, new cost is less then old cost then append the parent and the cost
                if new_node_id in open_list:
                    if new_node.cost < open_list[new_node_id].cost:
                        open_list[new_node_id].cost = new_node.cost
                        open_list[new_node_id].parent = new_node

                else:
                    open_list[new_node_id] = new_node
                    heapq.heappush(pr_list, [ open_list[new_node_id].cost, open_list[new_node_id]])
            
    return 0, nodes, list_path


# define obstacle space using half plane method
def obstacle_space_half_plane(x, y, radius, clearance):
    #total buffer zone width
    # c = radius + clearance
    c = 0.220 + clearance

    # Circular-obstacle
    circle = (x - 4.2)**2 + (y - 1.2)**2 <= (0.6 + c)**2 
    
    # rectangular-obstacles
    rectangle2 = (1.5 - c <= x <= 1.75 + c) and (1.0 - c <= y)   
    rectangle3 = (2.5 - c <= x <= 2.75 + c) and (y <= 1.0 + c)   
 
    # defining workspace border
    border1 = x <= 0 + c     
    border2 = x >= 5.99 - c
    border3 = y <= 0 + c
    border4 = y >= 1.99 - c

    # Check the presence of obstacle and border and returns a bool value
    if circle or rectangle2 or rectangle3 or border1 or border2 or border3 or border4:
        return True
    else:
        return False

# check function: to check if the node genrate by move_action function is valid
def movement_validty(x,y, r,c):
    
    if obstacle_space_half_plane(x, y, r, c):
        return False
    else:
        return True

# check function: to check if the current node is 15 eculidian distance from goal node
def goal_range_check(c, g):
    dt = dist((c.x, c.y), (g.x, g.y))
    if dt < 0.15:
        return True
    else:
        return False
    
# backtracking function to genrate path node coordinates 
def backtracker(goal_node): 
    x_path, y_path, theta_path = [goal_node.x], [goal_node.y], [goal_node.c_t]
    u_left_list,u_right_list = [goal_node.u_left], [goal_node.u_right]
    parent_node = goal_node.parent

    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        theta_path.append(parent_node.c_t)
        u_left_list.append(parent_node.u_left)
        u_right_list.append(parent_node.u_right)
        parent_node = parent_node.parent

    x_path.reverse()   
    y_path.reverse()
    theta_path.reverse()
    u_left_list.reverse()
    u_right_list.reverse()
    # printing the left and right wheel rpm as a list for the path following
    print("rpm action list")
    print(u_left_list) 
    print(u_right_list)

    x, y, theta = np.asarray(x_path), np.asarray(y_path), np.array(theta_path)
    return x, y, theta

# define an map canvas for video file 
environment = np.full((2, 6), 0) # map canvas
def plot(start_node, goal_node, x_path, y_path, list_path, environment):
    print("plotting path_line...")
    fig, axes = plt.subplots()
    axes.set(xlim=(0, 6), ylim=(0, 2))
    my_colors = np.array([(150, 210, 230), (255, 128, 0), (130, 130, 130)], dtype=float) / 255  # Normalize values between 0 and 1
    custom_cmap = ListedColormap(my_colors)
    axes.imshow(environment, cmap= custom_cmap)
    circle = plt.Circle((4.2, 1.2), 0.6, color='blue')

    rectangle2 = patches.Rectangle((1.5, 1.0), 0.15, 1.00, color= 'blue')
    rectangle3 = patches.Rectangle((2.5, 0), 0.15, 1.00, color='blue')

    #Buffer space for obstacles
    buffer_circle = plt.Circle((4.2, 1.2), 0.6 + 0.0001*clearance+robot_radius, color='red' , alpha = 0.5)

    buffer_rectangle2 = patches.Rectangle((1.5 - (0.001*clearance+robot_radius), 1.00 -  (0.001*clearance+robot_radius)), 0.15 + (2 * (0.001*clearance+robot_radius)), 1.15 +  (0.001*clearance+robot_radius), color= 'red' , alpha = 0.5)
    buffer_rectangle3 = patches.Rectangle((2.5 - (0.001*clearance+robot_radius), 0 -(0.001*clearance+robot_radius)), 0.15 + (2 * (0.001*clearance+robot_radius)), 1.15 + (0.001*clearance+robot_radius), color='red' , alpha = 0.5)
    border_rectangle = patches.Rectangle(
    (0 + 0.001*clearance+robot_radius, 0 + 0.001*clearance+robot_radius),          # Lower-left corner (x, y)
    6 - 2* (0.001*clearance+robot_radius),           # Width of the rectangle
    2 - 2 * (0.001*clearance+robot_radius),          # Height of the rectangle
    linewidth=1,
    edgecolor='red',
    facecolor='None'                 # No fill color
)
    border_rectangle2 = patches.Rectangle(
    (6 - (0.001*clearance+robot_radius), 2 - (0.001*clearance+robot_radius)),          # Lower-left corner (x, y)
    0.001*(clearance+robot_radius),           # Width of the rectangle
    0.001*(clearance+robot_radius),          # Height of the rectangle
    linewidth=1,
    edgecolor='red',
    facecolor='None'                 # No fill color
)
    
    axes.set_aspect('equal')
    axes.add_artist(buffer_rectangle2)
    axes.add_artist(buffer_rectangle3)
    axes.add_artist(buffer_circle)
    axes.add_artist(border_rectangle)
    # axes.add_artist(border_rectangle2)
    # axes.add_artist(buffer_circle)
    axes.add_artist(rectangle2)
    axes.add_artist(rectangle3)
    axes.add_artist(circle)
    # axes.invert_yaxis()  # Flip the y-axis to match the coordinate system
    #  Mark start and goal
    axes.plot(start_node.x, start_node.y, "Dw", markersize=3)  # Start
    axes.plot(goal_node.x, goal_node.y, "Dr", markersize=3)  # Goal

    path_line, = axes.plot([], [], 'y', lw=2)  # Initialize the line for the path

    def init():
        path_line.set_data([], [])
        return path_line,

    def update(frame):
        # Update the path line to include up to the current frame
        path_line.set_data(x_path[:frame], y_path[:frame])
        return path_line,

    frames = len(x_path)  # One frame for each step in the path

    anim = FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, repeat=False)

    # Save the animation
    anim.save('optimal_path_animation.mp4', writer='ffmpeg', fps=40)
    # print("plot_path_funtion_executed")
    plt.show()


def plot_node_exploration(start_node, goal_node, list_path, environment):
    print("exploringoal_nodes...")
    fig, axes = plt.subplots()
    axes.set(xlim=(0, 6), ylim=(0, 2))
    my_colors = np.array([(150, 210, 230), (255, 128, 0), (130, 130, 130)], dtype=float) / 255  # Normalize values between 0 and 1
    custom_cmap = ListedColormap(my_colors)
    rectangle2 = patches.Rectangle((1.5, 1.00), 0.15, 1.00, color= 'blue')
    rectangle3 = patches.Rectangle((2.5, 0), 0.15, 1.00, color='blue')
    circle = plt.Circle((4.2, 1.2), 0.6, color='blue')
    
    #Buffer space for obstacles
    buffer_circle = plt.Circle((4.2, 1.2), 0.6 + (0.001*clearance+robot_radius), color='red' , alpha = 0.5)
    buffer_rectangle2 = patches.Rectangle((1.5 - (0.001*clearance+robot_radius), 1.00 -  (0.001*clearance+robot_radius)), 0.15 + (2 * (0.001*clearance+robot_radius)), 1.15 +  (0.001*clearance+robot_radius), color= 'red' , alpha = 0.5)
    buffer_rectangle3 = patches.Rectangle((2.5 - (0.001*clearance+robot_radius), 0 - (0.001*clearance+robot_radius)), 0.15 + (2 * (0.001*clearance+robot_radius)),  1.15 + (0.001*clearance+robot_radius), color='red' , alpha = 0.5)
    border_rectangle = patches.Rectangle((0 + 0.001*clearance+robot_radius, 0 + 0.001*clearance+robot_radius),          # Lower-left corner (x, y)
    6 - 2*(0.001*clearance+robot_radius),           # Width of the rectangle
    2 - 2*(0.001*clearance+robot_radius),          # Height of the rectangle
    linewidth=1,
    edgecolor='red',
    facecolor='None'                 # No fill color
)
    border_rectangle2 = patches.Rectangle(
    (6 - (0.001*clearance+robot_radius), 2 - (0.001*clearance+robot_radius)),          # Lower-left corner (x, y)
    0.001*(clearance+robot_radius),           # Width of the rectangle
    0.001*(clearance+robot_radius),          # Height of the rectangle
    linewidth=1,
    edgecolor='red',
    facecolor='None'                 # No fill color
)
    axes.set_aspect('equal')
    axes.add_artist(circle)
    axes.add_artist(buffer_rectangle2)
    axes.add_artist(buffer_rectangle3)
    axes.add_artist(buffer_circle)
    axes.add_artist(border_rectangle)
    # axes.add_artist(border_rectangle2)
    # axes.add_artist(buffer_circle)
    axes.add_artist(rectangle2)
    axes.add_artist(rectangle3)
    axes.add_artist(circle)
    # axes.invert_yaxis()  # Flip the y-axis to match the coordinate system
    #  Mark start and goal
    axes.plot(start_node.x, start_node.x, "Dw", markersize=4 , label='Start')  # Start
    axes.plot(goal_node.x, goal_node.y, "Dr", markersize=4 , label='Goal')  # Goal

    explored_nodes, = axes.plot([], [], "ob", alpha=0.8, markersize=0.5, label='Explored Nodes')  # Initialize the line for explored nodes

    def init():
        explored_nodes.set_data([], [])
        return explored_nodes,

    def update(frame):
         # Determine the range of points to display in this frame
        if frame >= len(list_path):
            return explored_nodes,
        # Calculate the end_index for the current frame
        end_index = min((frame + 1) * max_nodes_per_frame, len(list_path))

    # Extract coordinates up to end_index
        x_coords, y_coords = zip(*[(wp[0], wp[1]) for wp in list_path[:end_index]])

        explored_nodes.set_data(x_coords, y_coords)
        return explored_nodes,

    if len(list_path)> 200:
        max_nodes_per_frame = 1000
    else:max_nodes_per_frame = 100
    # Use total path length for frames
    frames = (len(list_path) + max_nodes_per_frame) // max_nodes_per_frame
    first_plot_frame_T_no = frames
    total_frames = first_plot_frame_T_no + len(x_path)
    print("creating video file")
    print("total_frames:", total_frames)

    anim = FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, repeat=False, interval=100)
    # Save the animation
    anim.save('node_exploration_animation.mp4', writer='ffmpeg', fps=30)
    # print("video file saved")
    # plt.legend()
    plt.show()


if __name__ == '__main__':
    
    #define the workspace and robot dimensions
    width = 6
    height = 2
    robot_radius = 0.220
    # robot_radius = 0.01
    #define the clearence
    clearance = int(input("Enter clearance of robot (1-15): "))/1000
    # clearance = 0.220 + clearance 
    #define the high and low rpms
    RPM1 = int(input("Enter Low RPM (5-14): "))
    RPM2 = int(input("Enter the High RPM (5-14): "))
    #take start and goal coordinates
    start_coordinates = input("Enter starting point X, Y , Orientation seperated by spaces in mm ")
    start_x, start_y,start_theta = start_coordinates.split()
    start_x = int(start_x)/1000
    start_x = float(start_x)
    start_y = int(start_y)/1000
    start_y = float(start_y)
    start_theta = int(start_theta)

    num = int(start_theta)
    remainder = num % 30
    if remainder < 15:
        start_theta = num - remainder
    else:
        start_theta = num + (30 - remainder)
                      
    goal_coordinates = input("Enter Goal node x coordinte, y coordinate seperated by spaces in mm ")
    goal_x,goal_y = goal_coordinates.split()
    goal_x = int(goal_x)/1000
    goal_x = float(goal_x)
    goal_y = int(goal_y)/1000
    goal_y = float(goal_y)

    #checking start and goal nodes
    if not movement_validty(start_x, start_y, robot_radius, clearance):
        print("Start node is either invalid or in obstacle space")
        exit(-1)

    if not movement_validty(goal_x, goal_y, robot_radius, clearance):
        print("Goal node is either invalid or in obstacle space")
        exit(-1)
    #start timer 
    timer_start = time.time()  
    
    # append intial start node to the priority list 
    c_2_goal = dist((start_x, start_y), (goal_x, goal_y))
    cost = c_2_goal
    start_node = Node(start_x, start_y, -1, start_theta, 0, 0, 0, 0, c_2_goal, cost)
    goal_node = Node(goal_x, goal_y, -1, 0, 0, 0, 0, c_2_goal, 0, cost)
    # calling A_star function
    flag, nodes, list_path = A_star(start_node, goal_node, RPM1, RPM2, robot_radius, clearance)
    
    # calling backtracking function
    x_path, y_path, theta_path = backtracker(goal_node)
    plot_node_exploration(start_node, goal_node, list_path, environment)

    plot(start_node, goal_node, x_path, y_path, list_path, environment)

    # #plotting figures, axes for final plot
    # fig, axes = plt.subplots()
    # axes.set(xlim=(0, 6), ylim=(0, 2))

    # circle = plt.Circle((4.2, 1.2), 0.6, color='blue')

    # rectangle2 = patches.Rectangle((1.5, 0.75), 0.15, 1.25, color= 'blue')
    # rectangle3 = patches.Rectangle((2.5, 0), 0.15, 1.25, color='blue')


    # #Buffer space for obstacles
    # buffer_circle = plt.Circle((4.2, 1.2), 0.6 + clearance, fill='True', color='red' , alpha = 0.5)

    # buffer_rectangle2 = patches.Rectangle((1.5 - clearance, 0.75 -  clearance), 0.15 + (2 * clearance), 1.25 +  clearance, color= 'red' , alpha = 0.5)
    # buffer_rectangle3 = patches.Rectangle((2.5 - clearance, 0 -clearance), 0.15 + (2 * clearance), 1.25 + clearance, color='red' , alpha = 0.5)
    # axes.set_aspect('equal')
    # axes.add_artist(circle)
    # axes.add_artist(buffer_rectangle2)
    # axes.add_artist(buffer_rectangle3)
    # # axes.add_artist(buffer_circle)
    # axes.add_artist(rectangle2)
    # axes.add_patch(rectangle3)

    # plt.plot(start_node.x, start_node.y, "Dc")
    # plt.plot(goal_node.x, goal_node.y, "Dg")

    # for l in range(len(nodes)):
    #     plt.plot([list_path[l][0], nodes[l][0]], [list_path[l][1], nodes[l][1]], color="red")
    # plt.plot(x_path, y_path, ':g')
    # #stopping final timer
    # timer_stop = time.time()

    # count_time = timer_stop - timer_start
    # print("The Total Runtime is: ", count_time)

    # plt.show()
    # # plt.close('all')