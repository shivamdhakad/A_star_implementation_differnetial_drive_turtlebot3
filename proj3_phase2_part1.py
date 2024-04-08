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
    def __init__(self, x, y, parent, c_t, n_t, U_L, U_R, cost_to_come, cost_to_goal, cost):
        # Initializing the attributes of the Node class
        self.x = x
        self.y = y
        self.parent = parent
        self.c_t = c_t      #current node theta
        self.n_t = n_t      # new node theta
        self.U_L = U_L      #left wheel rpm
        self.U_R = U_R      #right wheel rpm
        self.cost_to_come = cost_to_come
        self.cost_to_goal = cost_to_goal
        self.cost = cost

    def __lt__(self, other):
        # Implementing comparison based on total cost, i.e., cost_to_come + cost_to_goal
        return (self.cost_to_come + self.cost_to_goal) < (other.cost_to_come + other.cost_to_goal)


# X_i, Y_i,theta_i : current node goal_coordinates
# U_L and U_R: left and right wheel rpm
# c: move actions from execute_function()
# plot: boolean value
# node_lists: current explored node list 
# find_path_list: neighbhour nodes of current node- list 
# function to plot non-holonomic direction curve for the robot
def curves(x_i, y_i, Thetai, U_L, U_R, c, plot, node_lists, find_path_list): 
    t = 0
    r = 0.033  # waffle wheel-radius   
    L = 0.287
    dt = 0.1
    cost = 0
    x_g = x_i
    y_g = y_i
    Theta_end = math.radians(Thetai)  # gefree to radians
    #calculating next new node coordinates and theta
    while t < 1:
        t += dt
        x_s = x_g
        y_s = y_g
        x_g += r * 0.5 * (U_L + U_R) * math.cos(Theta_end) * dt
        y_g += r * 0.5 * (U_L + U_R) * math.sin(Theta_end) * dt
        Theta_end += (r / L) * (U_R - U_L) * dt
        #if the goal lies in plot, calculate cost and append and then plot the path
        if execute_action(x_g, y_g, r, c):
            if plot == 0:
                cost_to_goal = dist((x_s, y_s), (x_g, y_g))
                cost += cost_to_goal
                node_lists.append((x_g, y_g))
                find_path_list.append((x_s, y_s))
            elif plot == 1:
                plt.plot([x_s, x_g], [y_s, y_g], color="red")
        else:
            return None
    # theta radian to degrees
    Theta_end = math.degrees(Theta_end)  
    return [x_g, y_g, Theta_end, cost, node_lists, find_path_list]


# Generateing unique key for each node in the dictionary
def key(node):
    key = 1000*node.x + 111*node.y 
    return key

# A star search algorithm
def Astar(s_node, g_node, rpm1, rpm2, radius, clearance):

    if check_goal(s_node, g_node):
        return 1, None, None
    #define nodes, path, open list and closed lists
    node_lists = []  # exlpored node list 
    find_path_list = [] # list to saveing node for backtracking function for rpm valuse
    closed_list = {}  # closed node list
    open_list = {}  # open node dictionary
    open_list[key(s_node)] = s_node 
    pr_list = []    
    # action set for non-holonomic turtlebot
    moves = [[rpm1, 0], [0, rpm1], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]

    heapq.heappush(pr_list, [s_node.cost, s_node])
    # node exploration loop
    while len(pr_list) != 0:  # iterate the loop until the priority list is empty

        c_nodes = heapq.heappop(pr_list)[1]
        c_id = key(c_nodes)
        
        #check condition if the goal is reached
        if check_goal(c_nodes, g_node):
            g_node.parent = c_nodes.parent
            g_node.cost = c_nodes.cost
            print("Goal_Node reached")
            return 1, node_lists, find_path_list
        #append the node to closed list and pop from open list
        if c_id in closed_list:  
            continue
        else:
            closed_list[c_id] = c_nodes
        del open_list[c_id]

        #define action set for current nodes and plot the curves
        for move in moves:
            action = curves(c_nodes.x, c_nodes.y, c_nodes.c_t, move[0], move[1],
                            clearance, 0, node_lists, find_path_list)
            
            #round of the coordinates to integer values
            if action is not None:
                angle = action[2]
                x = round(action[0] * 10) / 10
                y = round(action[1] * 10) / 10
                #nearest value which is multiple of 30
                theta = round(angle / 15) * 15
                
                #calculate cost to goal, total cost and make new nodes
                c_t = c_nodes.n_t - theta
                cost_to_goal = dist((x,y), (g_node.x, g_node.y))
                new_node = Node(x, y, c_nodes, theta, c_t, move[0], move[1], c_nodes.cost_to_come+action[3], cost_to_goal, c_nodes.cost_to_come+action[3]+cost_to_goal)
                new_node_id = key(new_node)
                #if the new node is a valid node, add the node in the closed list 
                if not execute_action(new_node.x, new_node.y, radius, clearance):
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
            
    return 0, node_lists, find_path_list


# define obstacle space using half plane method
def half_plane_obstcles(x, y, radius, clearance):
    #total buffer zone width
    c = radius + clearance 

    # Circular-obstacle
    circle = (x - 4)**2 + (y - 1.1)**2 <= (0.5 + c)**2 
    
    # rectangular-obstacles
    rectangle2 = (1.5 - c <= x <= 1.625 + c) and (0.75 - c <= y)   
    rectangle3 = (2.5 - c <= x <= 2.625 + c) and (y <= 1.25 + c)   
 
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
def execute_action(x,y, r,c):
    
    if half_plane_obstcles(x, y, r, c):
        return False
    else:
        return True

# check function: to check if the current node is 15 eculidian distance from goal node
def check_goal(c, g):
    dt = dist((c.x, c.y), (g.x, g.y))
    if dt < 0.15:
        return True
    else:
        return False
    


# backtracking function to genrate path node coordinates 
def backtracker(g_node): 
    x_path, y_path, theta_path = [g_node.x], [g_node.y], [g_node.c_t]
    U_L_list,U_R_list = [g_node.U_L], [g_node.U_R]
    parent_node = g_node.parent

    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        theta_path.append(parent_node.c_t)
        U_L_list.append(parent_node.U_L)
        U_R_list.append(parent_node.U_R)
        parent_node = parent_node.parent

    x_path.reverse()   
    y_path.reverse()
    theta_path.reverse()
    U_L_list.reverse()
    U_R_list.reverse()
    # printing the left and right wheel rpm as a list for the path following
    print("rpm action list")
    print(U_L_list) 
    print(U_R_list)

    x, y, theta = np.asarray(x_path), np.asarray(y_path), np.array(theta_path)
    return x, y, theta

# define an map canvas
environment = np.full((2, 6), 0) # map canvas
def plot(s_node, g_node, x_path, y_path, find_path_list, environment):
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
    axes.plot(s_node.x, s_node.y, "Dw", markersize=3)  # Start
    axes.plot(g_node.x, g_node.y, "Dr", markersize=3)  # Goal

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
def plot_node_exploration(s_node, g_node, find_path_list, environment):
    print("exploring_nodes...")
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
    border_rectangle = patches.Rectangle(
    (0 + 0.001*clearance+robot_radius, 0 + 0.001*clearance+robot_radius),          # Lower-left corner (x, y)
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
    axes.plot(s_node.x, s_node.x, "Dw", markersize=4 , label='Start')  # Start
    axes.plot(g_node.x, g_node.y, "Dr", markersize=4 , label='Goal')  # Goal
    # print("length of all_node:", len(find_path_list))
    # Mark start and goal

    # explored_nodes, = ax.plot([], [], "ob-", alpha=0.6, markersize=3, label='Explored Nodes')  # Initialize the line for explored nodes
    explored_nodes, = axes.plot([], [], "ob", alpha=0.8, markersize=0.5, label='Explored Nodes')  # Initialize the line for explored nodes

    def init():
        explored_nodes.set_data([], [])
        return explored_nodes,

    def update(frame):
         # Determine the range of points to display in this frame
        if frame >= len(find_path_list):
            return explored_nodes,
        # Calculate the end_index for the current frame
        end_index = min((frame + 1) * max_nodes_per_frame, len(find_path_list))

    # Extract coordinates up to end_index
        x_coords, y_coords = zip(*[(wp[0], wp[1]) for wp in find_path_list[:end_index]])

        explored_nodes.set_data(x_coords, y_coords)
        return explored_nodes,

    if len(find_path_list)> 200:
        max_nodes_per_frame = 1000
    else:max_nodes_per_frame = 100
    # frames = len(x_path)  # Use total path length for frames
    frames = (len(find_path_list) + max_nodes_per_frame) // max_nodes_per_frame
    first_plot_frame_T_no = frames
    total_frames = first_plot_frame_T_no + len(x_path)
    print("creating video file")
    print("total_frames:", total_frames)

    anim = FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, repeat=False, interval=100)
    # Save the animation
    anim.save('node_exploration_animation.mp4', writer='ffmpeg', fps=30)
    # print("video file saved")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    
    #define the workspace and robot dimensions
    width = 6
    height = 2
    robot_radius = 0.220
    # robot_radius = 0.01
    #define the clearence
    clearance = int(input("Enter clearance of robot in mm: "))/1000
    # clearance = 0.220 + clearance 
    #define the high and low rpms
    RPM1 = int(input("Enter the Low RPM: "))
    RPM2 = int(input("Enter the High RPM: "))
    #take start and goal coordinates
    start_coordinates = input("Enter starting x coordintes, y coordinates and orientation seperated by spaces in mm ")
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
    if not execute_action(start_x, start_y, robot_radius, clearance):
        print("Start node is either invalid or in obstacle space")
        exit(-1)

    if not execute_action(goal_x, goal_y, robot_radius, clearance):
        print("Goal node is either invalid or in obstacle space")
        exit(-1)
    #starting timer 
    timer_start = time.time()
    #taking c2g
    cost_to_goal = dist((start_x, start_y), (goal_x, goal_y))
    #adding total cost
    cost = cost_to_goal
    #append start node
    s_node = Node(start_x, start_y, -1, start_theta, 0, 0, 0, 0, cost_to_goal, cost)
    #append goal node
    g_node = Node(goal_x, goal_y, -1, 0, 0, 0, 0, cost_to_goal, 0, cost)
    #start Astar
    flag, node_lists, find_path_list = Astar(s_node, g_node, RPM1, RPM2, robot_radius, clearance)
    #start backtracking
    x_path, y_path, theta_path = backtracker(g_node)
    plot_node_exploration(s_node, g_node, find_path_list, environment)

    plot(s_node, g_node, x_path, y_path, find_path_list, environment)

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

    # plt.plot(s_node.x, s_node.y, "Dc")
    # plt.plot(g_node.x, g_node.y, "Dg")

    # for l in range(len(node_lists)):
    #     plt.plot([find_path_list[l][0], node_lists[l][0]], [find_path_list[l][1], node_lists[l][1]], color="red")
    # plt.plot(x_path, y_path, ':g')
    # #stopping final timer
    # timer_stop = time.time()

    # count_time = timer_stop - timer_start
    # print("The Total Runtime is: ", count_time)

    # plt.show()
    # # plt.close('all')