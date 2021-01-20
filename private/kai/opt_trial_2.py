from scipy.optimize import minimize
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import lines


##########################################################################################

global robot_width  #constant when turning
global robot_height # always constant
global ls_wall
global ls_corner


robot_width = 1.5                                        #constant when turning
robot_height = 2.2                                       #always constant
# ls_wall = [[(0,2),(3,2)],[(-3,0),(0,6)],[(-2,-2),(0,-2)]] #list of parameters fo each wall this is just a static value in future integrate function get_ls_wall with sensors to get values
ls_wall = [[(-3,0),(0,6)],[(-2,-2),(0,-2)]]

def get_ls_wall(): #place holder
    global ls_wall
    return ls_wall

def get_corners(): #erturns the coordinate of the four corners. Should eventually get the dynamic width value from the linear encoder
    global robot_width 
    global robot_height
    height = robot_height
    width = robot_width
    lf = (-width/2, height/2) #left front
    rf = (-width/2, height/2) #right front
    lb = (-width/2, -height/2) #left back
    rb = (width/2, -height/2) #right back
    return [lf,rf,lb,rb]

def get_wheel_angle():
    global robot_width
    global robot_height
    x1 = x[0]
    x2 = x[1]
    ls_corners = get_corners()
    
    x3 = ls_corners[0][0]        #lf.x
    x4 = ls_corners[0][1]        #lf.y
    x5 = ls_corners[1][0]        #rf.x
    x6 = ls_corners[1][1]        #rf.y
    x7 = ls_corners[2][0]        #lb.x  
    x8 = ls_corners[2][1]        #lb.y  
    x9 = ls_corners[3][0]        #rb.x
    x10 =ls_corners[3][1]        #rb.y

    lf_angle = math.atan(abs(x3-x1)/abs(x4-x1))
    rf_angle = math.atan(abs(x5-x1)/abs(x6-x1))
    lb_angle = math.atan(abs(x7-x1)/abs(x8-x1))
    rb_angle = math.atan(abs(x9-x1)/abs(x10-x1))

    return [lf_angle,rf_angle,lb_angle,rb_angle]  #return list of angles the wheels have to turn

def obj_fun(x):                                                     # returns maximum farthest
    x1 = x[0]                                                       # pivot.x
    x2 = x[1]
    length = (x1**2 + x2**2)**0.5
    return -length                                                  #negative cause running minimise contrain



def get_wall_constrains(x, wall_points): #takes in 2 arguments which is x , the coordinates of a particular corner of robot and wall_constrains whcih contains 2 points describing a particular wall
    global robot_width
    global robot_height
    x1 = x[0]                                                       # pivot.x
    x2 = x[1]                                                       # pivot.y
    x5 = wall_points[0]                                            # p1
    x6 = wall_points[1]                                             # p2
    p1_p2 = (x5[0]-x6[0],x5[1]-x6[1])                               # vector p1- p2 i.e vector of the line fo the wall
    p_p2 = (x1-x6[0],x2-x6[1])                                      # vector pivot - p2
    r2wall = np.abs(np.cross(p1_p2, p_p2)/(np.linalg.norm(p1_p2)))  # r2wall contains the shortest distance from the pivot to specific wall
    #corner_radius = ((x3-x1)**2+(x4-x2)**2)                        # corner radius is the radius of the area of the circle created by a particular corner at that particular turning radius
    return r2wall                                                   # returns the r2wall and corner radius
    
def get_corner_radius(x, corner):
    x1 = x[0]                                                       # pivot.x
    x2 = x[1]                                                       # pivot.y
    x3 = corner[0]
    x4 = corner[1]
    corner_radius = (((x3-x1)**2)+((x4-x2)**2))**0.5
    return corner_radius  


def constrain_function(x):
    global ls_wall
    ls_corner = get_corners()
    pivot = x
    ls_corner_radius = []
    ls_r2wall = []
    for corner in ls_corner:
        corner_radius = get_corner_radius(pivot, corner)
        ls_corner_radius.append(corner_radius)
    for wall in ls_wall:
        r2wall = get_wall_constrains(pivot, wall)
        ls_r2wall.append(r2wall)

    min_r2wall = min(ls_r2wall)
    max_corner_radius = max(ls_corner_radius)
    diff = (min_r2wall - max_corner_radius)
    
    # print(ls_r2wall)
    # print(max_corner_radius)
    # print(diff)
    return diff

def get_max_corner_radius(pivot):
    ls_corner = get_corners()
    ls_corner_radius = []
    for corner in ls_corner:
        corner_radius = get_corner_radius(pivot,corner)
        ls_corner_radius.append(corner_radius)
    max_corner_radius = max(ls_corner_radius)
    return max_corner_radius, ls_corner



def plot_graph(pivot):
    max_corner_radius, ls_corner_radius = get_max_corner_radius(pivot)
    ls_corner = get_corners() #[lf,rf,lb,rb]
    lb = ls_corner[2]
    rf = ls_corner[1]
    circle = plt.Circle(pivot, max_corner_radius, color = "r", fill = False)
    fig, ax = plt.subplots()
    ax.plot([-5, 5],[-5, 5], color = "none")

    for i in range(len(ls_wall)):
        x1 = [x for [x,y] in ls_wall[i]]
        y1 = [y for [x,y] in ls_wall[i]]
        line  = lines.Line2D(x1,y1, linestyle = "dashed", color = "k")
        #plt.axes().add_line(line)
        plt.plot(x1,y1, marker = "o")
    
    ax.add_patch(circle)
    ax.add_patch(Rectangle(lb, robot_width , robot_height, fill = False) )
    plt.scatter(pivot[0],pivot[1],10, color = 'g')
    plt.savefig('plot.png')
    plt.show()


constrain = {'type':"ineq", "fun":constrain_function}
ls_constrain = [constrain]

bounds_x = (-robot_width/2,robot_width/2)
bounds_y = (-robot_height/2,robot_height/2)
ls_bounds =[bounds_x,bounds_y]

x0 = [0,0]

result = minimize(obj_fun,x0, method = "SLSQP", bounds = ls_bounds, constraints=ls_constrain)
print(result)
pivot = result.x

    
plot_graph(pivot)

