#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import time
import sys
import os
# from context import RRT, utils
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from RRT_scan import RRT

from utils_scan import visualize_scan , make_obstacles_scan
from utils_scan import adjustable_random_sampler as sampler
from utils_scan import los_optimizer_scan as path_optimizer


import matplotlib.pyplot as plt

def callback(data):
    
    # List of obtacles as a list of lists of points
    # general_obstacle_list = [
    #     [ (8, 5), (7, 8), (2, 9), (3, 5) ],
    #     [ (3, 3), (3, 5), (5, 5), (5, 3) ], 
    # ]
    inf = 100
   
    scan = data.ranges

    # print scan

    # obstacle_list = general_obstacle_list

    # Instatiate rrt planner object
    my_tree = RRT(sample_area=(-5, 15), sampler=sampler, expand_dis=0.5)


    # Plan path while timing
    print('\n ' + '-'*30 +  "\n> Starting operation ...\n " + '-'*30 + '\n')
    start_time = time.time()

    path, node_list = my_tree((0, 0), (-2.5, 2.5), scan)
    print("Path planned.")

    print('\n ' + '-'*30 + "\n> Time taken: {:.4} seconds.\n ".format(time.time() - start_time) + '-'*30 + '\n')

    # Visualize tree
    # RRT.visualize_tree(node_list, obstacle_list)
    visualize_scan(path, scan)

    # Testing los path optimizer
    line_obs , pts = make_obstacles_scan(scan)
    print('\n ' + '-'*30 +  "\n> Starting operation ...\n " + '-'*30 + '\n')
    start_time = time.time()

    optimized_path = path_optimizer(path, line_obs)
    print("Path optimized.")

    print('\n ' + '-'*30 + "\n> Time taken: {:.4} seconds.\n ".format(time.time() - start_time) + '-'*30 + '\n')

    # # Visualize path
    visualize_scan(optimized_path, scan)
    
def listener():
     

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("scan", LaserScan , callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
