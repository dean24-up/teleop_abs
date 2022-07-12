#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

##filter_scan
#Gets scan info from laser scan and publishes braking data to brake topic based on data


#original file is turtlebot_obstacle, above warranty is from that
# Authors (turtlebot_obstacle_): Gilbert #

#adapted by dean24

#get_scan is from turtlebot_obstacle from turtlebot_examples by ROBOTIS. Main is very similar to that program as well.
#referenced get_pose program in roswiki_tutorials of Turtlesim_Python_Files to get a better
#understanding of subscriber/publisher relationships and remapping

import rospy
import math
import get_scan #imported get_scan.py as a module, hopefully it'll do something
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

STOP_DISTANCE = 0.5
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

#Processes the scans from get scans and decides what brake messages to publish
def filter_scans():
    #while the program is not shut down
    while not rospy.is_shutdown(): 
        #variables to hold scan data  
        lidar_distances = get_scan("C")
        lidar_distances_left = get_scan("L")
        lidar_distances_right = get_scan("R")
        min_distance_left = min(lidar_distances_left)
        min_distance_right = min(lidar_distances_right)
        min_distance = min(lidar_distances)
        
        #if Turtlebot3 is about to collide with an obstacle
        if min_distance < SAFE_STOP_DISTANCE:   #if the closest lidar distance to the robot is less than safe stopping distance -s
           #I moved this out here because otherwise it would be nested within every instance
            if turtlebot_moving:
                 #if an obstacle to the left is closer than an obstacle to the right
                if min_distance_left < min_distance_right:     
                    rospy.loginfo('Turn Right!')
                    pub.publish('Turn Right!')
                    turtlebot_moving = False
                #else if obstacle on right is closer
                elif min_distance_left > min_distance_right:     
                    rospy.loginfo('Turn Left!')
                    pub.publish('Turn Left!')
                    turtlebot_moving = False
       
        #if the robot is safe to keep going
        else:
            turtlebot_moving = True
            rospy.loginfo('Distance to collision: %f', min_distance)
            rospy.loginfo('left distance : %s meters', min_distance_left)
            rospy.loginfo('right distance : %s meters', min_distance_right)
            pub.publish('Go!')


#initializes filter_scan
#if the turtlebot is within stopping distance, it sends a message to the brake topic directing the 
#turtlebot to stop
if __name__ == '__main__':
    try: 

        print("filter_scan is running!")
        #creates a node of the name filter_scan
        rospy.init_node('filter_scan')

        #initialize publisher for braking robot
        pub = rospy.Publisher('brake', String, queue_size=1000) 
        turtlebot_moving = True

        #run filter_scan
        filter_scans()

    except rospy.ROSInterruptException:
        pass



#this looks like a function that processes scans? Not sure if I need to understand this for obstacle avoidance 
# def get_scan():
#     scan = rospy.wait_for_message('scan', LaserScan)                      #waits for scan messages from LaserScan
#     scan_filter = []
    
#     samples = len(scan.ranges)  # The number of samples is defined in 
#                                 # turtlebot3_<model>.gazebo.xacro file,
#                                 # the default is 360.
#     samples_view = 1            # 1 <= samples_view <= samples
    
#     if samples_view > samples:
#         samples_view = samples

#     if samples_view == 1:
#         scan_filter.append(scan.ranges[0])

#     else:
#         left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
#         right_lidar_samples_ranges = samples_view//2
        
#         left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
#         right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
#         scan_filter.extend(left_lidar_samples + right_lidar_samples)

#     for i in range(samples_view):
#         if scan_filter[i] == float('Inf'):
#             scan_filter[i] = 3.5
#         elif math.isnan(scan_filter[i]):
#             scan_filter[i] = 0
    
#     return scan_filter
