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

#turtlebot_abs_experimental (turtlebot_abs_exp)
#original file is turtlebot_obstacle, above warranty is from that
# Authors: Gilbert #

#edited by dean24

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None  #remember we need pub

##In its current state, the program should stop if it is within distance and you shouldn't be able to use teleop unless you pick up
#the turtle and move it.

#Otherwise, teleop should work normally

#referenced get_pose program in roswiki_tutorials of Turtlesim_Python_Files to get a better
#understanding of subscriber/publisher relationships and remapping

STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR


#this looks like a function tha tprocesses scans? dunno if I need to understand this for obstacle avoidance -s
def get_scan():
    scan = rospy.wait_for_message('scan', LaserScan)                      #waits for scan messages from LaserScan -s
    scan_filter = []
    
    samples = len(scan.ranges)  # The number of samples is defined in 
                                # turtlebot3_<model>.gazebo.xacro file,
                                # the default is 360.
    samples_view = 1            # 1 <= samples_view <= samples
    
    if samples_view > samples:
        samples_view = samples

    if samples_view == 1:
        scan_filter.append(scan.ranges[0])

    else:
        left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
        right_lidar_samples_ranges = samples_view//2
        
        left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
        right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
        scan_filter.extend(left_lidar_samples + right_lidar_samples)

    for i in range(samples_view):
        if scan_filter[i] == float('Inf'):
            scan_filter[i] = 3.5
        elif math.isnan(scan_filter[i]):
            scan_filter[i] = 0
    
    return scan_filter

#function to filter the velocities in and decide whether to publish them or not
def filter_vel(data):
    print("filter_vel is running!")
    #Take the Twist data received from cmd_vel and save it as vel_in
    vel_in = Twist()
    vel_in = data   

    #Declare variable to store twist data to be published to abs_vel as vel_out
    vel_out = Twist()

    turtlebot_moving = True

    #while the program is not shut down
    # I think this line below may be leading filter to only call once during the whole thing, because it gets into the loop of repeating
    #using the first data input
    #but the issue is if we delete it, then I don't think scans will update continuously
    while not rospy.is_shutdown():       
        lidar_distances = get_scan()
        min_distance = min(lidar_distances)

        #if it's necessary to stop
        if min_distance < SAFE_STOP_DISTANCE:   #if the closest lidar distance to the robot is less than safe stopping distance -s
            if turtlebot_moving:                #stop the robot immediately!! -s
                vel_out.linear.x = 0.0
                vel_out.angular.z = 0.0
                pub.publish(vel_out)  #publish vel_out
                rospy.loginfo('Stop!')
                turtlebot_moving = False
        
        #if the robot is safe to keep going
        else:
            vel_out.linear.x = vel_in.linear.x   #otherwise, keep the velocity the same as from teleop
            vel_out.angular.z = vel_in.angular.z
            pub.publish(vel_out)       #publish vel_out
            turtlebot_moving = True
            rospy.loginfo('Distance of the obstacle : %f', min_distance)

#initializes nodes and calls filter_vel
def main():
    try: 
    
        print("Main is running!")
        #creates a node of the name teleop_abs
        rospy.init_node('teleop_abs')

        #Subscribers and Publishers
        #Publisher which publishes to cmd_vel (stands for automatic braking system velocity)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000) 
            
        #Subscriber which subscribes to abs_vel. Calls filter_vel whenever it receives velocity data
        rospy.Subscriber('abs_vel', Twist, filter_vel, queue_size=1000)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
