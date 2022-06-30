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

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

##In its current state, the program should stop if it is within distance and you shouldn't be able to use teleop unless you pick up
#the turtle and move it.

#Otherwise, teleop should work normally

#get_scan is from turtlebot_obstacle from turtlebot_examples by ROBOTIS. Main is very similar to that program as well.
#referenced get_pose program in roswiki_tutorials of Turtlesim_Python_Files to get a better
#understanding of subscriber/publisher relationships and remapping

STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR


#this looks like a function that processes scans? Not sure if I need to understand this for obstacle avoidance 
def get_scan():
    scan = rospy.wait_for_message('scan', LaserScan)                      #waits for scan messages from LaserScan
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

#initializes teleop_abs
#if the turtlebot is within stopping distance, it sends a message to the brake topic directing the 
#turtlebot to stop

if __name__ == '__main__':
    try: 

        print("filter_scan is running!")
        #creates a node of the name teleop_abs
        rospy.init_node('filter_scan')

        #initialize publisher for braking robot
        pub = rospy.Publisher('brake', String, queue_size=1000) 
        turtlebot_moving = True

        #while the program is not shut down
        while not rospy.is_shutdown():   #if I get rid of this it seems to delay the lidar sensing or something, the stop thing doesn't run    
            lidar_distances = get_scan()
            min_distance = min(lidar_distances)

            #if it's necessary to stop
            if min_distance < SAFE_STOP_DISTANCE:   #if the closest lidar distance to the robot is less than safe stopping distance -s
                if turtlebot_moving:                #stop the robot immediately!! -s         
                    rospy.loginfo('Stop!')
                    pub.publish('Stop!')
                    turtlebot_moving = False
            
            #if the robot is safe to keep going
            else:
                turtlebot_moving = True
                rospy.loginfo('Distance of the obstacle : %f', min_distance)
                pub.publish('Go!')

    except rospy.ROSInterruptException:
        pass