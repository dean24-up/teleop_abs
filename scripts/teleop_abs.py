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

#original file is turtlebot_obstacle, above warranty is from that
# Authors: Gilbert #

#edited by dean24

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

##In its current state, the program should stop if it is within distance and you shouldn't be able to use teleop unless you pick up
#the turtle and move it.

#Otherwise, teleop should work normally

#referenced get_pose program in roswiki_tutorials of Turtlesim_Python_Files to get a better
#understanding of subscriber/publisher relationships and remapping


STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class Obstacle():
    def __init__(self):
        #creates a node of the name teleop_abs
        #rospy.init_node('teleop_abs')
        
        #Publisher which publishes to cmd_vel (stands for automatic braking system velocity)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000) 
        
        #Subscriber which subscribes to abs_vel. self.obstacle is called when a Twist msg received
        rospy.Subscriber('abs_vel', Twist, self.obstacle, queue_size=1000)
    
    #this looks like a function tha tprocesses scans? dunno if I need to understand this for obstacle avoidance -s
    def get_scan(self):
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


    def obstacle(self, data):

        print("I'm running!")
        
        #Take the Twist data received from cmd_vel and save it as vel_in
        vel_in = Twist()
        vel_in = data   
        
        #Declare variable to store twist data to be published to abs_vel as vel_out
        vel_out = Twist()
        
        turtlebot_moving = True

        #while the program is not shut down
        while not rospy.is_shutdown():       
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            #if it's necessary to stop
            if min_distance < SAFE_STOP_DISTANCE:   #if the closest lidar distance to the robot is less than safe stopping distance -s
                if turtlebot_moving:                #stop the robot immediately!! -s
                    vel_out.linear.x = 0.0
                    vel_out.angular.z = 0.0
                    self._cmd_pub.publish(vel_out)  #publish vel_out
                    rospy.loginfo('Stop!')
                    turtlebot_moving = False
            
            #if the robot is safe to keep going
            else:
                vel_out.linear.x = vel_in.linear.x   #otherwise, keep the velocity the same as from teleop
                vel_out.angular.z = vel_in.angular.z
                self._cmd_pub.publish(vel_out)       #publish vel_out
                turtlebot_moving = True
                rospy.loginfo('Distance of the obstacle : %f', min_distance)

def main():
    #creates a node of the name teleop_abs
    rospy.init_node('teleop_abs')

    try: 
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
