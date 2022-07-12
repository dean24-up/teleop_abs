#!/usr/bin/env python
#module that contains get_scan only, from obstacle6_edit created 7/11/22

import rospy
from sensor_msgs.msg import LaserScan
#import math

#get_scan
#Takes argument "dir" that specifies "L" for left scan, "C" for center, "R" for right
def get_scan(dir):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []

        samples = len(scan.ranges)

        samples_view = 1

        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            match dir:
                case "L":
                    scan_filter.append(scan.ranges[90])
                case "C":
                    scan_filter.append(scan.ranges[0])
            
                case "R":
                    scan_filter.append(scan.ranges[-90])  
            
        else:
             left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
             right_lidar_samples_ranges = samples_view//2

             left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
             right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]

            #extend based on whether we're doing a left, center, or right scan
             match dir:
                case "L":
                    scan_filter.extend(left_lidar_samples)
                case "C":
                    scan_filter.extend(left_lidar_samples + right_lidar_samples)
            
                case "R":
                    scan_filter.extend(right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] == 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0

        return scan_filter
