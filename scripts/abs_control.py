#!/usr/bin/env python

#abs_control
#program that decides whether or not to publish velocities from teleop based on info from brake topic
# or whether or not to stop robot
#by dean_24

import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
pub = None 
teleop_vel = Twist() #stores the most recent input from teleop

#Twist to stop robot completely
stop = Twist()
stop.linear.x = 0
stop.linear.y = 0
stop.linear.z = 0
stop.angular.x = 0
stop.angular.y = 0
stop.angular.z = 0

#change teleop_vel to incoming data
def update_vel(data):
    #print('Updating Velocity')
    teleop_vel.linear.x = data.linear.x
    teleop_vel.angular.z = data.angular.z 

#publish messages to cmd_vel based on input received from brake topic
#The .data of a String msg is the actual string
def stop_or_go(data):
    if data.data == 'Stop!':
        pub.publish(stop)
    elif data.data == 'Go!':
        pub.publish(teleop_vel)
    else:
        print("Error: string not recognized")

if __name__ == '__main__':
    try: 
        print("Main is running!")
        
        #creates a filter_vel node
        rospy.init_node('abs_control')

        #Subscribers and Publishers
        #Publisher which publishes to cmd_vel (stands for automatic braking system velocity)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000) 
            
        #Subscriber which subscribes to teleop_vel. Calls update_vel whenever it receives velocity data
        teleop = rospy.Subscriber('teleop_vel', Twist, update_vel, queue_size=1000)
        
        #Subscriber which subscribes to brake. Calls stop_or_go whenever it receives String data
        brake = rospy.Subscriber('brake', String, stop_or_go)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass