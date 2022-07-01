This is the first working version of teleop_abs!

The turtlebot will stop (overriding any teleop commands) when it is within 0.2 meters of an obstacle. It must be picked up and moved somewhere else to resume moving again, and it will begin moving at the velocity specified by teleop once it is safe to move. 


NOTE!!: My CMake file is messy. I'm not sure what's wrong with it right now, but I'll continue to work on that in future versions/branches. I'm not sure if this is an issue with my catkin workspace itself or my CMake file, but currently when I run catkin_make, it can't complete when it tried to build this package. 

Instructions to run:

Open a new terminal and execute command "export TURTLEBOT3_MODEL=burger". Run gazebo_abs.launch to get gazebo up and running. The .py files should run too. abs_system.launch starts the .py files independently (I was using this to troubleshoot when my .py files wouldn't launch, and decided to leave it in).

Note on Launch Files:
gazebo_abs.launch remaps teleop publishing from cmd_vel to a new topic called teleop_vel. This is necessary so that abs_control.py can control which messages from teleop are sent to the turtlebot. If you choose to run this program without running gazebo_abs.launch, you need to remap cmd_vel to teleop_vel on the command line when starting up teleop. 

Scripts: 
filter_scan.py - subscribes to scan (LaserScan) topic of turtlebot3. Processes scans and publishes message to brake (String) topic specifying whether turtlebot should stop ("Stop!") or whether it is safe to keep moving ("Go!")

abs_control.py - subscribes to brake (String) topic and teleop_vel (Twist) topic. Publishes to cmd_vel (Twist). When it receives a "Stop!" message, stops turtlebot3. When it receives "Go!", it passes Twist messages from teleop_vel to cmd_vel. 
