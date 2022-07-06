# Working_V2

This is the second working version of teleop_abs!

The turtlebot will pause for one second (overriding any teleop commands) and turn 45 degrees when it is within 0.4 meters (double distance of Working_V1) of an obstacle. After the turn, the turtlebot won't be able to move again until the user stops it (using 's') in teleop. You can also see these prompts in the terminal for abs_control.py


NOTE!!: My CMake file is STILL messy. I'm not sure what's wrong with it right now, but I'll continue to work on that in future versions/branches. I'm not sure if this is an issue with my catkin workspace itself or my CMake file, but currently when I run catkin_make, it can't complete when it tried to build this package. 

Instructions to run:

Open a new terminal and execute command "export TURTLEBOT3_MODEL=burger". Run gazebo_abs.launch to get gazebo up and running. The .py files should run too. abs_system.launch starts the .py files independently (I was using this to troubleshoot when my .py files wouldn't launch, and decided to leave it in).

Note on Launch Files:
gazebo_abs.launch remaps teleop publishing from cmd_vel to a new topic called teleop_vel. This is necessary so that abs_control.py can control which messages from teleop are sent to the turtlebot. If you choose to run this program without running gazebo_abs.launch, you need to remap cmd_vel to teleop_vel on the command line when starting up teleop. 

Scripts: 
filter_scan.py - subscribes to scan (LaserScan) topic of turtlebot3. Processes scans and publishes message to brake (String) topic specifying whether turtlebot should stop ("Stop!") or whether it is safe to keep moving ("Go!")

abs_control.py - subscribes to brake (String) topic and teleop_vel (Twist) topic. Publishes to cmd_vel (Twist). When it receives a "Stop!" message, stops turtlebot3, performs turn, and prompts for brake. When it receives "Go!", it passes Twist messages from teleop_vel to cmd_vel. 
