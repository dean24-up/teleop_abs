# Working_V4

This is the fourth working version of teleop_abs!

The turtlebot will turn left or right when it is within 0.5 meters of an obstacle until it is no longer facing an obstacle. The direction it turns depends on whether there is an obstacle closer to the left or right. After the turn, the turtlebot will resume normal teleoperation.

bugs and note: sometimes turtlebot gets "confused" and turns rapidly left and right when faced with obstacles on multiple sides. Also, the fix used to get it to turn continuously was to turn turtlebot_moving to TRUE always, which is a bit messy, but easiest for now. ANGULAR_SPEED was reduced. 

Instructions to run:

Open a new terminal and execute command "export TURTLEBOT3_MODEL=burger". Run gazebo_abs.launch to run the program with Gazebo. Run tbot3_abs.launch to launch without Gazebo. Note, tbot3_abs.launch is intended to be run after turtlebot3 is brought up normally. 

abs_system.launch starts the .py files independently (I was using this to troubleshoot when my .py files wouldn't launch, and decided to leave it in).
NOTE: If the abs_control.py or filter_scan.py DON'T run upon launch (I've been having trouble with that), you can run them separately as py files in their own terminals. 

Note on Launch Files:
gazebo_abs.launch and tbot3_abs.launch remaps teleop publishing from cmd_vel to a new topic called teleop_vel. This is necessary so that abs_control.py can control which messages from teleop are sent to the turtlebot. If you choose to run this program without running gazebo_abs.launch, you need to remap cmd_vel to teleop_vel on the command line when starting up teleop. 

Scripts: 
filter_scan.py - subscribes to scan (LaserScan) topic of turtlebot3. Processes scans and publishes message to brake (String) topic specifying whether turtlebot should stop ("Stop!") or whether it is safe to keep moving ("Go!")

abs_control.py - subscribes to brake (String) topic and teleop_vel (Twist) topic. Publishes to cmd_vel (Twist). When it receives a "Stop!" message, stops turtlebot3, performs turn, and prompts for brake. When it receives "Go!", it passes Twist messages from teleop_vel to cmd_vel. 

get_scan.py - module that gets left, right, and center scans. 
