This is the first working version of teleop_abs!

The turtlebot will stop (overriding any teleop commands) when it is within 0.2 meters of an obstacle. It must be picked up and moved somewhere else to resume moving again, and it will begin moving at the velocity specified by teleop once it is safe to move. 


NOTE!!: My CMake file is messy. I'm not sure what's wrong with it right now, but I'll continue to work on that in future versions/branches. I'm not sure if this is an issue with my catkin workspace itself or my CMake file, but currently when I run catkin_make, it can't complete when it tried to build this package. 

Instructions to run:

Open a new terminal and execute command "export TURTLEBOT3_MODEL=burger". Run gazebo_abs.launch to get gazebo up and running. The .py files should run too. abs_system.launch starts the .py files independently (I was using this to troubleshoot when my .py files wouldn't launch, and decided to leave it in).

