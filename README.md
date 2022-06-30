# teleop_abs

Working_V1 contains the first working version!

Run gazebo_abs.launch to get gazebo up and running. The .py files unfortunately terminate upon launch. I'm troubleshooting them now, but
just start them up in separate windows after gazebo_abs.launch (so run "python3 abs_control.py in one terminal and "python3 filter_scan.py"
in another).

I did most of this work on my local computer, which is why I just made a new branch. It differs significantly from main.

Currently, there is no way to get the robot to move again once it automatically brakes other than picking it up and moving it somewhere else. 
