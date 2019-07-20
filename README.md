# Multi Robot Simulation

# Introduction
In this repository, the temporal plan generated in [multi_agent_path_planning](https://github.com/atb033/multi_agent_path_planning) is simulated in Gazebo environment.



The code is written and tested in ROS Kinetic.


# Execution


- Launch the robots: 
'''
roslaunch robot_model full_env.launch
'''
- Trajectory control:
'''
rosrun robot_model trajectory_control.py
'''


