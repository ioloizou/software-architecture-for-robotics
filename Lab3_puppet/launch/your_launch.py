#!/usr/bin/env python3
from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
     
    # Include sim or bridge launch file
    sl.include(package="baxter_simple_sim", launch_file="sim_launch.py", launch_dir = None, launch_arguments= {'lab': 'puppet'})

    # Make a string with the arguments    
    tf_args = "0 0 0.1 0 3.14 0 right_gripper left_gripper_desired"
    
	# Start the node tf node
    sl.node(package='tf2_ros', executable='static_transform_publisher', arguments=tf_args.split())
    
    # Start the puppet node
    sl.node(package='lab3_puppet', executable='puppet')
    
    return sl.launch_description() 
