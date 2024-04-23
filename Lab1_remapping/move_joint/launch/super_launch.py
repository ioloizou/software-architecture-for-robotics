from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    for joint in ('right_e0', 'right_e1','left_e0','left_e1'):
        with sl.group(ns = joint):
            sl.include(package='move_joint', launch_file='slider_launch.py', launch_arguments = {'name': joint})

    return sl.launch_description() 