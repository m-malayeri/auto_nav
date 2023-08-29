#!/usr/bin/env python3

import functools
import py_trees as pt
import py_trees_ros as ptr
import py_trees.console as console
from sensor_msgs.msg import LaserScan
import rclpy
import sys
from autonomous_map_navigate.behaviors import *



def create_root() -> pt.behaviour.Behaviour:
    """
    Method to structure the behavior tree to monitor battery status and start rotation if battery is low.
    Also, the robot will stop if it detects an obstacle in front of it.
    
    The "collison_avoidance" behavior tree extends the "battery_monitor" behavior tree by adding a new feature
    to avoid collison with obstacles. Whenever the robot is about to collide with an object, the robot will
    automatically stop, overriding the input commands. The robot can be controlled either by joystick,
    where the command is published on the '/joy' topic or by command that is published on '/cmd_vel' topic.
    The laser scan data will be stored in blackboard by reading '/scan' topic. When an obstacle
    is detected, the 'stop_motion' behavior will be executed. The stop_motion behavor is prioritized over
    the rotate behavior.
    """

    ## define nodes and behaviors

    # define root node
    root = pt.composites.Parallel(
        name="root",
        policy=pt.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )    

    """
    Create a sequence node called "Topics2BB" and a selector node called "Priorities"    
    """
    ### YOUR CODE HERE ###
    
    # topics2bb = pt.composites.Parallel(name="Topics2BB")
    # priorities = pt.composites.Selector(name="Priorities")

    topics2bb = pt.composites.Sequence(name="Topics2BB",memory=True)
    priorities = pt.composites.Selector(name="Priorities",memory=False)
    # wall = pt.composites.Selector(name="Wall",memory=False)
    action = pt.composites.Sequence(name="Action",memory=False)
    # cond = pt.composites.Sequence(name="Condition",memory=False)
    

    """
    Using the battery_status2bb class, create a node called "Battery2BB" which subscribes to the topic "/battery_voltage"
    and the laser_scan_2bb class, create a node called "LaserScan2BB" which subscribes to the topic "/scan"
    """    
    ### YOUR CODE HERE ###
    
    battery2bb = battery_status2bb(
        name="Battery2BB",
        topic_name="/battery_voltage"  
    )
    
    laserScan2BB = laser_scan_2bb(
    	name="LaserScan2BB",
    	topic_name="/scan",
        safe_range=0.2
    )

    odom2BB = position_wrt_odom(
        name = "Position2BB"
        
    )

    walldata = wall_get_data(
        name = "WallData"
        # topic_name="/scan"
        
    )
    
    # align = alignment(
    #     name = "Aligning",
    #     topic_name="/cmd_vel"
        
    # )
   
    
       

      
  	

    """
    Using the rotate class, create a node called "rotate_platform", and using the stop_motion class, create a node called "stop_platform"
    """
    ### YOUR CODE HERE ###
    
    rotate_platform = rotate(name="RotatePlatform", topic_name="/cmd_vel")
    
    stop_platform = stop_motion(name="StopPlatform",topic_name1="/cmd_vel")

    align = rotate_wrt_angle(name="Aligning",topic_name="/cmd_vel")

    move = move_wrt_distance(name="MoveSafe",topic_name="/cmd_vel")
	

    """
    Read the 'battery_low_warning' and 'collison_warning' from the blackboard and set a decorator node called "Battery Low?" to check if the battery is low 
    and "Colliding?" to check if any obstacle is within minimum distance.
    Please refer to the py_trees documentation for more information on decorators.
    """
    ### YOUR CODE HERE ###
    
    def check_battery_low_on_blackboard(blackboard):
        return blackboard.battery_low_warning 
     
    def check_collison_warn_on_blackboard(blackboard):
        return blackboard.collison_warning
    
    def get_ransac(blackboard):
        return blackboard.ransac_warn 

    def check_detect_on_blackboard(blackboard):
        return blackboard.detect_warning
    
    def check_wallwarn_on_blackboard(blackboard):
        return blackboard.wall_warn 
    
    


       

    
    blackboard = pt.blackboard.Blackboard()
    # blackboard.battery_low_warning = False
    # blackboard.wall_detect_warning = False
    # blackboard.check_warning = False
    # blackboard.ransac_warn = False
    blackboard.wall_warn = False
    
    battery_emergency = pt.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child = rotate_platform
    )
    
    collide_emergency = pt.decorators.EternalGuard(
        name="Colliding?",
        condition=check_collison_warn_on_blackboard,
        blackboard_keys={"collison_warning"},

        child = stop_platform
        # child = align

    )
    

    # logic = pt.decorators.EternalGuard(
    #     name="RANSAC&ROTATE?",
    #     condition=get_ransac,
    #     blackboard_keys={"ransac_warn"},
    #     child = walldata
    # ) 

    # detect = pt.decorators.EternalGuard(
    #     name="Detect?",
    #     condition=check_detect_on_blackboard,
    #     blackboard_keys={"detect_warning"},
    #     child = cond
    # ) 


    check = pt.decorators.EternalGuard(
        name="CheckingWall?",
        condition=check_wallwarn_on_blackboard,
        blackboard_keys={"wall_warn"},
        child = action
    ) 








    idle = pt.behaviours.Running(name="Idle")
    # idle1 = pt.behaviours.Running(name="Idle_for_RANSAC")
    # idle2 = pt.behaviours.Running(name="Idle_for_Action")

    """
    construct the behvior tree structure using the nodes and behaviors defined above
    """

    ### YOUR CODE HERE ###
    
    root.add_child(topics2bb)
    topics2bb.add_child(battery2bb)
    topics2bb.add_child(laserScan2BB)
    topics2bb.add_child(walldata)

     
    root.add_child(priorities)
    priorities.add_child(collide_emergency)
    priorities.add_child(battery_emergency)
    priorities.add_child(check)
    action.add_child(move)
    action.add_child(align)
    priorities.add_child(idle)


    # root.add_child(logic)
    # # wall.add_child(wall_detect)
    # wall.add_child(idle1)


    # root.add_child(action)
    # action.add_child(check)
    # action.add_child(idle2)








    

    return root

def main():
    """
    Main function initiates behavior tree construction
    """
    rclpy.init(args=None)
    # Initialising the node with name "behavior_tree"
    root = create_root()
    tree = ptr.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
        )

    # setup the tree
    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # frequency of ticks
    tree.tick_tock(period_ms=10)    
    
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
