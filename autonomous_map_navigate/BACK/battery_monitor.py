#!/usr/bin/env python3s

import functools
import py_trees as pt
import py_trees_ros as ptr
import py_trees.console as console
import rclpy
import sys
from autonomous_map_navigate.behaviors import *

def create_root():
    """
    Method to structure the behavior tree to monitor battery status and start rotation if battery is low
    
    The "battery_monitor" behavior tree writes the voltage of the battery on 'blackboard' of behavior tree. 
    If the battery level is lower than a threshold, the robot will execute certain behavior. Here the robot 
    rotation is used to indicate low level of the battery
    """
    ## define nodes and behaviors

    # defining root node
    root = pt.composites.Parallel(name="root",policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))    
    topics2bb = pt.composites.Sequence(name="Topics2BB",memory=True)
    priorities = pt.composites.Selector(name="Priorities",memory=False)
    battery2bb = battery_status2bb(name="Battery2BB",topic_name="/battery_voltage")
    rotate_platform = rotate(name="rotate_platform", topic_name="/cmd_vel")
    def check_battery_low_on_blackboard(blackboard): return blackboard.battery_low_warning 
    
    blackboard = pt.blackboard.Blackboard()
    blackboard.battery_low_warning = False
    
    battery_emergency = pt.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child = rotate_platform
    )

    idle = pt.behaviours.Running(name="Idle")
    

    root.add_child(topics2bb)
    root.add_child(priorities)
    topics2bb.add_child(battery2bb)
    priorities.add_child(battery_emergency)
    priorities.add_child(idle)
    
    return root

def main():
    """
    Main function initiates behavior tree construction
    """
    rclpy.init(args=None)

    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root,unicode_tree_debug=True)

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
