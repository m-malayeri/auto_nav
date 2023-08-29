#!/usr/bin/env python3

import py_trees as pt
import py_trees_ros as ptr
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan,Joy
#from sensor_msgs.msg import Joy
import numpy as np
import pandas as pd
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autonomous_map_navigate.utilities import *
import math




class rotate(pt.behaviour.Behaviour):

    """
    Rotates the robot about z-axis 
    """

    def __init__(self, name="rotate platform", topic_name="/cmd_vel", direction=1, max_ang_vel=1.0):

        # self.logger.info("[ROTATE] initialising rotate behavior")

        # Set up topic name to publish rotation commands
        self.topic_name = topic_name

        # Set up Maximum allowable rotational velocity
        self.max_ang_vel = max_ang_vel # units: rad/sec

        # Set up direction of rotation
        self.direction = direction

        # Execution checker
        self.sent_goal = False

        # become a behaviour
        super(rotate, self).__init__(name)

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behavior")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where, if **direction** is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[ROTATE] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # Send the rotation command to self.topic_name in this method using the message type Twist()
        
        msg = Twist()
        msg.angular.z = 0.5
        """if (self.direction == +1):
        	msg.angular.z = 0.5
        elif (self.direction == -1):
        	msg.angular.z = - 0.5
        """
        self.cmd_vel_pub.publish(msg)
        
        ## YOUR CODE HERE ##
        
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)

class stop_motion(pt.behaviour.Behaviour):

    """
    Stops the robot when it is controlled using joystick or by cmd_vel command
    """

    def __init__(self, 
                 name: str="stop platform", 
                 topic_name1: str="/cmd_vel", 
                 topic_name2: str="/joy"):
        super(stop_motion, self).__init__(name)
        # Set up topic name to publish rotation commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2
        
        
       

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[STOP MOTION] setting up stop motion behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_topic = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[STOP] update: updating stop behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        """
        Send the zero rotation command to self.cmd_vel_topic
        """
        
        ## YOUR CODE HERE ##
       # self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        msg1 = Twist()
        msg1.linear.x = 0.0
        msg1.linear.x = 0.0
        # msg1.angular.z = 0.0
        self.cmd_vel_topic.publish(msg1)

           
        



        """
        sending self.joy_pub in this method. The frame_id for Joy() message is
        "/dev/input/js0". It is similar to just pressing the deadman button on the joystick.
        Nothing to implement here.
        """
        ## Uncomment the following lines to publish Joy() message when running on the robot ##
        # joyMessage = Joy()
        # joyMessage.header.frame_id = "/dev/input/js0"
        # joyMessage.axes = [0., 0., 0., 0., 0., 0.]
        # joyMessage.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        # self.joy_pub.publish(joyMessage)   
             

        return pt.common.Status.SUCCESS
        # return pt.common.Status.RUNNING


    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.

       
                    
        self.cmd_vel_topic.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)
    
    

class battery_status2bb(ptr.subscribers.ToBlackboard):

    """
    Checking battery status
    """
    def __init__(self, 
                 topic_name: str="/battery_voltage",
                 name: str=pt.common.Name.AUTO_GENERATED, 
                 threshold: float=30.0):
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=Float32,
                        blackboard_variables={'battery': 'data'},
                        initialise_variables={'battery': 100.0},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        qos_profile=ptr.utilities.qos_profile_unlatched()
                        )
        self.blackboard.register_key(
            key='battery_low_warning',
            access=pt.common.Access.WRITE
        ) 
        self.blackboard.battery_low_warning = False   # decision making
        self.threshold = threshold

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        """
        self.logger.info('[BATTERY] update: running batter_status2bb update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(battery_status2bb, self).update()
        
        """
        check battery voltage level stored in self.blackboard.battery. By comparing with threshold value, update the value of 
        self.blackboad.battery_low_warning
        """

        ## YOUR CODE HERE ##
        
        
        if self.blackboard.battery < self.threshold:
            self.blackboard.battery_low_warning = True
        else:
            self.blackboard.battery_low_warning = False
                
        return pt.common.Status.SUCCESS

class laser_scan_2bb(ptr.subscribers.ToBlackboard):

    """
    Checking laser_scan to avoid possible collison
    """
    def __init__(self, 
                 topic_name: str="/scan",
                 name: str=pt.common.Name.AUTO_GENERATED, 
                 safe_range: float=1.5):
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=LaserScan,
                        blackboard_variables={'laser_scan':'ranges'},
                        #initialise_variables={'battery': []},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        # qos_profile=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                        qos_profile=QoSProfile(
                                    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                    depth=10
                                )
                        )
        #self.blackboard=pt.blackboard.Blackboard()
        self.blackboard.register_key(
            key='collison_warning',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='point_at_min_dist',
            access=pt.common.Access.WRITE
        )
       
        self.blackboard.register_key(
            key='counter',
            access=pt.common.Access.WRITE
        )

        # self.blackboard.register_key(
        #     key='wall_detect_warning',
        #     access=pt.common.Access.WRITE
        # )

        # self.blackboard.register_key(
        #     key='wall_slope',
        #     access=pt.common.Access.WRITE
        # )

        # self.blackboard.register_key(
        #     key='perp_distance',
        #     access=pt.common.Access.WRITE
        # # )

        # self.blackboard.register_key(
        #     key='wall_data',
        #     access=pt.common.Access.WRITE
        # )

        # self.blackboard.register_key(
        #     key='ransac_warn',
        #     access=pt.common.Access.WRITE
        # )

        self.blackboard.counter = 0
        self.blackboard.collison_warning = False   # decision making
        self.safe_min_range = safe_range
        self.blackboard.point_at_min_dist = 0.0
        # self.blackboard.wall_detect_warning = False
        # self.blackboard.ransac_warn = False
        # self.blackboard.wall_slope = 0.0
        # self.blackboard.perp_distance = 0.0

    def update(self):
        """
        Primary function of the behavior is implemented in this method
        Call the parent to write the raw data to the blackboard and then check against the
        threshold to set the warning if the robot is close to any obstacle.
        """
        self.logger.info("[LASER SCAN] update: running laser_scan_2bb update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(laser_scan_2bb, self).update()

        """
        Based on the closeness of laser scan points (any of them) to robot, update the value of self.blackboard.collison_warning.
        Assign the minimum value of laser scan to self.blackboard.point_at_min_dist. 
        Note: The min() function can be used to find the minimum value in a list.
        """

        ## YOUR CODE HERE ##   
        self.blackboard.counter += 1

        if self.blackboard.counter > 500:
            laser_data = np.array(self.blackboard.laser_scan)

            laser_data[laser_data <= 0.05] = 1.0
            laser_data = np.nan_to_num(laser_data, nan=40)

            # data_check = laser_data
            # data_check[np.isinf(data_check)]=50
            # data_check = data_check.tolist()


            laser_data[np.isinf(laser_data)]=50
            # laser_data = laser_data.tolist()
            # print(laser_data)
            # self.blackboard.laser_data = laser_data

            # print(min(self.black   board.laser_scan))
            # self.blackboard=pt.blackboard.Blackboard()
        
            self.blackboard.point_at_min_dist = min(laser_data)

            if self.blackboard.point_at_min_dist < self.safe_min_range:

                self.blackboard.collison_warning = True
                # self.blackboard.ransac_warn = True
                # self.blackboard.wall_detect_warning = True

                # df = pd.DataFrame({"distance": laser_data, "angle": range(0, 150)})

                # # Filter points based on distance
                # scan_dist_thresh = 8000
                # df = df.drop(df[df.distance > scan_dist_thresh].index)
                # data_points = np.array(df[['distance', 'angle']])
                # array = np_polar2rect(reduction_filter(data_points, sigma=40, k=6))


                # # points = [point for point in array]
            
                # res = RANSAC_get_line_params(points=array, dist_thresh=10, iterations=10, thresh_count=10)

                # self.blackboard.wall_data = res
                # self.blackboard.wall_slope = res[0][1]
                # self.blackboard.perp_distance = res[0][0]

                # self.blackboard.check_warning = True

            else:
                self.blackboard.collison_warning = False
                # self.blackboard.wall_detect_warning = False

            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING            
        
class position_wrt_odom(ptr.subscribers.ToBlackboard):

    """
    Getting the Position of Robot with respect to Odometry
    """
    def __init__(self, 
                 topic_name: str="/odom",
                 name: str=pt.common.Name.AUTO_GENERATED):
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=Odometry,
                        blackboard_variables={'odom_data':'pose.pose.position','odom_orientation':'pose.pose.orientation'},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        # qos_profile=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                        qos_profile=QoSProfile(
                                    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                    depth=10
                                )
                        )
        
        # self.blackboard.odom_flag = False
        
        self.blackboard.register_key(
            key='robot_center',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='robot_end',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='robot_slope',
            access=pt.common.Access.WRITE
        )

        
       
       
    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard 
        """
        self.logger.info("[ODOM SCAN] update: running position_wrt_odom update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(position_wrt_odom, self).update()

        a=0
        odom_info = self.blackboard.odom_data
        odom_orient = self.blackboard.odom_orientation
        

        robot_position = [odom_info.x, odom_info.y]
        robot_orient = [odom_orient.z, odom_orient.w]
        theta = 2*math.atan2(robot_orient[0],robot_orient[1])
        d=0.5
        robot_end = [robot_position[0]+d*math.cos(theta),robot_position[1]+d*math.sin(theta)]
                
        # self.blackboard.robot_orientation = robot_orient

        lin_obj = Line(robot_position, robot_end)
        robot_slope = lin_obj.equation()

        robot_position = [odom_info.x, odom_info.y, theta]
        self.blackboard.robot_center = robot_position
        
        self.blackboard.robot_end = robot_end

        self.blackboard.robot_slope = robot_slope

    

        a=a+1
        
        if(a!=0):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING  



class wall_get_data(ptr.subscribers.ToBlackboard):

    def __init__(self, 
                 topic_name: str="/scan",
                 name: str=pt.common.Name.AUTO_GENERATED):
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=LaserScan,
                        blackboard_variables={'laser_scan':'ranges'},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        # qos_profile=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                        qos_profile=QoSProfile(
                                    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                    depth=10
                                )
                        )
        
        
        self.blackboard.register_key(
            key='wall_slope',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='laser_data',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='distance',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='wall_warn',
            access=pt.common.Access.WRITE
        )

        self.blackboard.wall_warn = False
        self.blackboard.wall_slope = 0.0
        self.blackboard.distance = 0.0

             


    def update(self):   
        data = np.array(self.blackboard.laser_scan)
        data[np.isinf(data)]=50
        data = data.tolist()
        # filter_data

        self.blackboard.laser_data = data

        
        # df = pd.DataFrame({"distance": data, "angle": range(0, 150)})
        
        # filtered_data = process_data(range_data= data, max_angle= 1.5700000524520874, min_angle= -1.5700000524520874, max_range= 5.599999904632568, min_range= 0.05000000074505806, sigma= 40.0 , rf_max_pts= 6, reduce_bool= True)
        # filtered_data = filtered_data.tolist()
    


        df = polardatadf(data)

        # Filter points based on distance
        scan_dist_thresh = 8000
        df = df.drop(df[df.distance > scan_dist_thresh].index)
        data_points = np.array(df[['distance', 'angles']])
        filtered_data = np_polar2rect(reduction_filter(data_points, sigma=40, k=6))

        # print(filtered_data)


        # points = [point for point in array]
    
        res = RANSAC_get_line_params(points=filtered_data, dist_thresh=0.2, iterations=10, thresh_count=8)

        if res == []:
            print("aligned")
            self.blackboard.wall_slope = 1000
            self.blackboard.distance = "insafe"
            self.blackboard.wall_warn = False

        else:
            print(res)
            self.blackboard.wall_slope = res[0][1]
            self.blackboard.distance = res[0][0]
            self.blackboard.wall_warn = True

        # res = online_get_line_params(points_array= filtered_data, e=0.45, incr=0.01, max_dist=0.1, k=5)

        # self.blackboard.wall_slope = res

        
                        

        return pt.common.Status.SUCCESS
    



class rotate_wrt_angle(pt.behaviour.Behaviour):

    """
    Rotates the robot about z-axis 
    """

    def __init__(self, name="rotate angle", topic_name="/cmd_vel", direction=1, max_ang_vel=1.0):

        self.topic_name = topic_name

        self.max_ang_vel = max_ang_vel # units: rad/sec

        # Set up direction of rotation
        self.direction = direction

        # Execution checker
        self.sent_goal = False

        # become a behaviour
        super(rotate_wrt_angle, self).__init__(name)

        self.blackboard = pt.blackboard.Blackboard()
        # self.blackboard.storage = {'m1': 'wall_slope'}

        # self.blackboard.register_key(
        #     key='wall_slope',
        #     access=pt.common.Access.WRITE
        # )
        

        # self.blackboard.register_key(
        #     key='check_warning',
        #     access=pt.common.Access.WRITE
        # )
        # self.blackboard.set('check_warning')

        self.slope = self.blackboard.get('wall_slope')
        self.m1 = abs(self.slope)
        # self.m1 = self.blackboard.wall_slope
        # print(self.blackboard.storage['m1'])
        # self.angle = float(self.blackboard.storage['m1'])
        # self.angle = 0.1
        # print(self.angle)

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[rotate_wrt_angle] setting up rotate behavior")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where, if **direction** is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[rotate_wrt_angle] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        # print(self.blackboard.storage['m1'])
        # self.m1 = self.blackboard.wall_slope
        self.slope = self.blackboard.get('wall_slope')
        print(self.slope)
        self.m1 = abs(self.slope)
        print(self.m1)

        # Send the rotation command to self.topic_name in this method using the message type Twist()
        

        msg = Twist()
    
        # if self.angle >= 0.05:
        #     msg.linear.x = 0.0
        #     msg.linear.y= 0.0
        #     msg.angular.z = -1.0

        #     self.cmd_vel_pub.publish(msg)
        #     # self.angle = self.angle - 1
        #     self.m1 = self.blackboard.get('wall_slope')
        #     self.angle = abs(self.m1)

        #     return pt.common.Status.RUNNING
            
        # else:

        #     return pt.common.Status.SUCCESS
        

        if self.m1 == 1000 or self.m1 >15:
            return pt.common.Status.SUCCESS
        
        else:
            msg.linear.x = 0.0
            msg.linear.y= 0.0
            msg.angular.z = 0.65

            self.cmd_vel_pub.publish(msg)
            # self.angle = self.angle - 1
            self.slope = self.blackboard.get('wall_slope')
            self.m1 = abs(self.slope)

            return pt.common.Status.RUNNING


        

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)

        # self.blackboard.set('check_warning',False)
        self.blackboard.set('wall_warn',False)

        self.slope = self.blackboard.get('wall_slope')
        self.m1 = abs(self.slope)


        # self.angle = abs(self.blackboard.wall_slope)

        return pt.common.Status.SUCCESS


class move_wrt_distance(pt.behaviour.Behaviour):

    """
    Rotates the robot about z-axis 
    """

    def __init__(self, name="moving_wrt_distance", topic_name="/cmd_vel", direction=1, max_ang_vel=0.75):

        self.topic_name = topic_name

        self.max_ang_vel = max_ang_vel # units: rad/sec

        self.direction = direction

        # Execution checker
        self.sent_goal = False

        # become a behaviour
        super(move_wrt_distance, self).__init__(name)

        self.blackboard = pt.blackboard.Blackboard()

        self.d = self.blackboard.get('distance')
        

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[moving_wrt_distance] setting up rotate behavior")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where, if **direction** is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[moving_wrt_distance] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.d = self.blackboard.get('distance')

        msg = Twist()
        

        if self.d < 0.6:
            return pt.common.Status.SUCCESS
        
        else:
            msg.linear.x = self.max_ang_vel
            msg.linear.y= 0.0

            self.cmd_vel_pub.publish(msg)
                       
            self.d = self.blackboard.get('distance')

            return pt.common.Status.RUNNING


        

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)

        self.d = self.blackboard.get('distance')


        # self.angle = abs(self.blackboard.wall_slope)

        return pt.common.Status.SUCCESS

