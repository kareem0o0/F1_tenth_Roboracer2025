#!/usr/bin/env python3

import rclpy 
import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String , Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

''' 1 > the initial position of car known from ips sensor data  
    2 > the target here is one goal point to verfiy the pure pursuit control
    start with a one goal point (selected based on a lookahead assumed ) located on the y axis of the car 
    3 > impelement the pure pursuit formula to get the steering angle value
'''


#==========================================================================
# with change point by look ahead   
# on quallification Round 
# throttle 0.07
# look_ahead = 3.3  

#==========================================================================


#-----------------------Global variables-------------------------------- 

## car current position & orientation 
x_postition =  0.748
y_postition =  3.16
postition = np.array([x_postition , y_postition ])

car_yaw = 0.0

# Centerline Path of CDC Practice 
path_data = pd.read_csv('/home/autodrive_devkit/src/pure_pursuit_pkg/pure_pursuit_pkg/csv_paths_practice_cdc/centerline_quallification_cdc_2025.csv')

#-----------------------------------------------------------------------------------

goal_list = list(zip(
     ( path_data['x'] + x_postition ) , 
     ( -(path_data['y'] + y_postition - 0.74 )) 
    ))
goal = np.array(goal_list)
path_len = len(goal)

# pure pursuit parameter 
velocity = 0.07  
look_ahead = 1.5 
wheelbase = 0.3240 
# goal is Nx2 -> [:,0] = x , [:,1] = y

distances = np.sqrt((goal[:,0] - x_postition)**2 + (goal[:,1] - y_postition)**2)
index = np.argmin(distances)  # index of closest point

count = index   # start index  
search_len = path_len / 5
search_end = min(count + int(search_len), path_len) # to avoid being out of range 


#-----------------------call back functions-------------------------------- 
def pose_callback (ips_msg):
    global postition

    postition[0] =  ips_msg.pose.pose.position.x + 0.03        
    postition[1] =  ips_msg.pose.pose.position.y + 0.4

def yaw_callback (imu_msg) : 

    global car_yaw 

    ## Quaternion from imu data 
    qx = imu_msg.orientation.x
    qy = imu_msg.orientation.y
    qz = imu_msg.orientation.z
    qw = imu_msg.orientation.w
    ## convert to euler to get yaw 
    r = R.from_quat ([qx,qy,qz,qw])
    roll , pitch , car_yaw = r.as_euler('xyz')
    #print(f"Euler angles: roll={roll}, pitch={pitch}, yaw={car_yaw}")


#-------------------------Pure pursuit functions--------------------------------

def transformation ( xy_world_arr , point_world_arr , yaw  ) : 

    R_T = np.array([ [np.cos(yaw) , np.sin(yaw)],
                     [-np.sin(yaw), np.cos(yaw)]   ])
    
    point_car_frame = R_T @ (point_world_arr-xy_world_arr)
    return  point_car_frame

def curvature_calc (xy_car_frame) :
    x = xy_car_frame[0]
    y = xy_car_frame[1]
    curvature = (2* y) / (look_ahead*look_ahead)  ## without abs()
    return curvature

def steering_func (wh_base , gamma) :

    steering_angle = np.arctan(wh_base *gamma) 
    
    """Avoid steering more than maximum"""
    # if (steering_angle >=  0.5236):
    #    steering_angle = 1
    # elif (steering_angle <= -0.5236):
    #    steering_angle = -1

    return steering_angle




#-------------------- ROS 2 Timer_function ------------------------
def timer_func( node, st_pub , thr_pub ) : 
    global  postition , car_yaw  , count
    st = Float32()
    thr = Float32() 
    start = count   
    search_end = min(count + int(search_len), path_len) # to avoid being out of range 

    node.get_logger().info("Publishing : >_<" )
 

    check_distance = np.sqrt((goal[count:search_end ,0] - postition[0])**2 + (goal[count:search_end ,1] - postition[1])**2)
    nearest_idx = np.where(check_distance >= (look_ahead))[0]   # index of next goal point

    if len(nearest_idx) >  0 :  
        count = start + nearest_idx[0]
    else :
        count += 1  

    if ( count >= path_len  ):  ## start point faraway = no oscullation
        count = 10

    xy_cf = transformation( postition , goal[count] ,car_yaw )
    curve = curvature_calc ( xy_cf )
    steer = steering_func( wheelbase, curve ) / 0.5236
    st.data =  float(steer)

    thr_msg = velocity
    thr.data = thr_msg
    st_pub.publish(st) 
    thr_pub.publish(thr) 
        
    node.get_logger().info(f" yaw angle  : {round(car_yaw,3)} " )
    node.get_logger().info(f" steeing command value : {round(steer,3)} >_<" )
    node.get_logger().info(f" throttle command value : {thr_msg} >_<" )
    node.get_logger().info(f" Lookahead  : {look_ahead} >_<" )
    node.get_logger().info(f" index   : {count} >_<" )

#------------------------- Main ------------------------
def main (args=None):    
    

    #   Node creation

    rclpy.init(args=args)    
    my_node = rclpy.create_node('control_node')

    #  subscribers

    car_pose = my_node.create_subscription(Odometry , '/odom_rf2o' , pose_callback , 10)
    ips_msg = Odometry()    
    imu_sub = my_node.create_subscription(Imu , '/autodrive/roboracer_1/imu' , yaw_callback , 10)
    imu_msg = Imu()
    steer_pub = my_node.create_publisher( Float32 , "/autodrive/roboracer_1/steering_command" , 10 )  
    st_msg = Float32()
    throttle_pub = my_node.create_publisher( Float32 , "/autodrive/roboracer_1/throttle_command" , 10 )
    th_msg = Float32()


    timer = my_node.create_timer ( 0.01 , lambda:timer_func(my_node,steer_pub,throttle_pub) )   #  100 hz  
    rclpy.spin(my_node)
    my_node.destroy_timer(timer)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__" :    # the entry of the code     


    main()   