#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist

cart_pose = Pose()
pole_pose = Pose()
pole_twist = Twist()
y_angular = 0
cart_pose_x = 0

def get_cart_pose(data):
    global cart_pose, pole_twist, cart_pose_x, y_angular, pole_tip_pose_z
    ind = data.name.index('cart_pole::cart_link')
    cart_pose = data.pose[ind]

    ind_pitch = data.name.index('cart_pole::pole_link')
    pole_twist = data.twist[ind_pitch]

    cart_pose_x = cart_pose.position.x
    y_angular = pole_twist.angular.y


def commander():
    global sentry_robot_vel, cart_pose, y_angular, cart_pose_x
    yaw_angle = 0
    target_yaw_angle = 0
    target_cart_pose_x = 0
    integral_position_error = 0
    integral_yaw_error = 0
    last_error_yaw = 0
    last_error_pos = 0

    Kp_y = 18
    Ki_y = 19.34
    Kd_y = 7.75
    Kp_p = 0
    Ki_p = 6.18
    Kd_p = 5.5

    time_interval = 0.005
    pub_cart = rospy.Publisher('/cart_controller/command', Float64, queue_size = 10)

    while not rospy.is_shutdown():
        time1 = time.time()

        yaw_angle += y_angular*time_interval
        error_yaw  = target_yaw_angle - yaw_angle
        integral_yaw_error += (error_yaw + last_error_yaw)*time_interval/2
        effort_yaw = -(Kp_y*error_yaw  + 
                       Ki_y*integral_yaw_error +
                       Kd_y*(error_yaw - last_error_yaw)/time_interval)
        
        error_pos = target_cart_pose_x - cart_pose_x
        integral_position_error += (error_pos + last_error_pos)*time_interval/2
        effort_pos = -(Kp_p*error_pos  + 
                       Ki_p*integral_position_error +
                       Kd_p*(error_pos - last_error_pos)/time_interval)    
        
        effort = effort_yaw + effort_pos    
        last_error_yaw = error_yaw
        last_error_pos = error_pos
        pub_cart.publish(effort)

        time2 = time.time()
        interval = time2 - time1
        if(interval < time_interval):
            time.sleep(time_interval - interval)   

if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_cart_pose)
    commander()
    rospy.spin()
