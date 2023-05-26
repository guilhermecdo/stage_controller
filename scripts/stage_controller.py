#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import numpy as np
from tf.transformations import euler_from_quaternion
import rrt

laser = LaserScan()
odometry = Odometry()

def odometry_callback(data):
	global odometry
	odometry = data


def laser_callback(data):
	global laser
	laser = data

def euclidDistance(robot_pose:list,target:list):
    return np.sqrt((robot_pose[0]-target[0])**2 + (robot_pose[1]-target[1])**2)

def robotSetVelocity(velocitys:list):
    velocity.linear.x = velocitys[0]
    velocity.angular.z = velocitys[1]
    pub.publish(velocity)

def robotAngle():
    _, _, yaw = euler_from_quaternion([odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z,odometry.pose.pose.orientation.w])
    
    yaw_graus = np.degrees(yaw)

    if yaw < 0:
        yaw_graus=yaw_graus + 359
  
    return float("{:.2f}".format(yaw_graus))

def obstacle():
    while min(laser.ranges[1:700]) <= 0.5:
        robotSetVelocity([0.0,-0.1])

def goTo(waypoint:list):
    dx = waypoint[0] - odometry.pose.pose.position.x
    dy = waypoint[1] - odometry.pose.pose.position.y

    angle=np.arctan2(dy, dx)
    angle_degrees=np.degrees(angle)

    if angle<0:
        angle_degrees=angle_degrees+359

    while euclidDistance([float("{:.2f}".format(odometry.pose.pose.position.x)),float("{:.2f}".format(odometry.pose.pose.position.y))],[float("{:.2f}".format(waypoint[0])),float("{:.2f}".format(waypoint[1]))])>0.2:

        if float("{:.2f}".format(np.sqrt((robotAngle()-angle_degrees)**2))) > 1 and min(laser.ranges[1:700]) > 0.5:
            if angle_degrees > robotAngle():
                robotSetVelocity([0.0,0.1])  
            else:
                robotSetVelocity([0.0,-0.1])

        elif float("{:.2f}".format(np.sqrt((robotAngle()-angle_degrees)**2))) <= 1 and min(laser.ranges[1:700]) > 0.5:
            robotSetVelocity([0.2,0])

        elif min(laser.ranges[1:700]) <= 0.5:
            robotSetVelocity([0.0,0.0])
            break
    
if __name__ == "__main__":

    rospy.init_node("stage_controller_node", anonymous=False)
    rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)
    rospy.Subscriber("/base_scan", LaserScan, laser_callback)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
    velocity=Twist()

    target=[-2.0,3.0]
    min_distance=0.2
    
    robot_pose=[odometry.pose.pose.position.x,odometry.pose.pose.position.y]
    path=rrt.rrt(robot_pose,target)

    while not path:
        path=rrt.rrt(robot_pose,target)
    print("New Path",path)
    
    while not rospy.is_shutdown():
        
        robot_pose=[odometry.pose.pose.position.x,odometry.pose.pose.position.y]
        
        if euclidDistance(robot_pose,target) > min_distance:
            if len(laser.ranges)>0: 
                if min(laser.ranges[1:700]) > 0.5:
                    for waypoints in path:
                        goTo(waypoints)
                else :
                    obstacle()
                    path.clear()
                    while not path:
                        path=rrt.rrt(robot_pose,target)
                    print("New Path",path)                 
        else:
            robotSetVelocity([0,0])
            print('cheguei no Objetivo')