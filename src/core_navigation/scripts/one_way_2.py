#!/usr/bin/env python

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep
import sys
import os
import time

points=[[30,0]]
goal = points[0]
counter = 0
prev = 0

# rostopic pub /husky_velocity_controller/cmd_vel geometry_msgs/Twist "linear:
#         x: 0.5
#         y: 0.0
#         z: 0.0
# angular:
#         x: 0.0
#         y: 0.0
#         z: 0.0" -r 10

############################################################################
#  Title: Husky Control In Gazebo
#  Author: Brian Bingham
#  Date: 9-6-2016
#  Availability: https://wiki.nps.edu/display/RC/Husky+Control+in+Gazebo
#  Related Source: https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/blob/master/gazebo-tutorial/scripts/nre_simhuskycontrol.py
############################################################################

def huskyOdomCallback(message,cargs):
    # Implementation of proportional position control
    # For comparison to Simulink implementation
    print("data revieced")
    global points, goal, counter, prev
    pub,msg = cargs
    counter = counter + 1

    # Tunable parameters
    wgain = 15.0 # Gain for the angular velocity [rad/s / rad]
    vconst = 2.0 # Linear velocity when far away [m/s]
    distThresh = 0.3 # Distance treshold [m]

    # Generate a simplified pose
    pos = message.pose.pose
    quat = pos.orientation
    # From quaternion to Euler
    angles = tf.transformations.euler_from_quaternion((quat.x,quat.y, quat.z,quat.w))

    theta = angles[2]
    pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta

    # Proportional Controller
    v = 0 # default linear velocity
    w = 0 # default angluar velocity
    distance = sqrt((pose[0]-goal[0])**2+(pose[1]-goal[1])**2)
    if (distance > distThresh):
        v = vconst
        w = 0
    else:
        if(len(points) > 1):
            print("")
            points.pop(0)
            print(points)
            goal = points[0]
        else:
            print("")
            os.system("rosservice call gazebo/get_model_state '{model_name: /}'")
            os.system('[ -z "$(rosnode list | grep simhusky)" ] && echo "Empty" || rosnode kill $(rosnode list | grep simhusky)')
            sys.exit()

    first = int(str(int(distance))[:2])
    if(first % 10 == 0 and first != prev):
        msg.linear.x = 0
        pub.publish(msg)
        rospy.sleep(4)
        stop = False
        print("STOP!")
        prev = first
    else:
        prev = first

    # Publish
    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)

    # Reporting
    print('huskyOdomCallback: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))

def main():
    # Initialize our node
    rospy.init_node('nre_simhuskycontrol',anonymous=True)

    # Setup publisher
    cmdmsg = geometry_msgs.msg.Twist()
    cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)

    rospy.Subscriber('/husky_velocity_controller/odom',nav_msgs.msg.Odometry,huskyOdomCallback, (cmdpub,cmdmsg))
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
