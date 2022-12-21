#!/usr/bin/env python
# "reactive_navigation" node: subscribes laser data and publishes velocity commands
​
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
​
obstacle_distance = None
robot_stopped = None
​
​
​
def laserCallback(msg):
    global obstacle_distance
    global robot_stopped
​
    if not robot_stopped:
         rospy.loginfo("Received a LaserScan with %i samples", len(msg.ranges))

         #For simplicity, let's save the distance of the closer obstacle to the robot:
         obstacle_distance = min(msg.ranges)
         rospy.loginfo("minimum distance to obstacle: %f", obstacle_distance)
​
​
​
def reactive_navigation():
    global obstacle_distance
    global robot_stopped
​
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.init_node('reactive_navigation')
​
    rospy.Subscriber("base_scan", LaserScan, laserCallback)
​
    rate = rospy.Rate(10)
​
    #initializations:
    cmd_vel_msg=Twist()
    robot_stopped = True
    obstacle_distance = 1.0
​
    rospy.loginfo("This is where hell breaks lose! Welcome to DOOM")
​
    while not rospy.is_shutdown():
        #fill the "cmd_vel_msg" data according to some conditions (depending on laser data)
        if obstacle_distance > 0.5:
		cmd_vel_msg.linear.x = 1.0;
        	cmd_vel_msg.angular.z = 0.0;
​
		if robot_stopped:
			rospy.loginfo("Moving Forward")
			robot_stopped = False
	else:
		cmd_vel_msg.linear.x = 0.0;
        	cmd_vel_msg.angular.z = 0.0;
​
		if not robot_stopped:
			rospy.loginfo("Stopping")
			robot_stopped = True
​
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()
​
​
​
​
if __name__ == '__main__':
    try:
        reactive_navigation()
    except rospy.ROSInterruptException:
        pass
