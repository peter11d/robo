#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)

def shutdown():
    vel_pub.publish(Twist())
    rospy.sleep(1)
    
def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)
    print(g_range_ahead)

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)



rospy.init_node('bug', anonymous=False)

state_change_time = rospy.Time.now()
rate = rospy.Rate(10)
linear_speed = 0.2

def move_x(dist):
    cmd = Twist()
    cmd.linear.x = linear_speed
    vel_pub.publish(cmd)
    rospy.sleep(1)
    
def move_y(dist):
    pass

def move_z(dist):
    pass
    

move_x(1)
move_x(1)
move_x(1)
move_x(1)
move_x(1)
move_x(1)
move_x(1)
move_x(1)
move_x(1)



#    (position, rotation) = self.get_odom()
#    x_start = position.x
#    y_start = position.y
#    
#    distance = 0

#while not rospy.is_shutdown():
#    if driving_forward:
#        if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
#            driving_forward = False
#            state_change_time = rospy.Time.now() + rospy.Duration(5)
#    else: # we're not driving_forward
#        if rospy.Time.now() > state_change_time:
#            driving_forward = True 
#            # we're done spinning, time to go forward!
#            state_change_time = rospy.Time.now() + rospy.Duration(30)
#    twist = Twist()
#    move_x(2, twist)
#    cmd_vel_pub.publish(twist)

shutdown()