#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from math import radians, copysign, sqrt, pow, pi
from rbx1_nav.transform_utils import quat_to_angle

vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)

def shutdown():
    vel_pub.publish(Twist())
    rospy.sleep(1)
    
def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)
    print(g_range_ahead)
    
def get_odom(self):
    # Get the current transform between the odom and base frames
    try:
        (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*trans), quat_to_angle(Quaternion(*rot)))

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.init_node('bug', anonymous=False)
state_change_time = rospy.Time.now()

base_frame = rospy.get_param('~base_frame', '/base_link')
odom_frame = rospy.get_param('~odom_frame', '/odom')
tf_listener = tf.TransformListener()
odom_frame = '/odom'

try:
    tf_listener.waitForTransform(odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
    base_frame = '/base_footprint'
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    try:
        tf_listener.waitForTransform(odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
        base_frame = '/base_link'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
        rospy.signal_shutdown("tf Exception")  

position = Point()

r = rospy.Rate(20)
linear_speed = 0.2



def move_x(move_distance):
     # Get the starting position values     
    (position, rotation) = get_odom()
                
    x_start = position.x
    y_start = position.y
    
    # Keep track of the distance traveled
    distance = 0
    
    cmd = Twist()
    cmd.linear.x = linear_speed
    
    while distance < move_distance and not rospy.is_shutdown():
        # Publish the Twist message and sleep 1 cycle   
        vel_pub.publish(cmd)
        r.sleep()
        
        # Get the current position
        (position, rotation) = get_odom()
        
        # Compute the Euclidean distance from the start
        distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
        print(distance)
        
    cmd = Twist()
    vel_pub.publish(cmd)
    rospy.sleep(1)
    
def move_y(dist):
    pass

def move_z(dist):
    pass
    

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