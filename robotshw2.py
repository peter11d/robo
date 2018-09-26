#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from math import radians, copysign, sqrt, pow, pi, atan2
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)

def shutdown():
    vel_pub.publish(Twist())
    rospy.sleep(1)
    
def scan_callback(msg):
    g_range_ahead = min(msg.ranges)
    return(g_range_ahead)
    
def get_odom():
    # Get the current transform between the odom and base frames
    try:
        (trans, rot)  = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
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
goal = Point()
goal.x = 10
goal.y = 0

r = rospy.Rate(20)
linear_speed = rospy.get_param("~linear_speed", 0.2)        # meters per second
angular_speed = rospy.get_param("~angular_speed", 0.7)      # radians per second
angular_tolerance = rospy.get_param("~angular_tolerance", radians(1)) # degrees to radians



def move(dist):
     # Get the starting position values     
    (position, rotation) = get_odom()
                
    x_start = position.x
    y_start = position.y
    
    # Keep track of the distance traveled
    distance_moved = 0
    
    cmd = Twist()
    cmd.linear.x = linear_speed
    
    if dist < 0:
        cmd.linear.x *= -1
    
    while abs(distance_moved) < abs(dist) and not rospy.is_shutdown():
        # Publish the Twist message and sleep 1 cycle   
        vel_pub.publish(cmd)
        r.sleep()
        
        # Get the current position
        (position, rotation) = get_odom()
        
        # Compute the Euclidean distance from the start
        distance_moved = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
        
    cmd = Twist()
    vel_pub.publish(cmd)
    rospy.sleep(1)
    
def rotate(angle):
    print("ROTATE!!!!!")
    # Get the starting position values     
    (position, rotation) = get_odom()
    
    # Track the last angle measured
    last_angle = rotation
    
    # Track how far we have turned
    turn_angle = 0
    
    cmd = Twist()
    cmd.angular.z = angular_speed
    
    if angle < 0:
        cmd.angular.z *= -1
    
    while abs(turn_angle + angular_tolerance) < abs(radians(angle)) and not rospy.is_shutdown():
        # Publish the Twist message and sleep 1 cycle   
        vel_pub.publish(cmd)
        r.sleep()
        
        # Get the current rotation
        (position, rotation) = get_odom()
        
        # Compute the amount of rotation since the last lopp
        delta_angle = normalize_angle(rotation - last_angle)
        
        turn_angle += delta_angle
        last_angle = rotation
       
    cmd = Twist()
    vel_pub.publish(cmd)
    rospy.sleep(1)


def follow_m_line():
    # Get the starting position and rotation values     
    (position, rotation) = get_odom()

    print("rotation: {}".format(rotation))
    print(position)
    # Goal is (10, 0, 0)
    y = goal.y - position.y
    x = goal.x - position.x
    print("x: {} \t y: {}".format(x, y))
    print("atan: {}".format(atan2(goal.y - position.y, goal.x - position.x)))
    #rotate(angle)
    angle = -rotation + atan2(y, x)
    print(angle)
    rotate(angle)
    rospy.sleep(5)
    move(1)
    
    

follow_m_line()
rospy.sleep(2)



shutdown()