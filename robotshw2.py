#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from math import radians, degrees, copysign, sqrt, pow, pi, atan2
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

class bug2():
    def __init__(self):
        rospy.init_node('bug', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)

        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        

        self.rate = 20
        self.r = rospy.Rate(self.rate)

        position = Point()
        self.goal = Point()
        self.goal.x = 10
        self.goal.y = 0

        self.linear_speed = rospy.get_param("~linear_speed", 0.3)        # meters per second
        self.angular_speed = rospy.get_param("~angular_speed", 0.7)      # radians per second
        self.angular_tolerance = rospy.get_param("~angular_tolerance", radians(1)) # degrees to radians

        state_change_time = rospy.Time.now()

        self.tf_listener = tf.TransformListener()
        rospy.sleep(2)

        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        
        self.odom_frame = '/odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

        #self.rotate(-110)
        #self.move(2.5)
        self.follow_m_line()
        rospy.sleep(2)
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.vel_pub.publish(Twist())
        rospy.sleep(1)
    
    def scan_callback(self, msg):
        self.range_left = msg.ranges[-1]
        self.range_center = msg.ranges[len(msg.ranges)//2]
        self.range_right = msg.ranges[0]
        print(self.range_center)
    
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))


    def move(self, dist):
        # Get the starting position values     
        (position, rotation) = self.get_odom()
                
        x_start = position.x
        y_start = position.y
    
        # Keep track of the distance traveled
        distance_moved = 0
    
        cmd = Twist()
        cmd.linear.x = self.linear_speed
    
        if dist < 0:
            cmd.linear.x *= -1
    
        while abs(distance_moved) < abs(dist) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle   
            self.vel_pub.publish(cmd)
            self.r.sleep()
        
            # Get the current position
            (position, rotation) = self.get_odom()
        
            # Compute the Euclidean distance from the start
            distance_moved = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
        
        cmd = Twist()
        self.vel_pub.publish(cmd)
    
    def rotate(self, angle):
        # Get the starting position values     
        (position, rotation) = self.get_odom()
    
        # Track the last angle measured
        last_angle = rotation
    
        # Track how far we have turned
        turn_angle = 0
    
        cmd = Twist()
        cmd.angular.z = self.angular_speed
    
        if angle < 0:
            cmd.angular.z *= -1
    
    
        while abs(turn_angle + self.angular_tolerance) < abs(radians(angle)) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle   
            #print("WHILE")
            self.vel_pub.publish(cmd)
            self.r.sleep()
        
            # Get the current rotation
            (position, rotation) = self.get_odom()
        
            # Compute the amount of rotation since the last lopp
            delta_angle = normalize_angle(rotation - last_angle)
        
            turn_angle += delta_angle
            last_angle = rotation
       
        cmd = Twist()
        self.vel_pub.publish(cmd)


    def follow_m_line(self):
        # Get the starting position and rotation values
        while self.range_center > 0.7 and not rospy.is_shutdown():
            (position, rotation) = self.get_odom()

            # Goal is (10, 0, 0)
            y = self.goal.y - position.y
            x = self.goal.x - position.x
            angle = -rotation + atan2(y, x)

            self.rotate(degrees(angle))
    
            self.move(self.linear_speed / self.rate)
        
        print("ENCOUNTERED")            
                
    
if __name__ == '__main__':
    bug2()