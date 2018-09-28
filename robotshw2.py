#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from math import radians, degrees, copysign, sqrt, pow, pi, atan2, isnan
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

class bug2():
    
    def __init__(self):
        rospy.init_node('bug', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        self.vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.rate = 20
        self.r = rospy.Rate(self.rate)
        
        self.linear_speed = rospy.get_param("~linear_speed", 0.4)        # meters per second
        self.linear_tolerance = rospy.get_param("~linear_speed", 0.05) 
        self.angular_speed = rospy.get_param("~angular_speed", 0.45)      # radians per second
        self.angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians
        self.unit_distance = 2 * self.linear_tolerance
        self.unit_rotation = 15 * self.angular_speed

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

        self.follow_m_line()
        self.circumnavigate(1)
        rospy.sleep(2)


    def move(self, dist=None):
        if dist is None:
            dist = self.unit_distance
        
        (position, rotation) = self.get_odom()
                
        x_start = position.x
        y_start = position.y
    
        # Keep track of the distance traveled
        distance_moved = 0
    
        cmd = Twist()
        cmd.linear.x = (self.linear_speed if dist >= 0 else -self.linear_speed)
    
        while abs(distance_moved) < abs(dist) and not rospy.is_shutdown():
            self.vel_pub.publish(cmd)

            (position, rotation) = self.get_odom()
        
            # Compute the Euclidean distance from the start
            distance_moved = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
        
        cmd = Twist()
        self.vel_pub.publish(cmd)
        self.r.sleep()

        
    def rotate(self, angle=None):
        if angle is None:
            angle = self.unit_rotation
        
        (position, initial_rotation) = self.get_odom()
        (position, rotation) = self.get_odom()
    
        cmd = Twist()
        cmd.angular.z = (self.angular_speed if angle >= 0 else -self.angular_speed)
    
        while abs(normalize_angle(rotation - initial_rotation)) + self.angular_tolerance < abs(radians(angle)) and not rospy.is_shutdown():
            self.vel_pub.publish(cmd)
        
            (position, rotation) = self.get_odom()
        
        cmd = Twist()
        self.vel_pub.publish(cmd)
        self.r.sleep()


    def follow_m_line(self):
        print("following m_line")
        (position, rotation) = self.get_odom()
        print(position)
        print(rotation)
        while self.range_center > 0.75 and not rospy.is_shutdown():
            (position, rotation) = self.get_odom()

            y = -position.y
            x = 10 - position.x
            angle = -rotation + atan2(y, x)
            if abs(angle) > abs(3 * self.angular_tolerance):
                self.rotate(degrees(angle))
                rospy.sleep(.1)
                

            if self.at_goal():
                print("AT GOAL MOFOS!!!")
            #if abs(rotation) > abs(self.angular_tolerance):
            #    self.rotate(degrees(-0.25 * rotation))
    
            self.move()
            rospy.sleep(.1)
        print("Object encountered")
        
        
    def side_dist_helper(self, direction):
        return (self.range_right if direction == 1 else self.range_left)

    def at_goal(self):
        (position, rotation) = self.get_odom()
        return (10 - 3 * self.linear_tolerance < position.x < 10 + 3 * self.linear_tolerance) and (3 * -self.linear_tolerance < position.y < 3 * self.linear_tolerance)

    
    def on_mline(self):
        (position, rotation) = self.get_odom()
        return (abs(position.y) < self.linear_tolerance)
    
    def circumnavigate(self, direction=1):
        print("Starting circumnavigate!")
        rospy.sleep(0.5)
        
        (position, rotation) = self.get_odom()
        hit_point = position
        #object_distance = self.min_dist
        
        side_dist = self.side_dist_helper(direction)
               
        target_side_dist = .9 #sqrt(2) * object_distance + 2 * self.linear_tolerance

        while not (self.on_mline() and position.x - self.linear_tolerance > hit_point.x) and not rospy.is_shutdown():
            print("circumnavigating like a boss")
            (position, rotation) = self.get_odom()

            side_dist = self.side_dist_helper(direction)
                        
            
            if isnan(side_dist):
                print("cant see object")
                self.move(target_side_dist * .7)
                while isnan(side_dist):
                    self.rotate(-direction * self.unit_rotation)
                    rospy.sleep(0.1)
                    side_dist = self.side_dist_helper(direction)
                    
                
            else:
                while self.range_center < .75 or side_dist < target_side_dist:
                    print("rotating to avoid object")
                    self.rotate(direction * self.unit_rotation)
                    rospy.sleep(0.1)
                    side_dist = self.side_dist_helper(direction)
    
                while side_dist > (target_side_dist + 3 * self.linear_tolerance):
                    print("rotating towards object")
                    self.rotate(-direction * self.unit_rotation)
                    rospy.sleep(0.1)
                    side_dist = self.side_dist_helper(direction)
                    
            self.move()

            
            # Circumnavigate other way if at same point
            '''distance = sqrt(pow((position.x - hit_point.x), 2) + pow((position.y - hit_point.y), 2))
            if distance < self.linear_tolerance:
                    if direction != 1:
                        print("FAIL HERE")
                    self.circumnavigate(-1)
                    break'''
            
        print("Circumnativation complete!")
        print(position)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.vel_pub.publish(Twist())
        rospy.sleep(1)
    
    def scan_callback(self, msg):
        self.msg_len = len(msg.ranges)
        
        self.range_left = msg.ranges[-1]
        self.range_center = msg.ranges[self.msg_len // 2]
        self.range_right = msg.ranges[0]
        
        self.min_dist = min(msg.ranges)
        self.min_indx = msg.ranges.index(self.min_dist)
        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
    
if __name__ == '__main__':
    bug2()
