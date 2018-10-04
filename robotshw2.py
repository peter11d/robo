#!/usr/bin/env python
import rospy
import tf
import os
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from math import radians, degrees, sqrt, pow, atan2, isnan
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle


class bug2():

    def __init__(self):
        rospy.init_node('bug', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.rate = 20
        self.r = rospy.Rate(self.rate)

        self.linear_speed = rospy.get_param("~linear_speed", 0.4)                  # meters per second
        self.linear_tolerance = rospy.get_param("~linear_speed", 0.05)
        self.angular_speed = rospy.get_param("~angular_speed", 0.45)               # radians per second
        self.angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians
        self.unit_distance = 2 * self.linear_tolerance
        self.unit_rotation = 7 * self.angular_speed

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

        # Top level loop
        self.follow_m_line()
        while not self.at_goal():
            self.circumnavigate()
            self.follow_m_line()
            rospy.sleep(1)
            
        print("Goal Reached")
        (position, rotation) = self.get_odom()
        print(position)
        rospy.sleep(3)
        

    def move(self, dist=None):
        # Move helper function, moves unit_distance if not distance is provided
        if dist is None:
            dist = self.unit_distance * 3

        (position, rotation) = self.get_odom()
   
        x_start = position.x
        y_start = position.y

        # Keep track of the distance traveled
        distance_moved = 0

        cmd = Twist()
        cmd.linear.x = (self.linear_speed if dist >= 0 else -self.linear_speed)

        # Keep moving at linear_speed until you travel >= inputted distance
        while abs(distance_moved) < abs(dist) and not rospy.is_shutdown() and not self.range_center < .4:
            self.vel_pub.publish(cmd)
            (position, rotation) = self.get_odom()

            # Compute the Euclidean distance from the start
            distance_moved = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))

        cmd = Twist()
        self.vel_pub.publish(cmd)
        self.r.sleep()


    def rotate(self, angle=None):
        # Rotation helper function, rotates unit_rotation if angle (in degrees) not provided
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
        # Commands robot to follow the m_line until robot is at goal or encounters obstacle
        print("Following m_line")
        (position, rotation) = self.get_odom()

        while (isnan(self.range_center) or self.range_center > 0.8) and not rospy.is_shutdown():
            (position, rotation) = self.get_odom()

            y = -position.y
            x = 10 - position.x
            angle = -rotation + atan2(y, x)
            if abs(angle) > abs(3 * self.angular_tolerance):
                self.rotate(degrees(angle))
                rospy.sleep(.1)
                
            if self.at_goal():
                return
    
            self.move()
            rospy.sleep(.1)
        print("Object encountered")
        

    def at_goal(self):
        # Helper function to check if at goal, uses 3 * linear_tolerance as tolerance
        (position, rotation) = self.get_odom()
        return (10 - 3 * self.linear_tolerance < position.x < 10 + 3 * self.linear_tolerance) and (3 * -self.linear_tolerance < position.y < 3 * self.linear_tolerance)

    
    def on_mline(self):
        # Helper function to check if robot is on m_line, within 3 * linear_tolerance
        (position, rotation) = self.get_odom()
        return (abs(position.y) < self.linear_tolerance * 3)
    
    def circumnavigate(self):
        # Handles circumnavigating obstacles until back on m_line and closer to goal
        print("Circumnavigating")
        rospy.sleep(0.5)
        
        (position, rotation) = self.get_odom()
        hit_point = position
        hit_distance_to_goal = abs(10 - position.x) 
        
        side_dist = self.range_right
               
        target_side_dist = .9
        
        # Loop counter used to ensure robot has traveled reasonably far before deciding if world is impossible
        i = 0
        
        # Circumnavigation loop: if not on m_line and closer to goal, then keep circumnavigating
        while not (self.on_mline() and abs(10 - position.x + self.linear_tolerance * 2) < hit_distance_to_goal and position.x < 10) and not rospy.is_shutdown():
            i += 1

            side_dist = self.range_right
                        
            # Handle when obstacle is no longer seen while circumnavigating
            if isnan(side_dist) and isnan(self.range_center):
                rospy.sleep(0.2)
                side_dist = self.range_right
                print("first object not seen")
                if isnan(side_dist) and isnan(self.range_center):
                    print('object not seen')
                    (position_before, rotation) = self.get_odom()
                    self.move(target_side_dist * .7)
                    (position_after, rotation) = self.get_odom()
                    if position_before.y * position_after.y < 0:
                        break
                    while isnan(side_dist):
                        self.rotate(-self.unit_rotation)
                        side_dist = self.range_right
                    
                
            else:
                # rotate towards obstacle if too far
                while side_dist > (target_side_dist + self.linear_tolerance) or isnan(side_dist) and not self.range_center < .8:
                    self.rotate(-self.unit_rotation)
                    side_dist = self.range_right

                # rotate away from obstacle if too close
                while self.range_center < .75 or self.min_right < (target_side_dist + 2 * self.linear_tolerance) or self.range_center < .8:
                    self.rotate(self.unit_rotation)
                    side_dist = self.range_right

                # rotate away from the obstacle a bit to be safe
                self.rotate(self.unit_rotation * 2.2)

                self.move()

            (position, rotation) = self.get_odom()

            # Circumnavigate other way if at same point
            distance = sqrt(pow((position.x - hit_point.x), 2) + pow((position.y - hit_point.y), 2))
            if distance < self.linear_tolerance * 5 and i >= 10:
                print(distance)
                print("Impossible to pass")
                self.shutdown()

                
    def shutdown(self):
        #Shutdown isntructions
        rospy.loginfo("Stopping the robot...")
        self.vel_pub.publish(Twist())
        rospy.sleep(1)
        os._exit(0)
    
    def scan_callback(self, msg):
        # Gets input from laser scanner and stores relevant values into variables
        self.msg_len = len(msg.ranges)

        self.range_left = msg.ranges[-1]
        self.range_center = msg.ranges[self.msg_len // 2]
        self.range_right = msg.ranges[0]

        self.min_right = min(msg.ranges[0:self.msg_len // 2])

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
