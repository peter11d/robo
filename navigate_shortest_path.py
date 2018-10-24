#!/usr/bin/env python

import numpy as np
import rospy
import math
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, Point, Quaternion
from scipy.spatial import ConvexHull
from collections import defaultdict
import tf
import os
from math import radians, pi, degrees, sqrt, pow, atan2, isnan
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
import time

class Robot():
    
    def __init__(self):
        rospy.on_shutdown(self.shutdown)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = 20
        self.r = rospy.Rate(self.rate)

        self.linear_speed = rospy.get_param("~linear_speed", 0.6)                  # meters per second
        self.angular_speed = rospy.get_param("~angular_speed", 0.5)               # radians per second

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


    def navigate(self, points):

        for i, point_2 in enumerate(points[1:]):

            (position, current_rotation) = self.get_odom()
            point_1 = (position.x, position.y)

            # Calculate distance to move and angle to rotate
            distance = get_distance(point_1, point_2)
            ideal_degree = degrees(atan2(point_2[1] - point_1[1], point_2[0] - point_1[0]))
            current_degree = degrees(current_rotation)
            rotate_degree = ideal_degree - current_degree

            # Rotate then move
            self.rotate(rotate_degree)
            self.move(distance)
            

    def move(self, dist):
        # Move helper function

        (position, rotation) = self.get_odom()
   
        x_start = position.x
        y_start = position.y

        # Keep track of the distance traveled
        distance_moved = 0

        cmd = Twist()
        cmd.linear.x = (self.linear_speed if dist >= 0 else -self.linear_speed)
        
        # Keep moving at linear_speed until you travel >= inputted distance
        while abs(distance_moved) < abs(dist) and not rospy.is_shutdown():
            self.vel_pub.publish(cmd)
            (position, rotation) = self.get_odom()

            # Compute the Euclidean distance from the start
            distance_moved = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))

        cmd = Twist()
        self.vel_pub.publish(cmd)
        self.r.sleep()


    def rotate(self, angle):
        # Rotation helper function
        (position, initial_rotation) = self.get_odom()
        (position, rotation) = self.get_odom()
    
        cmd = Twist()
        cmd.angular.z = (self.angular_speed if angle >= 0 else -self.angular_speed)
    
        while abs(normalize_angle(rotation - initial_rotation)) < abs(radians(angle)) and not rospy.is_shutdown():
            self.vel_pub.publish(cmd)
            (position, rotation) = self.get_odom()
        
        cmd = Twist()
        self.vel_pub.publish(cmd)
        self.r.sleep()

        
    def shutdown(self):
        #Shutdown isntructions
        rospy.loginfo("Stopping the robot...")
        self.vel_pub.publish(Twist())
        rospy.sleep(1)
        os._exit(0)

        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))



class Markers(object):
    # Class that holds and draws the lines on the map

    def __init__(self):
        # Create rospy node
        rospy.init_node('register')

        # Create publisher
        self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100)

        # Sleep to start up publisher
        rospy.sleep(0.5)

        # Create a marker array and keep track of id to use
        self.markerArray = MarkerArray()
        self.current_id = 0

        
    def add_marker(self, coordinates, isShortest):
        # Create a new line_strip and add in basic info
        line_strip = Marker()
        line_strip.header.frame_id = 'map'
        line_strip.ns  = "register"
        line_strip.action = line_strip.ADD
        line_strip.type = line_strip.LINE_STRIP
        
        line_strip.pose.orientation.w = 1.0
        line_strip.scale.x = .02

        # Give the line_strip an id and increment the current id
        line_strip.id = self.current_id
        self.current_id += 1

        # Make it red
        line_strip.color.a = 1.0
        line_strip.color.r = 1.0

        # Transform the (x, y) coords into points
        for coord in coordinates:
            point = Point()
            point.x = coord[0]
            point.y = coord[1]
            point.z = (.01 if not isShortest else .02)
            line_strip.points.append(point)
        
        # Change color if for the shortest path
        if isShortest:
            line_strip.scale.x = 0.03
            line_strip.color.r = 0
            line_strip.color.b = 1.0
        
        # Add the line_strip to the MarkerArray
        self.markerArray.markers.append(line_strip)

        # Publish the markerArray
        self.publisher.publish(self.markerArray)
        rospy.sleep(0.01)
        

def load_obstacles(object_path):
	'''
	Function to load a list of obstacles.
	The obstacle txt file show points in clockwise order

	Return:
		3d list [[[1, 2], [3, 4], [5, 6]], 
						[[7, 8], [9, 10], [10, 11]]]
	'''
	obstacles = []
	obstacle = []
	with open(object_path) as f:
		numObstacles = int(f.readline())
		coordinates = int(f.readline())
		for i in range(coordinates):
			line = f.readline()
			obstacle.append(list(map(int, line.strip().split(' '))))
		for line in f:
			coordinates = list(map(int, line.strip().split(' ')))
			if len(coordinates) == 1:
				obstacles.append(obstacle)
				obstacle = []
			else:
				obstacle.append(coordinates)
	obstacles.append(obstacle)
	assert len(obstacles)==numObstacles, "number of obstacles does not match the first line"
	return obstacles

    
def load_goal(goal_path):
    #Function that loads the goal
	with open(goal_path) as f:
		line = f.readline()
		goal = list(map(int, line.strip().split(' ')))
	return goal


def get_obstacles_convexhull_points(obstacles):
    # Get all the points that make up the convexhull of the obstacles
    
    # Init arrays to keep track of coords
    all_reflection_coords = []
    all_convex_hull_coords = []

    for obstacle in obstacles:
        reflection_coords = []

        # Get the possible points after reflecting over vertex
        for x, y in obstacle:
            for dx, dy in [[.18, .18], [.18, -.18], [-.18, .18], [-.18, -.18]]:
                reflection_coords.append([x + dx, y + dy])

        all_reflection_coords.append(reflection_coords)

    # Compute the convex hull points and get the vericies that make it up
    for reflection_coord in all_reflection_coords:
        convex_hull_coords = []
        convexHull = ConvexHull(reflection_coord)
        for index in convexHull.vertices:
            convex_hull_coords.append(reflection_coord[index])
        all_convex_hull_coords.append(convex_hull_coords)
        
    return all_convex_hull_coords


def get_m(v1, v2):
    # Get the slope between two points
    x1, y1 = v1
    x2, y2 = v2
    return ((y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else None)

def get_b(vertex, m):
    # Get the intersect given a point and slope
    if m == None:
        return None
    x, y = vertex
    return y - (m * x)

def intersects(l1, l2):
    #Returns if two lines l1, l2 intersect
    
    v1, v2 = l1
    v3, v4 = l2

    if v1 == v2 or v3 == v4:
        return True
    if v1 == v3 or v1 == v4:
        return False

    m1 = get_m(v1, v2)
    m2 = get_m(v3, v4)

    b1 = get_b(v2, m1)
    b2 = get_b(v4, m2)

    if m1 == m2:
        return False
    

    if m1 == None:
        x_intersect = v1[0]
        y_intersect = (m2 * x_intersect) + b2
        
        min_x = min(v3[0], v4[0])
        max_x = max(v3[0], v4[0])
        min_y = min(v3[1], v4[1])
        max_y = max(v3[1], v4[1])
        return min_x < x_intersect < max_x and min_y < y_intersect < max_y and min(v1[1], v2[1]) < y_intersect < max(v1[1], v2[1])
    elif m2 == None:
        x_intersect = v3[0]
        y_intersect = (m1 * x_intersect) + b1
        
        min_x = min(v1[0], v2[0])
        max_x = max(v1[0], v2[0])
        min_y = min(v1[1], v2[1])
        max_y = max(v1[1], v2[1])
        return min_x < x_intersect < max_x and min_y < y_intersect < max_y and min(v3[1], v4[1]) < y_intersect < max(v3[1], v4[1])
    elif m1 == 0.0:
        y_intersect = v1[1]
        x_intersect = (y_intersect - b2) / m2

        min_x = min(v3[0], v4[0])
        max_x = max(v3[0], v4[0])
        min_y = min(v3[1], v4[1])
        max_y = max(v3[1], v4[1])
        return min_x < x_intersect < max_x and min_y < y_intersect < max_y and min(v1[0], v2[0]) < x_intersect < max(v1[0], v2[0])
    elif m2 == 0.0:
        y_intersect = v3[1]
        x_intersect = (y_intersect - b1) / m1

        min_x = min(v1[0], v2[0])
        max_x = max(v1[0], v2[0])
        min_y = min(v1[1], v2[1])
        max_y = max(v1[1], v2[1])
        return min_x < x_intersect < max_x and min_y < y_intersect < max_y and min(v3[0], v4[0]) < x_intersect < max(v3[0], v4[0])
    else:
        x_intersect = (b2 - b1) / (m1 - m2)
        y_intersect = (m1 * x_intersect) + b1
        x1, y1 = v1
        x2, y2 = v2
        x3, y3 = v3
        x4, y4 = v4

        return ((((x1 > x_intersect > x2) and (y1 > y_intersect > y2)) or ((x1 < x_intersect < x2) and (y1 > y_intersect > y2)) or ((x1 > x_intersect > x2) and (y1 < y_intersect < y2)) or ((x1 < x_intersect < x2) and (y1 < y_intersect < y2))) and (((x3 > x_intersect > x4) and (y3 > y_intersect > y4)) or ((x3 < x_intersect < x4) and (y3 > y_intersect > y4)) or ((x3 > x_intersect > x4) and (y3 < y_intersect < y4)) or ((x3 < x_intersect < x4) and (y3 < y_intersect < y4))))
    

def get_vgraph_lines(possible_lines, avoid_lines):
    # Generate the lines that make the vgraph
    
    v_graph_lines = []

    # Get the lines that can be crossed by checking if available lines intersect with avoid_lines
    for possible_line in possible_lines:
        can_draw = True
        
        for avoid_line in avoid_lines:
            # If lines intersect, don't add them
            if intersects(possible_line, avoid_line):
                can_draw = False
                break
            
        if can_draw:
            v_graph_lines.append(possible_line)

    return v_graph_lines
    

def get_distance(l1, l2):
    #Returns the distance between two points
    return math.pow(math.pow(l1[0] - l2[0], 2) + math.pow(l1[1] - l2[1], 2), 0.5)


def modified_dijkstras(lines, goal_in_cm):
    # A modified version of dijkstras for finding shortest path
    
    #Set of all points
    vertices = set([tuple(point) for line in lines for point in line])

    # Get the neighbors
    neighbors = defaultdict(list)
    for line in lines:
        v1, v2 = line
        neighbors[tuple(v1)].append(v2)
    
    prev_point = {}  # Store prev point on shortest path
    dist_point = {}  # Store the shortest dist getting to each point
    
    for key, value in neighbors.items():
        prev_point[key] = None
        dist_point[key] = float('inf')

    available_points = {}
    start = (0.0,0.0)
    available_points[start] = 0
    
    dist_point[start] = 0
    seen = set(start)
    
    while len(available_points) > 0:
        curr = min(available_points, key=lambda k: dist_point[k])
        curr_neighbors = neighbors[curr]
        del available_points[curr]
        seen.add(curr)
        
        for point in curr_neighbors:
            distance = dist_point[curr] + get_distance(curr, point)
            if dist_point[tuple(point)] > distance:
                dist_point[tuple(point)] = distance
                prev_point[tuple(point)] = curr
                if tuple(point) not in seen:
                    available_points[tuple(point)] = distance

    path = []
    curr_point = tuple(goal_in_cm)
    
    while True:
        path.append(curr_point)
        if curr_point != start:
            curr_point = prev_point[curr_point]
        else:
            break
        
    path = path[::-1]
    return path


if __name__ == "__main__":
    obstacles = load_obstacles("../data/world_obstacles.txt")
    goal = load_goal("../data/goal.txt")
    start = [0, 0]

    obstacles_in_cm = [np.divide(np.array(obstacle, 'float'), 100).tolist() for obstacle in obstacles]
    goal_in_cm = np.divide(np.array(goal, 'float'), 100).tolist()
    start_in_cm = [0.0, 0.0]

    convexhull_coords = get_obstacles_convexhull_points(obstacles_in_cm)

    line_markers = Markers()
    for coord in convexhull_coords:
        # Add the first point to complete the border
        coord.append(coord[0])

        # Draw the border
        line_markers.add_marker(coord, False)

    # Wait 3 seconds
    time.sleep(3)
        
    # Get the lines around the obstacles
    grown_obstacle_lines = []
    for coord in convexhull_coords:
        for i in range(len(coord) - 1):
            grown_obstacle_lines.append((coord[i], coord[i+1]))

    # Generate all possible lines that could occur
    possible_lines = []
    for i, hull_v in enumerate(convexhull_coords):
        for v1 in hull_v:
            for v2 in [vertex for hull_v in convexhull_coords[0:i] + convexhull_coords[i+1:] for vertex in hull_v] + [goal_in_cm] + [start_in_cm]:
                possible_lines.append((v1, v2))


    # Get the lines of the obstacles border
    obstacle_lines =  [(v1, v2) for obstacle_v in obstacles for v1 in obstacle_v for v2 in obstacle_v if obstacle_v.index(v1) < obstacle_v.index(v2) or (v1 == obstacle_v[-1] and v2 == obstacle_v[0])]

    # Get the lines that make up the vgraph
    vgraph_lines = get_vgraph_lines(possible_lines, grown_obstacle_lines + obstacle_lines)

    #Display all possible path lines
    for line in vgraph_lines:
        line_markers.add_marker(line, False)

    # Wait 3 seconds
    time.sleep(3)

    all_lines = grown_obstacle_lines + vgraph_lines
    flipped = [line[::-1] for line in all_lines]
    
    # All lines with both possible start vertices
    lines = all_lines + flipped

    # Get the points that make up the shortest path
    traverse_points = modified_dijkstras(lines, goal_in_cm)
    
    
    path_lines = []
    
    #Display all possible path_lines
    for i in range(len(traverse_points)-1):
        path_lines.append([list(traverse_points[i]), list(traverse_points[i+1])])
    
    # Display the shortest path
    for line in path_lines:
        line_markers.add_marker(line, True)

    # Wait 3 seconds
    time.sleep(3)

    # Make a robot and have him traverse the points
    r_robot = Robot()
    r_robot.navigate(traverse_points)
    



