#!/usr/bin/env python
import rospy
import math
import rospkg
import os
import yaml
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
from path_planning import *

ROBOT_NAME = "mobile_base"   # TurtleBot2 base name

def get_robot_position():
    rospy.wait_for_service('/gazebo/get_model_state')
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    state = get_state("mobile_base", "")  # model name in Gazebo

    x = state.pose.position.x#initiates the first phase of PRM
    y = state.pose.position.y

    q = state.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    return x, y, yaw


def navigate_to_waypoint(pub, wx, wy):
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rx, ry, ryaw = get_robot_position()

        dx = wx - rx
        dy = wy - ry
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.1:
           break

        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - ryaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()

        if abs(angle_error) > 0.2:
            cmd.angular.z = 2.0 * angle_error
        else:
            cmd.linear.x =min(0.25,0.5*dist)
	if abs(angle_error) > 0.3:
            cmd.linear.x = 0
    	    cmd.angular.z = 1.5 * angle_error
	else:
    	    cmd.linear.x = min(0.25, 0.5 * dist)
    	    cmd.angular.z = 0
        pub.publish(cmd)
        rate.sleep()

def main():
    rospy.init_node("navigator_node")
    pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('surveillance_bot')
    map_path =os.path.join(pkg_path, 'maps', 'my_map.yaml') 
    with open(map_path, 'r') as f:
        map_data = yaml.safe_load(f)
        
    # YAML gives a relative path like "my_map.pgm"
    pgm_relative = map_data["image"]
    
    # Build absolute path based on the YAML file location
    yaml_dir = os.path.dirname(map_path)
    pgm_file = os.path.join(yaml_dir, pgm_relative)
    
    #Convert map to grid
    gridmap = loadpgm(pgm_file)
    #gridmap = [row for row in gridmap if len(row) > 0]
    
    # Convert grid to nodes + obstacles
    mapnodes, obstacles = converttonodes(gridmap)
    
    # PRM sampling
    samples = PRMphase1(obstacles, gridmap)
    rospy.loginfo("Map shape: {} x {}".format(gridmap.shape[0], gridmap.shape[1]))
    rospy.loginfo("Map min value: {}, max value: {}".format(np.min(gridmap),np.max(gridmap)))
    rospy.loginfo("Unique map values: {}".format(np.unique(gridmap)))
    rospy.loginfo("Free (254): {}".format(np.sum(gridmap == 254)))
    rospy.loginfo("Unknown (205): {}".format(np.sum(gridmap == 205)))
    rospy.loginfo("Obstacle (0): {}".format(np.sum(gridmap == 0)))
    start=None
    goal=None
    while not rospy.is_shutdown():
        # Ask user for goal
        gx = float(input("Enter goal x: "))
        gy = float(input("Enter goal y: "))
        goal = (gx, gy)

        start = get_robot_position()[:2]
        rospy.loginfo("Start position {},{}".format(int(start[0]), int(start[1])))
        rospy.loginfo("Planning path using PRM...")
        
        start_str = "{},{}".format(float(start[0]), float(start[1]))
        goal_str = "{},{}".format(float(goal[0]), float(goal[1]))
        start,goal= goalstartinit(start_str, goal_str)import os
        rospy.loginfo("Start node: ({}, {})".format(start.x, start.y))
        rospy.loginfo("Goal node: ({}, {})".format(goal.x, goal.y))
        reset_nodes(samples)
        path= PRMphase2(gridmap, obstacles, samples, goal, start)
        rospy.loginfo("Start neighbors: {}".format(len(start.neighbours)))
        rospy.loginfo("Goal neighbors: {}".format(len(goal.neighbours)))
        path_nodes =converttoworld(path)
        rospy.loginfo("Converted path to {} waypoints".format(len(path_nodes)))
        waypoints = path_nodes
        rospy.loginfo("Got {} waypoints".format(len(waypoints)))
	
        for wx, wy in waypoints:
	    rospy.loginfo("Going {},{} waypoint".format(wx,wy))
            navigate_to_waypoint(pub, wx, wy)
		
    rospy.loginfo("Navigation complete.")

if __name__ == "__main__":
    main()
