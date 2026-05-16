#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
from path_planning import prm_plan

ROBOT_NAME = "mobile_base"   # TurtleBot2 base name

def get_robot_position():
    rospy.wait_for_service('/gazebo/get_model_state')
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    state = get_state("mobile_base", "")  # model name in Gazebo

    x = state.pose.position.x
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

        if dist < 0.3:
           break

        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - ryaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()

        if abs(angle_error) > 0.2:
            cmd.angular.z = 1.0 * angle_error
        else:
            cmd.linear.x = 0.25

        pub.publish(cmd)
        rate.sleep()

def main():
    rospy.init_node("navigator_node")
    pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)

    map_path = "/home/ros/robot_assignment_ws/my_map.yaml"

    # Ask user for goal
    gx = float(input("Enter goal x: "))
    gy = float(input("Enter goal y: "))
    goal = (gx, gy)

    start = get_robot_position()[:2]

    rospy.loginfo("Planning path using PRM...")
    
    start_str = "{},{}".format(int(start[0]), int(start[1]))
    goal_str = "{},{}".format(int(goal[0]), int(goal[1]))

    waypoints = prm_plan(map_path, start_str, goal_str)
    rospy.loginfo("Got {} waypoints".format(len(waypoints)))

    for wx, wy in waypoints:
        navigate_to_waypoint(pub, wx, wy)

    rospy.loginfo("Navigation complete.")

if __name__ == "__main__":
    main()

