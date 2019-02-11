#!/usr/bin/env python
"""
# Spline-based planning node

Subscribe to gate (PoseArray) and odometry messages.
Construct trajectory by smoothly connecting current post and upcoming gates via spline.
Publish MultiDOFJointTrajectory.
"""

from __future__ import print_function
import rospy
import math
from geometry_msgs.msg import PoseArray, Pose, Point, Transform, Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from scipy.interpolate import interp1d
import ros_geometry as geo

class SplinePlanner():

    def __init__(self, _node_name, _namespace):
        self.node_name = _node_name
        self._namespace = _namespace
        self.gates = []
        self.last_visited_gate = None
        self.odometry = None
        self.max_speed = rospy.get_param("~target_exit_speed", 1.0)
        self.gates_sub = rospy.Subscriber('gt_gates_publisher/gt_gates', PoseArray, self.gates_callback)
        self.odometry_sub = rospy.Subscriber('/' + self._namespace + '/odometry_sensor1/odometry', Odometry, self.odometry_callback)
        self.traj_pub = rospy.Publisher('/' + _namespace + '/command/trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)

    def gates_callback(self, msg):
        self.gates = msg.poses
        # print("gates_callback " + str(msg.poses))

    def odometry_callback(self, msg):
        # http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
        self.odometry = msg
        for gate in self.gates:
            if geo.distance(self.odometry.pose.pose.position, gate.position) < 1.0:
                if self.last_visited_gate is None or self.last_visited_gate != gate:
                    self.last_visited_gate = gate
                    #print("Visited gate:")
                    #print(str(gate))

    def start(self):
        rate = rospy.Rate(1)
        started = False
        while not rospy.is_shutdown():
            if len(self.gates) > 0 and self.odometry is not None:
                trajectory = self.create_path(self.odometry.pose.pose.position, self.odometry.twist.twist.linear, self.gates, self.last_visited_gate)
                if trajectory is None:
                    print("Failed to create trajectory")
                else:
                    self.traj_pub.publish(trajectory)
                    if not started:
                        started = True
                        print("spline_planner published first trajectory")
                    #print("Trajectory:")
                    #print(str(trajectory))
            else:
                print("spline_planner waiting for input")
            rate.sleep()

    def create_path(self, start_position, start_velocity, gates, last_visited_gate): # types are Point, Vector3, [Pose], Pose
        if last_visited_gate is not None:
            visited_index = None
            for i in range(len(gates)):
                if geo.distance(last_visited_gate.position, gates[i].position) < 1.0:
                    visited_index = i
            if visited_index is not None:
                gates = gates[visited_index+1:] + gates[:visited_index+1]

        waypoints = [start_position]

        #start_speed = geo.magnitude_vector(start_velocity)
        #if start_speed > 0.3:
        #    waypoints.append(geo.point_plus_vector(start_position, geo.normalize(start_velocity, 0.1)))

        for gate in gates:
            for offset in [-0.1,0.1]:
                waypoints.append(geo.point_plus_vector(gate.position, geo.quaternion_to_vector(gate.orientation, offset)))

        waypoint_spline = geo.points_to_spline(waypoints)

        # TODO: determine appropriate speeds based on start speed, max speed, and max acceleration (centripetal acceleration limits speed on tight curve)
        # For now just use constant speed of self.max_speed.

        # TODO: determine thrust direction at each point, based on linear acceleration, centripetal acceleration, gravity, and drag.
        # For now just set accelerations to 0.

        # TODO: determine orientation required to achieve that thrust while keeping quad mostly facing in direction of motion
        # For now just using constant orientation.

        trajectory = MultiDOFJointTrajectory()
        trajectory.header.frame_id=''
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = ['base']
        ds = 1.0
        for i in range(10):
            distance = i * ds
            transform = Transform()
            transform.translation = geo.point_to_vector(geo.spline_distance_to_point(waypoint_spline, distance))
            # TODO: transform.rotation
            velocity = Twist()
            velocity.linear = geo.scalar_multiply(self.max_speed, geo.spline_distance_to_tangent(waypoint_spline, distance))
            # TODO: velocity.angular
            acceleration = Twist()
            # TODO: acceleration.linear,angular
            trajectory.points.append(MultiDOFJointTrajectoryPoint([transform], [velocity], [acceleration], rospy.Duration(distance / self.max_speed)))

        return trajectory

        # return MultiDOFJointTrajectory
        #            joint_names=['uav']
        #            points (type=MultiDOFJointTrajectoryPoint)
        #                transforms (translation and rotation)
        #                velocities (linear and angular)
        #                accelerations (linear and angular)
        #                time_from_start (duration, first point should have duration 0.0)
        #
        # http://docs.ros.org/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html
        # https://github.com/ethz-asl/rotors_simulator/issues/510#issuecomment-414996170
        #
        # Note: The "multi" part is inappropriate, so lots of lists with one element each.

if __name__ == '__main__':
    _node_name = 'spline_planner'
    _namespace = 'firefly'
    print('* {} starting... '.format(_node_name), end="")
    rospy.init_node(_node_name, anonymous=True)
    node = SplinePlanner(_node_name, _namespace)
    print('Ready.')
    node.start()
    rospy.spin()

"""
Misc planning references

http://wiki.ros.org/base_local_planner
http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
http://wiki.ros.org/octomap
"""

