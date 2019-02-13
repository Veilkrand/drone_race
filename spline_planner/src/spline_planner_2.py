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
from geometry_msgs.msg import PoseArray, Pose, Point, Transform, Twist, Vector3
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from scipy.interpolate import interp1d
import ros_geometry as geo
from SmoothedPath import SmoothedPath
import numpy as np

class SplinePlanner():

    def __init__(self, _node_name, _namespace):
        self.node_name = _node_name
        self._namespace = _namespace
        self.gates = []
        self.last_visited_gate = None
        self.odometry = None
        self.min_speed = 0.1
        self.max_speed = rospy.get_param("~target_exit_speed", 1.0)
        self.max_acceleration = 3 # Racing drone is probably around 10 m/s2. Set very low for now.
        self.gates_sub = rospy.Subscriber('gt_gates_publisher/gt_gates', PoseArray, self.gates_callback)
        self.odometry_sub = rospy.Subscriber('/' + self._namespace + '/odometry_sensor1/odometry', Odometry, self.odometry_callback)
        self.traj_pub = rospy.Publisher('/' + _namespace + '/command/trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)

    def gates_callback(self, msg):
        self.gates = msg.poses
        # print("gates_callback " + str(msg.poses))

    def odometry_callback(self, msg):
        # http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
        self.odometry = msg
        self.odometry_time = rospy.Time.now()
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
                trajectory = self.create_path(self.odometry, self.gates, self.last_visited_gate)
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

    def create_path(self, odometry, gates, last_visited_gate): # types are Odometry, [Pose], Pose
        start_position = odometry.pose.pose.position
        start_velocity = odometry.twist.twist.linear
        start_time = odometry.header.stamp
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
            for offset in [-0.5, 0.5]:
                waypoints.append(geo.point_plus_vector(gate.position, geo.quaternion_to_vector(gate.orientation, offset)))

        # waypoint_spline = geo.points_to_spline(waypoints)
        path = SmoothedPath()
        path.fit(waypoints)
        
        # TODO: determine appropriate speeds based on start speed, max speed, and max acceleration (centripetal acceleration limits speed on tight curve)
        # For now just use constant speed of self.max_speed.
        ds = 0.2
        trajectory_points = []
        def visit_cb(pt, deriv1, deriv2, s):
            # curvature is calcuated as norm(deriv1 x deriv2) / norm(deriv1)**3
            # see: https://en.wikipedia.org/wiki/Curvature#Local_expressions_2
            d1xd2 = np.cross(deriv1, deriv2)
            norm_d1 = np.linalg.norm(deriv1)
            c = np.linalg.norm(d1xd2) / norm_d1 ** 3
            # the first order derivative is given as ds/dt, where s is the arc length and t is the internal parameter of the spline,
            # not time.
            # because of this, the magnitude of first order derivative is not the same with viable speed,
            # but nevertheless, the direction of the derivative of them are the same.
            trajectory_points.append({
                's': s,
                'point': Vector3(*(pt.tolist())),
                'speed': self.max_speed,
                'speed_direction': Vector3(*((deriv1 / norm_d1).tolist())),
                'curvature': c
            })
        path.visit_at_interval(ds, visit_cb, ds * 50)
#        trajectory_points = [{'s': i * ds, 'speed': self.max_speed, 'curvature': geo.spline_distance_to_curvature(waypoint_spline, i*ds)} for i in range(30)]
        vmin = self.min_speed
        vmax = self.max_speed
        amax = self.max_acceleration

        def safe_speed(curvature, speed_neighbor):
            centripetal = curvature * speed_neighbor**2
            if centripetal >= amax:
                return max(vmin, min(vmax, math.sqrt(abs(amax / curvature))))
            remaining_acceleration = math.sqrt(amax**2 - centripetal**2)
            # see /Planning Motion Trajectories for Mobile Robots Using Splines/
            # (refered as Sprunk[2008] later) for more details (eq 3.21)
            v_this = math.sqrt(speed_neighbor ** 2 + 2 * ds * remaining_acceleration)
            return max(vmin, min(vmax, v_this))

        trajectory_points[0]['speed'] = min(vmax, max(vmin, geo.magnitude_vector(start_velocity)))
        num_traj = len(trajectory_points)
        for i in range(num_traj - 1): # iterate forwards, skipping first point which is fixed to current speed
            trajectory_points[i+1]['speed'] = safe_speed(trajectory_points[i+1]['curvature'], trajectory_points[i]['speed'])
        for i in range(num_traj - 2):
            j = num_traj - i - 2 # iterate backwards, skipping both end points
            curvature = trajectory_points[j]['curvature']
            min_neighbor_speed = min(trajectory_points[j-1]['speed'],trajectory_points[j+1]['speed'])
            trajectory_points[j]['speed'] = safe_speed(curvature, min_neighbor_speed)

        # Set velocity based on speed and direction
        for point in trajectory_points:
            point['velocity'] = geo.scalar_multiply(point['speed'], point['speed_direction'])

        # Set time based on speed and distance
        trajectory_points[0]['time'] = 0.0
        for i in range(num_traj - 1):
            prev_time = trajectory_points[i]['time']
            # see eq 3.20
            ave_speed = 0.5 * (trajectory_points[i]['speed'] + trajectory_points[i+1]['speed'])
            trajectory_points[i+1]['time'] = prev_time + ds / ave_speed

        # Set acceleration based on change in velocity
        dt = trajectory_points[1]['time']
        deltav = geo.vector_from_to(trajectory_points[0]['velocity'], trajectory_points[1]['velocity'])
        trajectory_points[0]['acceleration'] = geo.scalar_multiply(1.0/dt, deltav)
        for i in range(num_traj - 2):
            j = i + 1 # iterate skipping both ends
            dt = trajectory_points[j+1]['time'] - trajectory_points[j-1]['time']
            deltav = geo.vector_from_to(trajectory_points[j-1]['velocity'], trajectory_points[j+1]['velocity'])
            trajectory_points[j]['acceleration'] = geo.scalar_multiply(1.0/dt, deltav)
        dt = trajectory_points[-1]['time'] - trajectory_points[-2]['time']
        deltav = geo.vector_from_to(trajectory_points[-2]['velocity'], trajectory_points[-1]['velocity'])
        trajectory_points[0-1]['acceleration'] = geo.scalar_multiply(1.0/dt, deltav)

        # TODO: determine thrust direction at each point, based on linear acceleration, centripetal acceleration, gravity, and drag.
        # For now just set accelerations to 0.

        # TODO: determine orientation required to achieve that thrust while keeping quad mostly facing in direction of motion
        # For now just using constant orientation.

        trajectory = MultiDOFJointTrajectory()
        trajectory.header.frame_id=''
        trajectory.header.stamp = start_time
        trajectory.joint_names = ['base']
        # remove the first trajectory point (current poisition)
        for point in trajectory_points[1:]:
            transform = Transform()
            transform.translation = point['point']
            # TODO: transform.rotation
            velocity = Twist()
            velocity.linear = point['velocity']
            # TODO: velocity.angular
            acceleration = Twist()
            acceleration.linear = point['acceleration']
            # TODO: acceleration.angular
            trajectory.points.append(MultiDOFJointTrajectoryPoint([transform], [velocity], [acceleration], rospy.Duration(point['time'])))

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

