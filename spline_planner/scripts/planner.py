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
from geometry_msgs.msg import PoseArray, Pose, Point, Transform, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from scipy.interpolate import interp1d
import ros_geometry as geo
from SmoothedPath import SmoothedPath
import numpy as np

from dynamic_reconfigure.server import Server
from spline_planner.cfg import PlannerConfig

class SplinePlanner():

    def __init__(self, _node_name, _namespace):
        self.node_name = _node_name
        self._namespace = _namespace
        self.gates = []
        self.last_visited_gate = None
        self.odometry = None
        self.min_speed = 0.1
        self.max_speed = rospy.get_param("~target_exit_speed", 1.0)
        self.max_acceleration = 1 # Racing drone is probably around 10 m/s2. Set very low for now.
        self.gates_sub = rospy.Subscriber('gt_gates_publisher/gt_gates', PoseArray, self.gates_callback)
        self.odometry_sub = rospy.Subscriber('/' + self._namespace + '/odometry_sensor1/odometry', Odometry, self.odometry_callback)
        self.traj_pub = rospy.Publisher('/' + _namespace + '/command/trajectory', MultiDOFJointTrajectory, queue_size=1, latch=True)

    def reconfigure_parameters(self, config, level):
        rospy.loginfo("Parameters reconfiguration requested.")
        rospy.loginfo("""Parameters reconfiguration requested:
ds: {ds}
max_speed: {max_linear_speed}
max_total_acceleration: {max_total_acceleration}
trajectory_length: {trajectory_length}""".format(**config))

        self.max_speed = config.max_linear_speed
        self.max_acceleration = config.max_total_acceleration
        self.ds = config.ds
        self.trajectory_length = config.trajectory_length
        return config

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
            if started:
                pass
            elif len(self.gates) > 0 and self.odometry is not None:
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
        #for p in waypoints:
        #    print('({}, {}, {}),'.format(p.x, p.y, p.z))

        # waypoint_spline = geo.points_to_spline(waypoints)
        path = SmoothedPath()
        path.fit(waypoints)
        
        # TODO: determine appropriate speeds based on start speed, max speed, and max acceleration (centripetal acceleration limits speed on tight curve)
        # For now just use constant speed of self.max_speed.
        trajectory_points = []
        def visit_cb(pt, deriv1, deriv2, s):
            # curvature is calcuated as norm(deriv1 x deriv2) / norm(deriv1)**3
            # see: https://en.wikipedia.org/wiki/Curvature#Local_expressions_2
            d1xd2 = np.cross(deriv1, deriv2)
            norm_d1 = np.linalg.norm(deriv1)
            norm_d2 = np.linalg.norm(deriv2)
            if norm_d1 > 1e-5:
                c = np.linalg.norm(d1xd2) / norm_d1 ** 3
            else:
                c = 0
            # the first order derivative is given as ds/dt, where s is the arc length and t is the internal parameter of the spline,
            # not time.
            # because of this, the magnitude of first order derivative is not the same with viable speed,
            # but nevertheless, the direction of the derivative of them are the same.

            # also note that unit normal vector at point is just the second order derivative,
            # and it is in the opposite direction of the radius vector

            # the cross product of unit tangent vector and unit normal vector
            # is also mentioned as unit binormal vector
            if norm_d1 > 1e-5:
                unit_d1 = deriv1 / norm_d1
            else:
                unit_d1 = np.array([0.0, 0.0, 0.0])
                
            if norm_d2 > 1e-5:
                unit_d2 = deriv2 / norm_d2
            else:
                unit_d2 = np.array([0.0, 0.0, 0.0])
                
            unit_binormal = np.cross(unit_d1, unit_d2)

            ds = 0
            if len(trajectory_points) > 0:
                last_point = trajectory_points[-1]
                ds = s - last_point['s']
            
            trajectory_points.append({
                's': s,
                'ds': ds,
                'point': Vector3(*(pt.tolist())),
                'speed': self.max_speed,
                'speed_direction': Vector3(*(unit_d1.tolist())),
                'unit_normal': Vector3(*(unit_d2.tolist())),
                'unit_binormal': Vector3(*(unit_binormal.tolist())),
                'curvature': c
            })
        path.visit_at_interval(self.ds, visit_cb, self.trajectory_length)
#        trajectory_points = [{'s': i * ds, 'speed': self.max_speed, 'curvature': geo.spline_distance_to_curvature(waypoint_spline, i*ds)} for i in range(30)]
        vmin = self.min_speed
        vmax = self.max_speed
        amax = self.max_acceleration

        def safe_speed(curvature, speed_neighbor, ds):
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
            trajectory_points[i+1]['speed'] = safe_speed(trajectory_points[i+1]['curvature'], trajectory_points[i]['speed'], trajectory_points[i+1]['ds'])
        for i in range(num_traj - 2):
            j = num_traj - i - 2 # iterate backwards, skipping both end points
            curvature = trajectory_points[j]['curvature']
            min_neighbor_speed = min(trajectory_points[j-1]['speed'],trajectory_points[j+1]['speed'])
            trajectory_points[j]['speed'] = safe_speed(curvature, min_neighbor_speed, trajectory_points[i+1]['ds'])

        # Set velocity based on speed and direction
        for point in trajectory_points:
            point['velocity'] = geo.scalar_multiply(point['speed'], point['speed_direction'])
            # the relationship between angular velocity and linear velocity is:
            # \omega = r x v / || r ||^2
            # where r is the radius vector, v is linear velocity.
            # see: https://en.wikipedia.org/wiki/Angular_velocity#Particle_in_three_dimensions
            # note: with the anti-commutative law of cross product,
            # unit binomal = v x (-r) / (||v|| * ||r||) = r x v / (||v|| * ||r||)
            omega = geo.scalar_multiply(geo.magnitude_vector(point['velocity']) * point['curvature'], point['unit_binormal'])
            point['omega'] = omega

        # Set time based on speed and distance
        trajectory_points[0]['time'] = 0.0
        for i in range(num_traj - 1):
            ds = trajectory_points[i+1]['ds']
            prev_time = trajectory_points[i]['time']
            # see Sprunk[2018] eq 3.20
            ave_speed = 0.5 * (trajectory_points[i]['speed'] + trajectory_points[i+1]['speed'])
            trajectory_points[i+1]['time'] = prev_time + ds / ave_speed

        # Set acceleration based on change in velocity
        # we're assuming constant accelerations between waypoints,
        # a is just \delta V / \delta t
        for i in range(num_traj - 1):
            t_current = trajectory_points[i]['time']
            t_next = trajectory_points[i + 1]['time']
            dt = t_next - t_current
            
            v_current = trajectory_points[i]['velocity']
            v_next = trajectory_points[i + 1]['velocity']
            dv = geo.vector_from_to(v_current, v_next)
            
            w_current = trajectory_points[i]['omega']
            w_next = trajectory_points[i]['omega']
            dw = geo.vector_from_to(w_current, w_next)
            
            trajectory_points[i]['acceleration'] = geo.scalar_multiply(1.0/dt, dv)
            trajectory_points[i]['acceleration_w'] = geo.scalar_multiply(1.0/dt, dw)
        trajectory_points[-1]['acceleration'] = trajectory_points[-2]['acceleration']
        trajectory_points[-1]['acceleration_w'] = trajectory_points[-2]['acceleration_w']

        # TODO: determine thrust direction at each point, based on linear acceleration, centripetal acceleration, gravity, and drag.
        # For now just set accelerations to 0.

        # TODO: determine orientation required to achieve that thrust while keeping quad mostly facing in direction of motion
        # For now just using constant orientation.

        trajectory = MultiDOFJointTrajectory()
        trajectory.header.frame_id=''
        trajectory.header.stamp = start_time
        trajectory.joint_names = ['base']
        for point in trajectory_points:
            transform = Transform()
            transform.translation = point['point']
            transform.rotation = Quaternion(*(geo.vector_to_quat(point['velocity']).tolist()))
            velocity = Twist()
            velocity.linear = point['velocity']
            velocity.angular = point['omega']
            
            acceleration = Twist()
            acceleration.linear = point['acceleration']
            acceleration.angular = point['acceleration_w']
            
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
    rospy.init_node(_node_name)
    node = SplinePlanner(_node_name, _namespace)
    srv = Server(PlannerConfig, node.reconfigure_parameters)
    print('Ready.')
    node.start()
    rospy.spin()

"""
Misc planning references

http://wiki.ros.org/base_local_planner
http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
http://wiki.ros.org/octomap
"""

