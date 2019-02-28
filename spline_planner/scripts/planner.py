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
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from scipy.interpolate import interp1d
import ros_geometry as geo
from SmoothedPath import SmoothedPath
from LowPassFilter import LowPassFilter
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
        self.raw_waypoints_viz_pub = rospy.Publisher("~raw_waypoints_viz", Marker, queue_size=1, latch=True)
        self.current_path = None
        self.last_position = None
        self.target_gate_idx = None
        self.stop_planning = False

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
        if self.target_gate_idx is None:
            self.target_gate_idx = 0
            rospy.loginfo("Next gate index: {}".format(self.target_gate_idx))

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

        current_position = geo.vector_to_list(self.odometry.pose.pose.position)
        # print(current_position, self.last_position)
        if self.last_position is not None and self.target_gate_idx is not None:
            # check if the drone has crossed the gate
            current_gate_location = geo.vector_to_list(self.gates[self.target_gate_idx].position)
            if self.is_cross_gate(self.target_gate_idx, self.last_position, current_position):
                rospy.loginfo("Passed gate {}".format(self.target_gate_idx))
                if self.target_gate_idx + 1 < len(self.gates):
                    self.target_gate_idx += 1
                    rospy.loginfo("Next gate index: {}".format(self.target_gate_idx))
                else:
                    rospy.loginfo("All gates have been visited.")
            
        self.last_position = current_position

    def start(self):
        rate = rospy.Rate(2)
        started = False
        while not rospy.is_shutdown():
            if not self.stop_planning:
                if started and False:
                    pass
                elif len(self.gates) > 0 and self.odometry is not None and self.target_gate_idx is not None:
                    trajectory = self.create_path(self.odometry, self.gates, self.last_visited_gate)
                    if self.stop_planning:
                        rospy.loginfo("All gates have been visited. Planning stopped.")
                    elif trajectory is None:
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

    def search_for_nearest_waypoint(self, position):
        for i in range(len(self.current_path['path']) - 1, -1, -1):
            # point in path format: ((s, t), (pt, deriv1))
            prev_pt = self.current_path['path'][i - 1]["point_as_list"]
            next_pt = self.current_path['path'][i]["point_as_list"]
            vec_ref = np.array(next_pt) - np.array(prev_pt)
            vec = np.array(next_pt) - np.array(position)
            if np.dot(vec, vec_ref) <= 0:
                return i
                #return i + 1 if i < len(self.current_path['path']) - 1 else i
        return 0

    def distance_to_gate(self, position, gate_index):
        if self.gates is None or gate_index >= len(self.gates):
            rospy.logwarn("Currently no gates information or do not have gate at index {}. Will return a very large value".format(gate_index))
            return 1e6

        gate_loc = geo.vector_to_list(self.gates[gate_index].position)
        return SplinePlanner.distance(gate_loc, position)

    @staticmethod
    def distance(p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def is_cross_gate(self, gate_index, position_before, position_after):
        if SplinePlanner.distance(position_before, position_after) < 1e-5:
            # the two positions are too close
            return False
        gate_orientation = self.gates[gate_index].orientation
        gate_heading = geo.rotate_vector_wrt_quaternion(gate_orientation, Vector3(1, 0, 0))
        gate_normal_xy = np.array([-gate_heading.y, gate_heading.x])

        gate_position = np.array(geo.vector_to_list(self.gates[gate_index].position))
        position_before_xy = (np.array(position_before) - gate_position)[:2]
        position_after_xy = (np.array(position_after) - gate_position)[:2]

        return np.cross(position_before_xy, gate_normal_xy) * np.cross(position_after_xy, gate_normal_xy) < 0

    @staticmethod
    def is_cross_position(target_position, position_before, position_after):
        if SplinePlanner.distance(position_before, position_after) < 1e-5:
            # the two position is too close
            return False
        vec1 = np.array(target_position) - np.array(position_before)
        vec2 = np.array(target_position) - np.array(position_after)
        if np.dot(vec1, vec2) < 0:
            # the two vectors are in the opposite direction, so we think we have cross the target position
            return True
        return False

    @staticmethod
    def is_cross_position_debug(target_position, position_before, position_after):
        print("Distance: ", SplinePlanner.distance(position_before, position_after))
        if SplinePlanner.distance(position_before, position_after) < 1e-5:
            # the two position is too close
            return False
        vec1 = np.array(target_position) - np.array(position_before)
        vec2 = np.array(target_position) - np.array(position_after)
        print("Vec1: ", vec1)
        print("Vec2: ", vec2)
        print("Dot: ", np.dot(vec1, vec2))
        if np.dot(vec1, vec2) < 0:
            # the two vectors are in the opposite direction, so we think we have cross the target position
            return True
        return False

    def create_path(self, odometry, gates, last_visited_gate): # types are Odometry, [Pose], Pose
        start_position = odometry.pose.pose.position
        start_velocity = geo.rotate_vector_wrt_quaternion(odometry.pose.pose.orientation, odometry.twist.twist.linear)
        if last_visited_gate is not None:
            visited_index = None
            for i in range(len(gates)):
                if geo.distance(last_visited_gate.position, gates[i].position) < 1.0:
                    visited_index = i
            if visited_index is not None:
                gates = gates[visited_index+1:] + gates[:visited_index+1]

        # in case of re-planning, determine the current nearest waypoint. look forward about some specific 
        # time into the current path (currently 1s) and let that future waypoint be the starting waypoint of 
        # re-planning
        waypoints = []
        start = geo.vector_to_list(start_position)
        look_ahead_time = 1
        nearest_waypoint_idx = None
        planning_waypoint_idx = None
        current_gate_location = geo.vector_to_list(self.gates[self.target_gate_idx].position)
        if self.current_path is not None:
            nearest_waypoint_idx = self.search_for_nearest_waypoint(geo.vector_to_list(start_position))
            if nearest_waypoint_idx >= 0:
                current_path = self.current_path["path"]
                waypoints.append(current_path[nearest_waypoint_idx]["point_as_list"])

                waypoint_time = current_path[nearest_waypoint_idx]["time"]

                planning_waypoint_idx = nearest_waypoint_idx
                current_path_length = len(current_path)
                while planning_waypoint_idx < current_path_length:
                    planning_waypoint = current_path[planning_waypoint_idx]
                    if planning_waypoint["time"] - waypoint_time < look_ahead_time:
                        planning_waypoint_idx += 1
                    else:
                        break
                if planning_waypoint_idx >= current_path_length:
                    planning_waypoint_idx = current_path_length - 1
                start_position = current_path[planning_waypoint_idx]["point_as_list"]
                waypoints.append(start_position)
        
        if len(waypoints) == 0:
            print("The first planning or a re-planning from scratch is needed.")
            orientation = np.array(current_gate_location) - np.array(start)
            orientation_normalized = orientation / np.linalg.norm(orientation)
            # append a point 5 meters behind along with the current position as the starting position
            waypoints.append((np.array(start) - 5 * orientation_normalized).tolist())
            waypoints.append(start)
        else:
            print("Re-planning")

        # we already have two position around current position,
        # we still need 2 positionp around the gate
        # 
        # make sure we won't going back if we're still heading to the current gate
        # and if we're too close to the target gate, head for the next too.
        target_gate_location = geo.vector_to_list(self.gates[self.target_gate_idx].position)
        if self.is_cross_gate(self.target_gate_idx, waypoints[0], waypoints[1]):
            if self.target_gate_idx + 1 < len(self.gates):
                rospy.loginfo("About to cross gate{}. Heading for next gate at index {}".format(self.target_gate_idx, self.target_gate_idx + 1))
                gate = self.gates[self.target_gate_idx + 1]
            else:
                rospy.loginfo("Gates are about to be all passed. Stop planning.")
                gate = self.gates[self.target_gate_idx]
                self.stop_planning = True
                return
        elif self.distance_to_gate(waypoints[0], self.target_gate_idx) <= 0.5 or \
              self.distance_to_gate(waypoints[1], self.target_gate_idx) <= 0.5:
            if self.target_gate_idx + 1 < len(self.gates):
                rospy.loginfo("Close to gate {}. Heading for gate at index {}".format(self.target_gate_idx, self.target_gate_idx + 1))
                gate = self.gates[self.target_gate_idx + 1]
            else:
                rospy.loginfo("Gates are about to be all passed. Stop planning.")
                gate = self.gates[self.target_gate_idx]
                self.stop_planning = True
                return
        else:
            gate = self.gates[self.target_gate_idx]

        # append waypoints +- 0.5 meters around the next gate
        offsets = [-0.5, 0.5]
        for offset in offsets:
            waypoints.append(
                geo.vector_to_list(
                    geo.point_plus_vector(gate.position, geo.quaternion_to_vector(gate.orientation, offset))))

        # wp = np.array(waypoints)
        # w0 = wp[:-1]
        # w1 = wp[1:]
        # diff = w0 - w1
        # norm = np.linalg.norm(diff, axis=1)
        # new_path_chord_length = np.sum(norm)
                
        # scale derivative w.r.t. total chord length
        # if deriv is not None:
        #    old_path_chord_length = self.current_path["chord_length"]
        #    ratio = min(new_path_chord_length / old_path_chord_length, 1.0)
        #    deriv = list(ratio * v for v in deriv)
        raw_waypoints_marker = Marker()
        raw_waypoints_marker.header.stamp = rospy.Time.now()
        raw_waypoints_marker.header.frame_id = "world"
        raw_waypoints_marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
        raw_waypoints_marker.scale = Vector3(0.5, 0.5, 0.5)
        raw_waypoints_marker.type = Marker.SPHERE_LIST
        raw_waypoints_marker.action = Marker.ADD
        raw_waypoints_marker.id = 1
        for wp in waypoints:
            raw_waypoints_marker.points.append(Point(wp[0], wp[1], wp[2]))
        self.raw_waypoints_viz_pub.publish(raw_waypoints_marker)
        
        path = SmoothedPath()
        path.fit(waypoints)

        trajectory_points = []
        def visit_cb(pt, deriv1, deriv2, s, t):
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

            # also note that unit normal vector at point is just the normalized second order derivative,
            # it is in the opposite direction of the radius vector

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
                't': t,
                's': s,
                'point_as_list': pt,
                'derivative': deriv1,
                'derivative2': deriv2,

                'ds': ds,
                'point': Vector3(*(pt.tolist())),
                'speed': self.max_speed,
                'speed_direction': Vector3(*(unit_d1.tolist())),
                'unit_normal': Vector3(*(unit_d2.tolist())),
                'unit_binormal': Vector3(*(unit_binormal.tolist())),
                'curvature': c
            })
        path.visit_at_interval(self.ds, visit_cb, self.trajectory_length)
        self.current_path = { "chord_length" : 0, "path": trajectory_points }
#        trajectory_points = [{'s': i * ds, 'speed': self.max_speed, 'curvature': geo.spline_distance_to_curvature(waypoint_spline, i*ds)} for i in range(30)]
        vmin = self.min_speed
        vmax = self.max_speed
        amax = self.max_acceleration

        def safe_speed(curvature, speed_neighbor, ds, accel = None):
            if accel is not None:
                return math.sqrt(speed_neighbor ** 2 + 2 * ds * accel)

            centripetal = curvature * speed_neighbor**2
            if centripetal >= amax:
                return max(vmin, min(vmax, math.sqrt(abs(amax / curvature))))
            remaining_acceleration = math.sqrt(amax**2 - centripetal**2)
            # see /Planning Motion Trajectories for Mobile Robots Using Splines/
            # (refered as Sprunk[2008] later) for more details (eq 3.21)
            v_this = math.sqrt(speed_neighbor ** 2 + 2 * ds * remaining_acceleration)
            return max(vmin, min(vmax, v_this))

        #print(geo.magnitude_vector(start_velocity))
        # trajectory_points[0]['speed'] = min(vmax, max(vmin, geo.magnitude_vector(start_velocity)))
        trajectory_points[0]['speed'] = geo.magnitude_vector(start_velocity)
        #print(trajectory_points[0]['speed'])
        num_traj = len(trajectory_points)
        for i in range(num_traj - 1): # iterate forwards, skipping first point which is fixed to current speed
            trajectory_points[i+1]['speed'] = safe_speed(trajectory_points[i+1]['curvature'], trajectory_points[i]['speed'], trajectory_points[i+1]['ds'])
        # skip the backward phase for 2 reason:
        #   1. we don't need a smooth stop. just complete the course
        #      asap
        #   2. backward phase would change the start speed,
        #      usually slower than the current speed. this
        #      sudden change of speed would make the drone
        #      "jerking" between trajectory switch.
        # for i in range(num_traj - 2):
        #    j = num_traj - i - 2 # iterate backwards, skipping both end points
        #    curvature = trajectory_points[j]['curvature']
        #    min_neighbor_speed = min(trajectory_points[j-1]['speed'],trajectory_points[j+1]['speed'])
        #    trajectory_points[j]['speed'] = safe_speed(curvature, min_neighbor_speed, trajectory_points[i+1]['ds'])

        # Set velocity based on speed and direction
        for point in trajectory_points:
            point['velocity'] = geo.scalar_multiply(point['speed'], point['speed_direction'])
            # the relationship between angular velocity and linear velocity is:
            # \omega = r x v / || r ||^2
            # where r is the radius vector, v is linear velocity.
            # see: https://en.wikipedia.org/wiki/Angular_velocity#Particle_in_three_dimensions
            # note: with the anti-commutative law of cross product,
            # unit binormal B = v x (-r) / (||v|| * ||r||) = r x v / (||v|| * ||r||)
            # and curvature k = 1 / || r ||
            # so, omega = || v || * k * B 
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

        # low pass filter on the designated speed.
        lpf = LowPassFilter(trajectory_points[0]['speed'], 5)
        for i in range(1, num_traj):
            trajectory_points[i]['speed'] = lpf.update(trajectory_points[i]['speed'], trajectory_points[i]['time'])
        
        # print(geo.magnitude_vector(start_velocity))
        # for pt in trajectory_points:
        #     print(pt['speed'])
        # print("------")

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

        trajectory = MultiDOFJointTrajectory()
        trajectory.header.frame_id=''
        trajectory.joint_names = ['base']
        for idx in range(len(trajectory_points)):
            point = trajectory_points[idx]
            point['time'] = trajectory_points[idx]['time']
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

        trajectory.header.stamp = rospy.Time.now()
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

