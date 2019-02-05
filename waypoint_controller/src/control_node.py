#!/usr/bin/env python
"""
# Waypoint control node

Subscribe to waypoints messages.
Iterate waypoints:
    - Send current waypoint control command
    - Calculate condition to advance to next waypoint

"""

from __future__ import print_function
import rospy


import math
from geometry_msgs.msg import Pose, Point

import ros_geometry as geo
from CommandPublisher import CommandPublisher


class WaypointController():

    def __init__(self, _node_name, _namespace, _poses):
        self.node_name = _node_name
        self.poses = _poses

        self._namespace = _namespace

        self._last_distance = None # To measure distance to waypoint

        # Params
        self.target_exit_speed = rospy.get_param("~target_exit_speed", 1)


        # Publisher topics

        ## Command publisher
        self.command_pub = CommandPublisher(_namespace)
        self.waypoint_index = -1

    def start(self):
        # First Waypoint
        # self.command_pub.publish_pose_command(poses[0][0], poses[0][1], poses[0][2], poses[0][3], poses[0][4], poses[0][5])

        # Subscriber
        self.ego_pose_sub = rospy.Subscriber('/' + self._namespace + '/odometry_sensor1/pose', Pose, self._callback, queue_size=1)
        self.latest_msg = None  # to keep latest received message
        self.new_msg_available = False

        ## Rospy loop
        r = rospy.Rate(100)  # 30 # 10 Hz
        while not rospy.is_shutdown():
            if self.new_msg_available:
                self.new_msg_available = False

                self.process_msg(self.latest_msg)

                # rospy.loginfo(msg)
            r.sleep()


    def process_msg(self, msg):

        _exit_speed = self.target_exit_speed #waypoint exit speed in m/s
        _distance_threshold = 2 # in m to switch to the next waypoint

        waypoint = Point(
                            self.poses[self.waypoint_index][0],
                            self.poses[self.waypoint_index][1],
                            self.poses[self.waypoint_index][2]
                            )


        d = geo.euclidean_distance(msg.position, waypoint)

        if self._last_distance is None:
            self._last_distance = d

        delta_d = self._last_distance - d

        self._last_distance = d

        print("W[{}] delta:{}m D:{}m".format(self.waypoint_index, delta_d, d))


        # Method 2 for threshold to switch waypoint:
        # Use the distance from the plane of the waypoint.
        # It should be the scalar projection of the ego position vector (from waypoint to position point),
        # into the pose unit vector of the waypoint. This is the dot product of both vectors d=(x0*x1)+(y0*y1)+(z0*z1)
        _pos_vector = geo.vector_two_points(msg.position, waypoint)
        _waypoint_vector = geo.vector_from_RPY(
                self.poses[self.waypoint_index][3],
                self.poses[self.waypoint_index][4],
                self.poses[self.waypoint_index][5]
            )
        _distance_waypoint_plane = geo.dot_product_vectors(_pos_vector, _waypoint_vector)


        print("plane distance {}m".format(_distance_waypoint_plane))

        #if d < _threshold or (d < 2 and delta_d < 0):
        if (_distance_waypoint_plane < 0 and d < _distance_threshold) or self.waypoint_index == -1:

            self._last_distance = None
            self.waypoint_index += 1
            if self.waypoint_index == len(self.poses):
                self.waypoint_index = 0


            # Advance waypoint target 1m in front of the gate
            waypoint = Point(
                self.poses[self.waypoint_index][0],
                self.poses[self.waypoint_index][1],
                self.poses[self.waypoint_index][2]
            )
            waypoint_vector = geo.vector_from_RPY(
                self.poses[self.waypoint_index][3],
                self.poses[self.waypoint_index][4],
                self.poses[self.waypoint_index][5]
            )
            advance_wp = Point(
                waypoint.x + waypoint_vector.x,
                waypoint.y + waypoint_vector.y,
                waypoint.z + waypoint_vector.z
            )

            '''    
            # Method 1. Speed vector is from actual position to gate
            waypoint = Point(
                self.poses[self.waypoint_index][0],
                self.poses[self.waypoint_index][1],
                self.poses[self.waypoint_index][2]
            )
            speed_vector = unit_vector(msg.position, waypoint)
            '''

            '''
            # Method 2. Speed vector is from previous waypoint to next waypoint
            last_waypoint_index = self.waypoint_index - 1
            if last_waypoint_index < 0: last_waypoint_index = 0
            last_waypoint = Point(
                self.poses[last_waypoint_index][0],
                self.poses[last_waypoint_index][1],
                self.poses[last_waypoint_index][2]
            )
            if self.waypoint_index > 0:
                speed_vector = unit_vector_two_points(last_waypoint, waypoint)
            else:
                speed_vector = unit_vector_two_points(msg.position, waypoint)
            '''

            '''
            # Method 3. Speed vector is the pose of the gate
            speed_vector = waypoint_vector
            '''


            # Method 4. Speed vector is from current waypoint to next waypoint
            next_waypoint_index = self.waypoint_index + 1
            if next_waypoint_index >= len(self.poses): next_waypoint_index = 0

            next_waypoint = Point(
                self.poses[next_waypoint_index][0],
                self.poses[next_waypoint_index][1],
                self.poses[next_waypoint_index][2]
            )

            speed_vector = geo.unit_vector_two_points(msg.position, next_waypoint)


            # Scale speed
            speed_vector.x *= _exit_speed
            speed_vector.y *= _exit_speed
            speed_vector.z *= _exit_speed


            self.command_pub.publish_pose_speed_command(poses[self.waypoint_index][0],
                                                  poses[self.waypoint_index][1],
                                                  poses[self.waypoint_index][2],
                                                  poses[self.waypoint_index][3],
                                                  poses[self.waypoint_index][4],
                                                  poses[self.waypoint_index][5],
                                                        speed_vector)
            '''
            self.command_pub.publish_pose_speed_command(advance_wp.x,
                                                        advance_wp.y,
                                                        advance_wp.z,
                                                        poses[self.waypoint_index][3],
                                                        poses[self.waypoint_index][4],
                                                        poses[self.waypoint_index][5],
                                                        speed_vector)
            '''


    def _callback(self, data):
        self.latest_msg = data
        self.new_msg_available = True



if __name__ == '__main__':

    _node_name = 'control_node'
    _namespace = 'firefly'

    print('* {} starting... '.format(_node_name), end="")

    rospy.init_node(_node_name, anonymous=True)



    '''
    path = [
        (2, 0, 2, 0), # x, y, z, yaw_deg
        (2, 2, 2, 0),
        (0, 2, 2, 0),
        (-2, 2, 2, 0),
        (-2, 0, 2, 0),
        (-2, -2, 2, 0),
        (0, -2, 2, 0)
    ]
    '''

    poses = [
        (-3.43295, 1.13137, 2.17771 + 0.5, 0.0, -0.0, 0.0 - math.pi), # x, y, z, Roll, Pitch, Yaw
        (-8.00466, 1.05556, 3.19768 + 0.5, 0.0, -0.0, 1.04754 - math.pi),
        (-8.86541, -3.8308, 3.72565 + 0.5, 0.0, -0.0, 1.59136 - math.pi),
        (-6.41389, -7.30975, 3.77515 + 0.5, 0.0, -0.0, 2.52009 - math.pi),
        (-0.679768, -6.62442, 1.404311 + 0.5, 0.0, 0.0, -3.1392 - math.pi),
        (5.69689, -7.06057, 3.2338 + 0.5, 0.0, 0.0, -2.59997 - math.pi),
        (8.19989, -2.97992, 2.32779 + 0.5, 0.0, 0.0, -1.85578 - math.pi),
        (8.85175, 3.19015, 2.36702 + 0.5, 0.0, 0.0, -1.49054 - math.pi),
        (7.68937, 7.67133, 2.65619 + 0.5, 0.0, 0.0, -0.205786 - math.pi),
        (3.63155, 6.28969, 3.84484 + 0.5, 0.0, -0.0, 0.69089 - math.pi)
    ]



    pub = WaypointController(_node_name, _namespace, poses)
    pub.start()

    '''
    pub = CommandPublisher('firefly')

    for x in range(3):
        for i in poses:
            pub.publish_pose_command(i[0], i[1], i[2], i[3], i[4], i[5])
            time.sleep(2)
            #pub.publish_point_command(i[0], i[1], i[2], i[3])
            #time.sleep(0.5)
    '''

    print('Ready.')

    rospy.spin()


