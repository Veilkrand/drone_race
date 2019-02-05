#!/usr/bin/env python
"""
Publish ground truth drone position in Rviz

"""


from __future__ import print_function

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose


class ReusableIdGenerator:

    def __init__(self, number):
        self.index = -1
        self.number = number

    def get_id(self):

        self.index += 1
        if self.index == self.number: self.index = 0

        return self.index

class SubscriberPublisher:

    def __init__(self, node_name):

        self.node_name = node_name

        # Params

        self.parent_frame_id = rospy.get_param("~parent_frame_id", default='base_link')

        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", default='world')

        self._input_topic_ego_pose = '/firefly/odometry_sensor1/pose'


        self.ego_pose_sub = rospy.Subscriber(self._input_topic_ego_pose, Pose, self._ego_pose_callback, queue_size=1)

        # Publishers

        _ouput_topic_trajectory_viz = node_name + '/trajectory_viz_markers'
        self.ego_trajectory_viz = rospy.Publisher(_ouput_topic_trajectory_viz, Marker, queue_size=100)

        # Ids gen
        self.id_gen = ReusableIdGenerator(500)

        ## Rospy loop
        r = rospy.Rate(5)  # 10 Hz
        while not rospy.is_shutdown():

            # Publish Map
            r.sleep()

    def _ego_pose_callback(self, data):
        pose = self.create_marker_line_object(data, self.fixed_frame_id)
        #print(pose.id)
        self.ego_trajectory_viz.publish(pose)

    def create_marker_line_object(self, pose, frame_id):

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame_id
        marker.ns = self.node_name
        marker.id = self.id_gen.get_id()

        #marker.type = marker.LINE_STRIP
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(12)

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.15

        marker.pose = pose

        return marker


if __name__ == '__main__':

    _node_name = 'gt_ego_position'

    print('* {} starting... '.format(_node_name), end="")

    rospy.init_node(_node_name, anonymous=True)

    SubscriberPublisher(_node_name)

    print('Ready.')

    rospy.spin()
