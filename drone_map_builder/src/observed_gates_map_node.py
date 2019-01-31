#!/usr/bin/env python
"""
Build a map of observed markers relative to the camera.
Pipeline
1. Subscribe detected markers pose
2. Subscribe ego pose in time
3. Transform markers local pose to map pose
4. Data association: Identify unique gates by Weighted nearest neighbour (euclidean distance threshold?)
4.5. Correct flipping Z axis with a median moving window?
5. Update pose estimation per observed gate. Lineal KF
6. Publish results as markers for visualization and topic for Planning
7. RMSE Error calculation with ground truth

"""


from __future__ import print_function

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseArray, Pose  #, Point32
import tf.transformations


class ReusableIdGenerator:

    def __init__(self, number):
        self.index = -1
        self.number = number

    def get_id(self):

        self.index += 1
        if self.index == self.number: self.index = 0

        return self.index

class SubscriberPublisherGates:

    def __init__(self, node_name):

        self.node_name = node_name

        # Params
        self.input_markers_topic = rospy.get_param("~input_markers_topic", default='markertracker_node/markers')
        self.parent_frame_id = rospy.get_param("~parent_frame_id", default='base_link')

        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", default='world')

        self._input_topic_ego_pose = '/firefly/odometry_sensor1/pose'

        # Subscribers
        #self.markers_sub = rospy.Subscriber(_input_image_topic, Image, self._markers_callback, queue_size=100)
        #self.tf_listener = tf.TransformListener()

        self.ego_pose_sub = rospy.Subscriber(self._input_topic_ego_pose, Pose, self._ego_pose_callback, queue_size=1)

        # Publishers
        #self.gates_map_pub = rospy.Publisher(_result_poses_topic, MarkersArray, queue_size=100)

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

    def _markers_callback(self, data):

        if len(data.marker) <= 0:
            return None

        stamp = data.camera_frame_stamp  # For synchronizing TF

        if self.tf_listener.waitForTransform(self.fixed_frame_id, self.parent_frame_id, stamp, rospy.Duration(0.1)):

            world_pose_array = MarkerArray()

            for m in data.marker:

                pose = m.marker.pose_cov_stamped

                world_pose = self.tl.transformPose(self.fixed_frame_id, pose)


if __name__ == '__main__':

    _node_name = 'gates_map_node'

    print('* {} starting... '.format(_node_name), end="")

    rospy.init_node(_node_name, anonymous=True)

    SubscriberPublisherGates(_node_name)

    print('Ready.')

    rospy.spin()
