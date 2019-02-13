#!/usr/bin/env python
"""
Build a map of observed markers relative to the camera.
Pipeline
1. Subscribe detected markers pose
2. Subscribe ego pose in time
3. Transform markers local pose to world pose
4. Data association: Identify unique gates by Weighted nearest neighbour (euclidean distance threshold?)
4.5. Correct flipping Z axis with a median moving window?
5. Update pose estimation per observed gate. Lineal KF
6. Publish results as markers for visualization and topic for Planning
7. RMSE Error calculation with ground truth

TODO:
 - Include new message array with confidence per gate by numbers of observations per period
 - Use real ego pose covariances to compute KF
 - Weighted median average for low confidence gates

"""


from __future__ import print_function

import rospy

from visualization_msgs.msg import Marker, MarkerArray
from markertracker_node.msg import GateMarker, GateMarkersArray
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseArray, Pose  #, Point32

import tf2_ros as tf2
import math
#import tf.transformations


class Gate:

    def __init__(self, _id):

        # TODO: Move to params
        self.observations_size = 75  # Only the most relevant observations are kept

        # Put better ones at top, worse ones at the button
        self.observations = list()

        self.total_observations = 0
        self.confidence = 0.0

        self.id = _id
        self.name = 'Gate'+str(_id)

        print('*** New gate:' + self.name)

    def add_marker_observation(self, marker):

        self.total_observations += 1

        observation = {}
        observation['pose'] = marker.pose_cov_stamped.pose.pose.pose
        observation['covariance'] = marker.pose_cov_stamped.pose.covariance
        _area = self._area_of_quadrilateral(marker.corners)
        observation['weight'] = _area

        if len(self.observations) >= self.observations_size:  # start popping
            if _area < self.observations[-1]['weight']:
                return
            else:
                self.observations.pop(-1)

        self.observations.append(observation)

        self.observations = sorted(self.observations, key=lambda o: o['weight'], reverse=True)

        self.confidence = self._calc_confidence()

        #print(self.observations[0])

    def _calc_confidence(self):
        # [0,1] Confidence based on observations to be sure about a gate is real
        #confidence = self.total_observations / self.observations_size

        confidence = len(self.observations) / float(self.observations_size)
        if confidence > 1.0: confidence = 1.0

        return confidence

    def get_best_pose_estimation(self):

        pose = self.observations[0]['pose']

        return pose

    def _area_of_quadrilateral(self, corners):
        a = math.sqrt((corners[0] - corners[2]) ** 2 + (corners[1] - corners[3]) ** 2)
        b = math.sqrt((corners[2] - corners[4]) ** 2 + (corners[3] - corners[5]) ** 2)
        c = math.sqrt((corners[4] - corners[6]) ** 2 + (corners[5] - corners[7]) ** 2)
        d = math.sqrt((corners[6] - corners[0]) ** 2 + (corners[7] - corners[1]) ** 2)
        return (a + b + c + d) / 2

    def euclidean_distance_observation(self, marker):
        point1 = self.observations[0]['pose'].position
        point2 = marker.pose_cov_stamped.pose.pose.pose.position
        return math.sqrt((point2.x - point1.x) ** 2 + (point2.y - point1.y) ** 2 + (point2.z - point1.z) ** 2)


class GateMap:

    def __init__(self):
        self.gates = list()
        # TODO: move to params
        self.d_threshold = 3  # cluster observation when closer than threshold by euclidean distance in meters

    def process_marker_observation(self, marker):

        # First observation

        if len(self.gates) == 0:
            gate = Gate(1)
            gate.add_marker_observation(marker)
            self.gates.append(gate)
            return

        for g in self.gates:

            print(g.name, g.total_observations, g.confidence)

            if g.euclidean_distance_observation(marker) < self.d_threshold:
                g.add_marker_observation(marker)
                return

        # Create a new gate in the map
        gate = Gate(len(self.gates)+1)
        gate.add_marker_observation(marker)
        self.gates.append(gate)




class SubscriberPublisherObservations:

    def __init__(self, node_name):

        self.node_name = node_name

        # Params
        _input_markers_topic = rospy.get_param("~input_markers_topic", default='/markertracker_node/gate_markers')
        self.camera_frame_id = rospy.get_param("~parent_frame_id", default='firefly/vi_sensor/base_link')
        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", default='world')

        # Subscribers
        self.markers_sub = rospy.Subscriber(_input_markers_topic, GateMarkersArray, self._markers_callback, queue_size=100)
        self.tf_buffer = tf2.Buffer()
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

        # To store the gate map
        self.gates_map = GateMap()


        # Publishers
        self.poses_pub = rospy.Publisher(node_name + '/poses', PoseArray, queue_size=100)
        self.viz_markers_pub = rospy.Publisher(node_name + '/visualization_markers', MarkerArray, queue_size=100)

        ## Rospy loop
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            #self.marker_viz_pub.publish(self.ground_truth_markers)
            #self.gate_pose_pub.publish(self.ground_truth_poses)

            # Publish best map estimations
            self._build_gate_messages_and_publish()

            r.sleep()



    def _build_gate_messages_and_publish(self):

        #TODO: move to params
        _confidence_threshold = 0.05  # Ignore gates with lower confidence than threshold

        if len(self.gates_map.gates) == 0: return False

        marker_viz_array = MarkerArray()
        pose_array = PoseArray()
        pose_array.header.frame_id = self.fixed_frame_id

        for g in self.gates_map.gates:

            if g.confidence < _confidence_threshold: continue

            pose = g.get_best_pose_estimation()
            gate_id = g.id

            pose_array.poses.append(pose)

            marker_msg = self._create_gate_rviz_marker(pose, gate_id, self.fixed_frame_id)
            _str_label = "{} {:.0f}%".format(g.name, g.confidence*100)
            label_msg = self._create_gate_rviz_label(pose, gate_id, _str_label, self.fixed_frame_id)
            marker_viz_array.markers.append(label_msg)
            marker_viz_array.markers.append(marker_msg)


        self.poses_pub.publish(pose_array)
        self.viz_markers_pub.publish(marker_viz_array)


    def _create_gate_rviz_marker(self, pose, gate_id, frame_id):

        marker = Marker()
        marker.header.stamp = rospy.Time.now()

        marker.header.frame_id = frame_id
        marker.ns = self.node_name + '/gate_marker'
        marker.id = gate_id

        marker.type = marker.CUBE
        marker.action = marker.ADD

        # TODO: dims to params
        marker.scale.x = 0.025
        marker.scale.y = 1
        marker.scale.z = 1

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5

        marker.lifetime = rospy.Duration()  # forever

        marker.pose = pose

        return marker

    def _create_gate_rviz_label(self, pose, gate_id, text, frame_id):

        marker = Marker()
        marker.header.stamp = rospy.Time.now()

        marker.header.frame_id = frame_id
        marker.ns = self.node_name + '/gate_label'
        marker.id = gate_id

        marker.text = text
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD

        # TODO: dims to params
        #marker.scale.x = 0.025
        #marker.scale.y = 1
        marker.scale.z = 0.5

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.75

        marker.lifetime = rospy.Duration()  # forever

        marker.pose = pose

        return marker


    def _markers_callback(self, data):

        if len(data.marker) <= 0:
            return None

        stamp = data.camera_frame_stamp  # For synchronizing TF

        try:
            camera_tf = self.tf_buffer.lookup_transform(self.fixed_frame_id, self.camera_frame_id, stamp, rospy.Duration(0.1))
            #print('camera_tf', camera_tf)
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
            return None

        # world_pose_array = MarkersArray()

        for m in data.marker:
            pose = m.pose_cov_stamped.pose
            #print(pose)
            pose_stamped_world = do_transform_pose(pose, camera_tf)
            m.pose_cov_stamped.pose.pose = pose_stamped_world
            self.gates_map.process_marker_observation(m)

            #print(pose_stamped_world)





if __name__ == '__main__':

    _node_name = 'gates_map_node'

    print('* {} starting... '.format(_node_name), end="")

    rospy.init_node(_node_name, anonymous=True)

    SubscriberPublisherObservations(_node_name)

    print('Ready.')

    rospy.spin()
