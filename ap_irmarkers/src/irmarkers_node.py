#!/usr/bin/env python
"""
##


"""

from __future__ import print_function
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations
from geometry_msgs.msg import Point, Pose, PoseArray
from flightgoggles.msg import IRMarker, IRMarkerArray

from MarkersEstimator import MarkersEstimator

class SubscriberPublisher(object):

    def __init__(self, _node_name):

        self.node_name = _node_name

        # Publisher topics
        _markers_topic = _node_name + '/rviz_markers'
        #_result_pose_topic = _node_name + '/gt_gates'

        # Params
        #self.frame_id = rospy.get_param("~frame_id", "/world")  # TODO: Document
        _input_topic_irmarkers = rospy.get_param("~topic_irmarkers", "/uav/camera/left/ir_beacons")

        # Publishers
        self.marker_viz_pub = rospy.Publisher(_markers_topic, MarkerArray, queue_size=100)
        #self.gate_pose_pub = rospy.Publisher(_result_pose_topic, PoseArray, queue_size=100)

        # Subscriber
        self.ego_pose_sub = rospy.Subscriber(_input_topic_irmarkers, IRMarkerArray, self._irmarker_callback, queue_size=1)

        # Objects
        self.gates = {}
        # Test track
        self.track = ['Gate2']
        # Pose estimator
        self.estimator = MarkersEstimator()


    def _irmarker_callback(self, data):

        viz_marker_array = MarkerArray()

        for m in data.markers:

            if m.landmarkID.data not in self.gates:
                self.gates[m.landmarkID.data] = {}

            self.gates[m.landmarkID.data][m.markerID.data] = (m.x, m.y)

            # self._print_gates_stats()

            # if m.landmarkID.data in self.track:
            #     print('Waypoint detected')
            #     self.estimator.estimate_gate_markers(self.gates[m.landmarkID.data])

            #print(m.landmarkID.data)
            #print(self.gates[m.landmarkID.data])

            result = self.estimator.estimate_gate_markers( self.gates[m.landmarkID.data] )

            if result is not None:
                print(m.landmarkID.data, result)
                pose = Pose()
                pose.position.x = result['position'][0]
                pose.position.y = result['position'][1]
                pose.position.z = result['position'][2]

                # _q = tf.transformations.quaternion_from_euler(result['rpy'][0], result['rpy'][1], result['rpy'][2])
                # pose.orientation.x = _q[0]
                # pose.orientation.y = _q[1]
                # pose.orientation.z = _q[2]
                # pose.orientation.w = _q[3]


                _id = int(m.landmarkID.data[4:]) # Only the numeral of Gate00
                marker = self._create_gate_rviz_marker(pose, _id, 'world')

                viz_marker_array.markers.append(marker)

        # Publish results
        self.marker_viz_pub.publish(viz_marker_array)



    def _print_gates_stats(self):

        print('***',len(self.gates),' detected gates.')

        for key, val in self.gates.items():

            print('    ', key, ':', len(val))

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
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.15

        marker.lifetime = rospy.Duration(0.5)  # forever

        marker.pose = pose

        return marker



if __name__ == '__main__':
    _node_name = rospy.get_param("~node_name", default="irmarkers_node")

    print('* {} starting... '.format(_node_name), end="")
    rospy.init_node(_node_name, anonymous=True)

    SubscriberPublisher(_node_name)

    print('Ready.')

    rospy.spin()