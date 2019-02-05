#!/usr/bin/env python
"""
# Ground Truth Gate Publisher
Load a ground truth gates file and publish:
1. Visualization topic into RViz
3. PoseArray topic of gate ground truth poses
"""

from __future__ import print_function
import rospy
import rospkg
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations
from geometry_msgs.msg import Point, Pose, PoseArray


import json

class PublisherViz(object):

    def __init__(self, _node_name, gate_objects):
        
        self.node_name = _node_name

        # Publisher topics
        _result_marker_topic = _node_name + '/gt_visualization'
        _result_pose_topic = _node_name + '/gt_gates'

        # Params
        self.frame_id = rospy.get_param("~frame_id", "/world")  # TODO: Document

        # Publishers
        self.marker_viz_pub = rospy.Publisher(_result_marker_topic, MarkerArray, queue_size=100)
        self.gate_pose_pub = rospy.Publisher(_result_pose_topic, PoseArray, queue_size=100)

        '''
        # Wait for at least 1 subscriber before publishing anything
        while self.marker_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            #rospy.logwarn_once("Waiting for a subscriber to the topic...")
            print(self.marker_pub.get_num_connections())
            rospy.logwarn("Waiting for a subscriber for: "+_result_marker_topic)
            rospy.sleep(1)
        '''

        # Iterate objects
        self.ground_truth_markers = MarkerArray()
        self.ground_truth_poses = PoseArray()

        self.ground_truth_poses.header.stamp = rospy.Time.now()
        self.ground_truth_poses.header.frame_id = self.frame_id

        for gate in gate_objects:

            if gate['marker_value'] == '10':  # Only add gates not landing spot

                # Notation from Gazebo World: Roll Pitch Yaw
                gate['quaternion'] = tf.transformations.quaternion_from_euler(float(gate['pose'][3]), float(gate['pose'][4]), float(gate['pose'][5]), 'rxyz')


                marker = self.create_rviz_object(gate, self.frame_id)
                pose = self.create_pose_object(gate)
                self.ground_truth_markers.markers.append(marker)
                self.ground_truth_poses.poses.append(pose)
                

            
        
        # Done
        #r.sleep()
        #rospy.loginfo("Gates published.")
        
        #rospy.spin()
        

        ## Rospy loop
        r = rospy.Rate(1)
        while not rospy.is_shutdown():

            self.marker_viz_pub.publish(self.ground_truth_markers)
            self.gate_pose_pub.publish(self.ground_truth_poses)

            r.sleep()

    def create_pose_object(self, gate):

        pose = Pose()

        pose.position.x = gate['pose'][0]
        pose.position.y = gate['pose'][1]
        pose.position.z = gate['pose'][2]

        pose.orientation.x = gate['quaternion'][0]
        pose.orientation.y = gate['quaternion'][1]
        pose.orientation.z = gate['quaternion'][2]
        pose.orientation.w = gate['quaternion'][3]

        return pose

    def create_rviz_object(self, gate, frame_id):
    
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        # marker.header.frame_id = "/map"
        marker.header.frame_id = frame_id
        marker.ns = self.node_name
        marker.id = gate['id']
        
        marker.type = marker.CUBE
        marker.action = marker.ADD
        
        # TODO: dims to params
        marker.scale.x = 0.025
        marker.scale.y = 1
        marker.scale.z = 1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.25
        
        marker.lifetime = rospy.Duration() # forever

        marker.pose.position.x = gate['pose'][0]
        marker.pose.position.y = gate['pose'][1]
        marker.pose.position.z = gate['pose'][2]

        marker.pose.orientation.x = gate['quaternion'][0]
        marker.pose.orientation.y = gate['quaternion'][1]
        marker.pose.orientation.z = gate['quaternion'][2]
        marker.pose.orientation.w = gate['quaternion'][3]
        
        return marker
        
        
         
if __name__ == '__main__' :

 
    _node_name = rospy.get_param("~node_name", default="gt_gates_publisher")

    print('* {} starting... '.format(_node_name), end="")
    rospy.init_node(_node_name, anonymous=True)
    
    # Put this everything somewhere else
    rospack = rospkg.RosPack()
    _gate_objects_file = rospy.get_param("~path_to_gates_file", rospack.get_path('drone_map_builder')+"/configs/test_track_gates.json")
    gate_objects = None
    with open(_gate_objects_file) as _rf:
        gate_objects = json.load(_rf)
    # ---
    
    PublisherViz(_node_name, gate_objects)

    print('Ready.')

    rospy.spin()
    
