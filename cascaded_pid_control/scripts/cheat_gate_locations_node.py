#!/usr/bin/python
import rospy
import rospkg
import yaml
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseArray, Pose
import tf


class PublisherViz(object):
    """
    This is basically the same `PublisherViz` from `drone_map_builder` node (with some changes)
    """

    @staticmethod
    def vec_to_quat(vec):
        """To fully determine the orientation represented by the resulting quaternion, this method will assume the top of objects would always facing up
        """
        np_vec = vec
        norm = np.linalg.norm(np_vec)
        if norm < 1e-5:
            return np.array([0.0, 0.0, 0.0, 1.0])
        obj_x = np_vec / norm
        obj_z_t = np.array([0.0, 0.0, 1.0])
        obj_y = np.cross(obj_z_t, obj_x)
        obj_y /= np.linalg.norm(obj_y) 
        obj_z = np.cross(obj_x, obj_y)
        rot_mat = np.identity(4)
        rot_mat[:3,:3] = np.array([obj_x,
                                  obj_y,
                                  obj_z]).T
        q = tf.transformations.quaternion_from_matrix(rot_mat)
        return q / np.linalg.norm(q)


    def __init__(self, gate_objects):
            # Publisher topics
        _result_marker_topic = '~gt_visualization'
        _result_pose_topic = '~gt_gates'

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
            gate['quaternion'] = PublisherViz.vec_to_quat(gate['orientation'])

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
  rospy.init_node("gt_gates_publisher")

  rospack = rospkg.RosPack()
  gate_locations_file = rospy.get_param("~gate_locations_definition", rospack.get_path('flightgoggles')+"/config/challenges/gate_locations.yaml")
  gate_locations = {}
  with open(gate_locations_file) as f:
    all_gates = yaml.load(f)
    for k in all_gates.keys():
      value = all_gates[k]
      loc = np.array(value['location'])
      norms = [np.linalg.norm(loc[1] - loc[0]),
               np.linalg.norm(loc[2] - loc[0]),
               np.linalg.norm(loc[3] - loc[0])]
      # a simple strategy for determining center of the gate is,
      # use the center of the gate corner
      idx = np.argmax(norms)
      center = (loc[idx + 1] + loc[0]) / 2
      # orientation of the gate is deteremined by the course
      # connect consecutive all_gates. the resulting vector is then can be 
      # used as "un-normalized" orientation.
      gate_locations[k] = {'corners': loc, 'center': center, 'orientation': np.cross(loc[1] - loc[0], loc[3] - loc[0])}

  gates = rospy.get_param("/uav/gate_names", None)
  if gates is None:
    rospy.logwarn("Unable to load course definition from /uav/gate_names")
    exit(-1)

  course = list(int(g.replace("Gate", ""), 10) for g in gates)
  rospy.set_param("/uav/gate_names", list("Gate{}".format(c) for c in course))
  print("Course gate sequence: {}".format(course))

  course_gates = []
  g1_idx = None
  g2_idx = None
  for idx in range(len(course) - 1):
    g1_idx = 'Gate' + str(course[idx])
    g2_idx = 'Gate' + str(course[idx + 1])
    g1_loc = gate_locations[g1_idx]['center']
    g2_loc = gate_locations[g2_idx]['center']
    g1_to_g2 = g2_loc - g1_loc
    g1_orientation = gate_locations[g1_idx]['orientation']
    if np.dot(g1_to_g2, g1_orientation) >= 0:
      course_gates.append({'id': course[idx], 'pose': g1_loc, 'orientation': g1_orientation})
    else:
      course_gates.append({'id': course[idx], 'pose': g1_loc, 'orientation': -g1_orientation})

  if g2_idx is not None:
    # only one gate
    g2_orientation = gate_locations[g2_idx]['orientation']
    if np.dot(g1_to_g2, g2_orientation) >= 0:
      course_gates.append({'id': course[-1], 'pose': g2_loc, 'orientation': g2_orientation})
    else:
      course_gates.append({'id': course[-1], 'pose': g2_loc, 'orientation': -g2_orientation})
  else:
    gt_idx = 'Gate' + str(course[0])
    course_gates.append({'id': course[0], 'pose': gate_locations[gt_idx]['center'], 'orientation': gate_locations[gt_idx]['orientation']})
  print(course_gates)
  
  publisherViz = PublisherViz(course_gates)
  
  rospy.spin()