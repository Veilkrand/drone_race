"""
## IR Markers message id position:

[3]----[4]
 |      |
 |      |
[2]----[1]

[1] (1,1,0)
[2] (-1,1,0)
[3] (-1,-1,0)
[4] (-1,1,0)

Gate width is 0.3 m

## Left Camera info:

height: 768
width: 1024
distortion_model: "plum_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [548.4088134765625, 0.0, 512.0, 0.0, 548.4088134765625, 384.0, 0.0, 0.0, 1.0]
R: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
P: [548.4088134765625, 0.0, 512.0, 0.0, 0.0, 548.4088134765625, 384.0, 0.0, 0.0, 0.0, 1.0, 0.0]

"""
import numpy as np
import cv2
import math
from geometry_msgs.msg import Point, Pose, PoseArray

class MarkersEstimator:

    def __init__(self):

        # From camera info K and D
        self.mtx = np.array([ [548.4088134765625, 0.0, 512.0], [0.0, 548.4088134765625, 384.0], [0.0, 0.0, 1.0]], dtype = "double")
        self.dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype = "double")

        self.gate_w = 30

    def estimate_gate_markers(self, gate):

        #print(gate)

        #pts = np.empty(shape=(0,1))
        pts = []


        for g in gate:
            pts.append(gate[g])

        #print(pts)
        # Find the rotation and translation vectors.

        if len(pts) == 4:
            pts = np.array(pts, dtype="double").reshape(4, 1, 2) #it needs to be reshaped for Ransac

            # print(pts)
            objp = np.array([ (self.gate_w,self.gate_w,0),
                              (-self.gate_w,self.gate_w,0),
                              (-self.gate_w,-self.gate_w,0),
                              (-self.gate_w,self.gate_w,0) ], dtype="double").reshape(4, 1, 3) #it needs to be reshaped for Ransac
            # print(objp)
            rval, rvec, tvec, inliers = cv2.solvePnPRansac(objp, pts, self.mtx, self.dist)
            #rvecs, tvecs, inliers = cv2.solvePnP(objp, pts, self.mtx, self.dist)

            if rval is False:
                return None

            rpy = rvec2rpy_ros(rvec)
            pos = tvec2point_ros(tvec.flatten())
            return {'rpy' : rpy,
                    'position' : pos}





def tvec2point_ros(tvec):

    pos = (tvec.item(2) / 100, -tvec.item(0) / 100, -tvec.item(1) / 100)  # z, -x, -y

    return pos

'''
/** this conversion uses conventions as described on page:
*   https://www.euclideanspace.com/maths/geometry/rotations/euler/index.htm
*   Coordinate System: right hand
*   Positive angle: right hand
*   Order of euler angles: heading first, then attitude, then bank
*   matrix row column ordering:
*   [m00 m01 m02]
*   [m10 m11 m12]
*   [m20 m21 m22]*/
'''
def rvec2rpy_ros(rvec):

    m, _ =  cv2.Rodrigues(rvec)

    # // Assuming the angles are in radians.
    if (m[1, 0] > 0.998):  # // singularity at north pole
        yaw = math.atan2(m[0, 2], m[2, 2])
        roll = math.PI / 2
        pitch = 0
    elif m[1, 0] < -0.998:  # // singularity at south pole
        yaw = math.atan2(m[0, 2], m[2, 2])
        roll = -math.PI / 2
        pitch = 0

    else:
        yaw = -math.atan2(-m[2, 0], m[0, 0]) + math.pi
        pitch = math.atan2(m[2, 2], m[1, 2]) + math.pi / 2  # math.atan2(-m[1, 2], m[1, 1])
        roll = -math.asin(m[1, 0])

    return roll, pitch, yaw
