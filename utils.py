import math
import numpy as np

# Function from https://stackoverflow.com/questions/21030391/how-to-normalize-an-array-in-numpy
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

# Function from https://stackoverflow.com/questions/39000758/how-to-multiply-two-quaternions-by-python-or-numpy
def multiply_quaternions(quaternion1, quaternion2):
    return np.array([[quaternion1[3] * quaternion2[0] + quaternion1[0] * quaternion2[3] + quaternion1[1] * quaternion2[2] - quaternion1[2] * quaternion2[1]],
                     [quaternion1[3] * quaternion2[1] - quaternion1[0] * quaternion2[2] + quaternion1[1] * quaternion2[3] + quaternion1[2] * quaternion2[0]],
                     [quaternion1[3] * quaternion2[2] + quaternion1[0] * quaternion2[1] - quaternion1[1] * quaternion2[0] + quaternion1[2] * quaternion2[3]],
                     [quaternion1[3] * quaternion2[3] - quaternion1[0] * quaternion2[0] - quaternion1[1] * quaternion2[1] - quaternion1[2] * quaternion2[2]]], dtype=np.float64)

# Function from https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
def quat_to_axis_angle(quaternion):
	quat = np.array(quaternion)

	if(quat[3] > 1):
		quat = normalize(quat)

	angle = 2 * math.acos(quat[3])
	s = math.sqrt(1 - quat[3] * quat[3])
	if (s < 0.001):
		x = quat[0]
		y = quat[1]
		z = quat[2]
	else:
		x = quat[0] / s
		y = quat[1] / s
		z = quat[2] / s

	axis = np.array([x, y, z])
	normalized_axis = normalize(axis)
	axis_angle = angle * normalized_axis

	return axis_angle

# Function from https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
def axis_angle_to_quaternion(axis_angle):
	axis_angle = np.array(axis_angle)
	angle = np.linalg.norm(axis_angle)
	axis = normalize(axis_angle)

	s = math.sin(angle / 2)

	x = axis[0] * s
	y = axis[1] * s
	z = axis[2] * s
	w = math.cos(angle / 2)

	return [x,y,z,w]

def rotate_around_axis(axis_angle, axis, angle):
	quaternion1 = axis_angle_to_quaternion(-np.array(axis_angle))
	axis_angle2 = angle * normalize(axis)
	quaternion2 = axis_angle_to_quaternion(-np.array(axis_angle2))
	final_quaternion = multiply_quaternions(quaternion1,quaternion2)
	final_axis_angle = -quat_to_axis_angle(final_quaternion)
	return final_axis_angle