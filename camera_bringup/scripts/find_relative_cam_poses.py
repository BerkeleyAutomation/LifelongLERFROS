#!/usr/bin/env python3
import numpy as np
import cv2

def construct_transformation_matrix(rvec, tvec):
    """
    Construct a 4x4 transformation matrix from a rotation vector and translation vector.
    """
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = tvec.flatten()
    return transformation_matrix

def rodrigues_inverse(rotated_vector, rotation_axis, theta):
    rotated_vector = np.array(rotated_vector)
    rotation_axis = np.array(rotation_axis)
    
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    original_vector = rotated_vector * cos_theta + np.cross(rotation_axis, rotated_vector) * sin_theta + \
                      rotation_axis * np.dot(rotation_axis, rotated_vector) * (1 - cos_theta)

    return original_vector

def compute_camera_transformation(rvec_A1, tvec_A1, rvec_B2, tvec_B2):
    """
    Compute the transformation matrix T_AB from the rotation and translation vectors.
    """
    
        #  this si the one that was measured originally
    # R_10 = np.array([[ 0.99948712, -0.0175596,  -0.02677979],
#  [ 0.01740421  ,0.99983038 ,-0.00602465],
#  [ 0.02688104,  0.00555548  ,0.9996232 ]])
    # t_10 = np.array([[ 2.64722008 , 0.04831344, -0.18714052]])

   
    # _, _, rotation_vector, *_ = cv2.RQDecomp3x3(R_10)

    # # Convert the rotation vector to Rodrigues vector
    # rvec_10, _ = cv2.Rodrigues(rotation_vector)
    # rvec_10 = rvec_10.flatten()

    # rvec_10 = np.array([-rvec_10[2], -rvec_10[0], rvec_10[1]])

    # R_10, _  = cv2.Rodrigues(rvec_10)

    # t_10 = np.array([[0.18714052,-2.64722008,0.04831344]])

    # more recently recorded version
    R_10 = np.array([[0.9980228, -0.05859092, 0.02275068],
                        [0.05864119, 0.99827792, -0.00154813],
                        [-0.02262079, 0.00287919, 0.99973997]])
    
    # t_10= np.array([[2.68020233, 0.04033166, -0.15284672]])
    t_10= np.array([[2.68020233, 0.04033166, -0.15284672]])

    # Construct the individual transformation matrices
    T_real0 = construct_transformation_matrix(rvec_A1, tvec_A1)
    T_10 = np.eye(4)
    T_10[:3, :3] = R_10
    T_10[:3,3] = t_10.flatten()
    T_left1 = construct_transformation_matrix(rvec_B2, tvec_B2)


    # T_10[:3, 0] = -R_10[:3,2]
    # T_10[:3, 1] = -R_10[:3,0]
    # T_10[:3, 2] = R_10[:3,1]

    # Compute T_AB
    T_realleft = T_real0 @ np.linalg.inv(T_10) @ np.linalg.inv(T_left1)


    left_arducam_matrix = np.array([[ 0.89625127,  0.04907124, -0.44082387,  0.57687621],
                                [ 0.01781156, -0.99704138, -0.07477459, -0.26084511],
                                [-0.44318892,  0.05916506, -0.89447363,  1.55624093],
                                [ 0.,          0.,          0.,          1.]])

    # Realsense on right aruco
    realsense_matrix = np.array([[ 0.30527018,  0.01522319,  0.95214409, -1.12351372],
                                [ 0.02631816, -0.99962515,  0.00754438, -0.29546134],
                                [ 0.95190203,  0.0227556,  -0.30555639,  3.05801331],
                                [ 0.,          0.,          0.,          1.]])
    
    aruco0_to_aruco1_tf = np.array([[ 0.9970716,   0.04015762 , 0.06508135 ,-2.73741648],
 [-0.04386139 , 0.99743771 , 0.05651719,  0.06753912],
 [-0.062645 ,  -0.05920624,  0.99627819 ,-0.02887055],
 [ 0.      ,    0.   ,       0.    ,      1.        ]])
    
    T_realleft = left_arducam_matrix @ np.linalg.inv(aruco0_to_aruco1_tf) @ np.linalg.inv(realsense_matrix)
    return T_realleft

# Example usage
# Define your rotation vectors (rvecs) and translation vectors (tvecs) for T_A1, T_12, T_B2

# rvec_real0 = np.array([[-2.87783496 ,-0.0364822 , -1.30865319]])
# tvec_real0 = np.array([[0.10027268 ,-0.28212355 , 1.75048375]])

rvec_real0 = np.array([[-2.9084428, -0.04394755, -1.30172324]])
tvec_real0 = np.array([[-1.1817357, -0.18745572, 2.57591312]])


# rvec_left1 = np.array([[-2.91290936 ,-0.06046243,  1.11711058]])
# tvec_left1 = np.array([[-0.20480061, -0.35622574,  1.59488601]])

rvec_left1 = np.array([[[2.90915968, 0.08407114, -1.09370135]]])
tvec_left1 = np.array([[0.54987041, -0.29770925, 2.85064408]])



# Compute T_AB
T_AB = compute_camera_transformation(rvec_real0, tvec_real0, rvec_left1, tvec_left1)
print("Transformation matrix T_AB:\n", T_AB)