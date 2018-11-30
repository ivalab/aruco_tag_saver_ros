import sys
import os
import time
import socket
import struct
import freenect

sys.path.append('./')
import cv2
import numpy as np
import threading
import cv2.aruco as aruco
import math
import tf
from shapely.geometry import Polygon


# CHANGE-ABLE PARAMETERS
OBJECT_TYPE = "scoop"
CAM_POSE = 0
OBJ_POSE = 0
START_COUNT = 0
OBJ_POSE_ONLY = True
CAMERA_DIM = (640, 480)

tmpPathGT_pose = './data/graspPositions.txt'
tmpPathGT_ar = './data/arucoPositions.txt'
tmpPathGT = './data/graspPoseLabels.txt'


# UNIVERSAL PARAMETERS
IMG_SAVEPATH = os.path.join("./data/", OBJECT_TYPE, str(CAM_POSE), str(OBJ_POSE))

# CEREATE path IF NOT EXISTS
if not os.path.exists(IMG_SAVEPATH):
    os.makedirs(IMG_SAVEPATH)

cameraMatrix = np.array([[524.82863, 0.0,  348.39958],
                         [0.0, 525.53556, 228.20485],
                         [0.0, 0.0, 1.0]])
distCoeffs = np.array([0.16950, -0.20215, -0.02522, 0.02576, 0.0])

default_M_CL = ([8.05203923e-04,  -9.98258274e-01,  5.89895796e-02, 2.11182116e-02],
                [-7.25197650e-01, -4.11996435e-02, -6.87307033e-01, 1.19383476e-01],
                [6.88540282e-01,  -4.22256822e-02, -7.23967728e-01, 5.68361874e-01],
                [0.00000000e+00,   0.00000000e+00,  0.00000000e+00, 1.00000000e+00])

##id:3
x1 = 0.0569
y1 = 0.0
z1 = -0.13275
R1 = 3.1416
P1 = 3.2279E-18
Y1 = -1.5708
##id:4
x2 = -0.0556
y2 = 0.0
z2 = -0.13275
R2 = -3.1416
P2 = 1.027E-15
Y2 = 1.249E-15
##id:5
x3 = -0.0221
y3 = 0.04025
z3 = -0.13275
R3 = 3.1416
P3 = -1.027E-15
Y3 = -3.1416

M_Yflip = np.zeros((4, 4))
M_Yflip[:3, :3] = np.array([[-1,0,0],[0,1,0],[0,0,-1]])
M_Yflip[:3, 3] = np.array((0.0, 0.0, 0.0))
M_Yflip[3, :] = np.array([0, 0, 0, 1])

PI = 3.1415
ARM_BASE_HEIGHT = 0.00
GRIPPER_DEPTH = 0.05
GRIPPER_WIDTH = 0.035
GRASP_WIDTH = 0.13



# function to get depth image from kinect
def get_depth():
    array_raw, _ = freenect.sync_get_depth()
    array = array_raw.astype(np.uint8)
    return array, array_raw


def grasp_check(pose_grasp, rot_angle_grasp, pose_regrasp, rot_angle_regrasp):
    absAngle = abs((rot_angle_grasp - rot_angle_regrasp) / PI * 180.0)
    if not (absAngle < 30.0 or absAngle > 330 or (absAngle > 150 and absAngle < 210) ) :
        print('rotate problem, ungraspable')
        return False
    else:
        if abs(pose_grasp[2] - ARM_BASE_HEIGHT - pose_regrasp[2]) > (0.3 * GRIPPER_DEPTH):
            print('grasp height problem, ungraspable: ' + str(abs(pose_grasp[2] - ARM_BASE_HEIGHT - pose_regrasp[2])))
            return False
        else:
           p1, p2 = pose_2_rect(pose_grasp, rot_angle_grasp, pose_regrasp, rot_angle_regrasp)
           if IoU(p1, p2) < 0.25:
               print('IoU problem, ungraspable')
               return False
           else:
               print('graspable')
               return True

def pose_2_rect(pose_grasp, rot_angle_grasp, pose_regrasp, rot_angle_regrasp):
       center_x_grasp = pose_grasp[0]
       center_y_grasp = pose_grasp[1]
       center_z_grasp = pose_grasp[2]

       center_x_regrasp = pose_regrasp[0]
       center_y_regrasp = pose_regrasp[1]
       center_z_regrasp = pose_regrasp[2]

       x = GRIPPER_WIDTH / 2
       y = GRASP_WIDTH / 2
       diagonal = math.sqrt(x * x + y * y)
       theta = math.atan(x / y)
       theta_1_grasp = theta + rot_angle_grasp
       theta_2_grasp = -theta + rot_angle_grasp

       p1_1 = (
       math.sin(theta_1_grasp) * diagonal + center_x_grasp, math.cos(theta_1_grasp) * diagonal + center_y_grasp)
       p1_2 = (
       math.sin(theta_2_grasp) * diagonal + center_x_grasp, math.cos(theta_2_grasp) * diagonal + center_y_grasp)
       p1_3 = (-math.sin(theta_1_grasp) * diagonal + center_x_grasp,
               -math.cos(theta_1_grasp) * diagonal + center_y_grasp)
       p1_4 = (-math.sin(theta_2_grasp) * diagonal + center_x_grasp,
               -math.cos(theta_2_grasp) * diagonal + center_y_grasp)
       p1 = Polygon([p1_1, p1_2, p1_3, p1_4])

       theta_1_regrasp = theta + rot_angle_regrasp
       theta_2_regrasp = -theta + rot_angle_regrasp
       p2_1 = (math.sin(theta_1_regrasp) * diagonal + center_x_regrasp,
               math.cos(theta_1_regrasp) * diagonal + center_y_regrasp)
       p2_2 = (math.sin(theta_2_regrasp) * diagonal + center_x_regrasp,
               math.cos(theta_2_regrasp) * diagonal + center_y_regrasp)
       p2_3 = (-math.sin(theta_1_regrasp) * diagonal + center_x_regrasp,
               -math.cos(theta_1_regrasp) * diagonal + center_y_regrasp)
       p2_4 = (-math.sin(theta_2_regrasp) * diagonal + center_x_regrasp,
               -math.cos(theta_2_regrasp) * diagonal + center_y_regrasp)
       p2 = Polygon([p2_1, p2_2, p2_3, p2_4])

       return p1, p2

###compute the percentage of intersection area from the union area
def IoU(p1, p2):
       intersection = p1.intersection(p2)
       area_inter = intersection.area
       union = p1.union(p2)
       area_union = union.area

       return area_inter / area_union


def get_M_CL(gray, image_init, visualize=False):
    # parameters
    markerLength_CL = 0.06
    aruco_dict_CL = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)  # DICT_ARUCO_ORIGINAL
    parameters = aruco.DetectorParameters_create()

    corners_CL, ids_CL, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict_CL, parameters=parameters)

    # for the first frame, it may contain nothing
    if ids_CL is None:
        return default_M_CL

    rvec_CL, tvec_CL, _objPoints_CL = aruco.estimatePoseSingleMarkers(corners_CL[0], markerLength_CL,
                                                                      cameraMatrix, distCoeffs)
    dst_CL, jacobian_CL = cv2.Rodrigues(rvec_CL)
    M_CL = np.zeros((4, 4))
    M_CL[:3, :3] = dst_CL
    M_CL[:3, 3] = tvec_CL
    M_CL[3, :] = np.array([0, 0, 0, 1])

    if visualize:
        aruco.drawAxis(image_init, cameraMatrix, distCoeffs, rvec_CL, tvec_CL, markerLength_CL)
    return M_CL


def get_M_Cm0(gray, image_init, visualize=False):
    # parameters
    markerLength = 0.0635
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    M_Cm0 = np.zeros((4, 4))
    if ids is not None:
        rvec_Cm0, tvec_Cm0, _objPoints_Cm0 = aruco.estimatePoseSingleMarkers(corners[0], markerLength,
                                                                             cameraMatrix, distCoeffs)
        dst_Cm0, jacobian = cv2.Rodrigues(rvec_Cm0)
        M_Cm0[:3, :3] = dst_Cm0
        M_Cm0[:3, 3] = tvec_Cm0
        M_Cm0[3, :] = np.array([0, 0, 0, 1])

        # re-position to ids == 7 for 8,9 and 10
        M_mxm0 = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        if ids[0] == 8:
            M_mxm0 = np.array([[1,0,0,-0.1205],
                               [0,1,0, 0.000],
                               [0,0,1, 0.000],
                               [0,0,0, 1]])
        elif ids[0] == 9:
            M_mxm0 = np.array([[1,0,0, 0.000],
                               [0,1,0, 0.1715],
                               [0,0,1, 0.000],
                               [0,0,0, 1]])
        elif ids[0] == 10:
            M_mxm0 = np.array([[1,0,0,-0.1205],
                               [0,1,0, 0.1715],
                               [0,0,1, 0.000],
                               [0,0,0, 1]])
        M_Cm0 = np.dot(M_Cm0, M_mxm0)

    else:
        print('m0 not detected!')

    #TODO: get 4 works
    if visualize and ids is not None:
        aruco.drawAxis(image_init, cameraMatrix, distCoeffs, rvec_Cm0, tvec_Cm0, markerLength)
    return M_Cm0

def get_M_m0mP(object_type):
    if object_type == 'scoop':
        M_m0mP1 = np.zeros((4, 4))
        M_m0mP1[:3, :3] = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        M_m0mP1[:3, 3] = np.array((0.06025, -0.08475, 0.000))
        M_m0mP1[3, :] = np.array([0, 0, 0, 1])

        M_m0mP2 = np.zeros((4, 4))
        M_m0mP2[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        M_m0mP2[:3, 3] = np.array((0.11, -0.10, 0.010))
        M_m0mP2[3, :] = np.array([0, 0, 0, 1])

        M_m0mP3 = np.zeros((4, 4))
        M_m0mP3[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        M_m0mP3[:3, 3] = np.array((0.14, -0.10, 0.010))
        M_m0mP3[3, :] = np.array([0, 0, 0, 1])
        M_m0mP = [M_m0mP1, M_m0mP2, M_m0mP3]
    elif object_type == 'screwdriver':
        M_m0mP1 = np.zeros((4, 4))
        M_m0mP1[:3, :3] = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        M_m0mP1[:3, 3] = np.array((0.06025, -0.08475, 0.000))
        M_m0mP1[3, :] = np.array([0, 0, 0, 1])

        M_m0mP2 = np.zeros((4, 4))
        M_m0mP2[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        M_m0mP2[:3, 3] = np.array((0.11, -0.10, 0.004))
        M_m0mP2[3, :] = np.array([0, 0, 0, 1])

        M_m0mP3 = np.zeros((4, 4))
        M_m0mP3[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        M_m0mP3[:3, 3] = np.array((0.14, -0.10, 0.004))
        M_m0mP3[3, :] = np.array([0, 0, 0, 1])
        M_m0mP = [M_m0mP1, M_m0mP2, M_m0mP3]
    elif object_type == 'mouse':
        M_m0mP1 = np.zeros((4, 4))
        M_m0mP1[:3, :3] = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        M_m0mP1[:3, 3] = np.array((0.06025, -0.08475, 0.000))
        M_m0mP1[3, :] = np.array([0, 0, 0, 1])

        M_m0mP2 = np.zeros((4, 4))
        M_m0mP2[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        M_m0mP2[:3, 3] = np.array((0.030, -0.10, 0.012))
        M_m0mP2[3, :] = np.array([0, 0, 0, 1])

        M_m0mP3 = np.zeros((4, 4))
        M_m0mP3[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        M_m0mP3[:3, 3] = np.array((0.045, -0.10, 0.012))
        M_m0mP3[3, :] = np.array([0, 0, 0, 1])
        M_m0mP = [M_m0mP1, M_m0mP2, M_m0mP3]
    elif object_type == 'ball':
        M_m0mP1 = np.zeros((4, 4))
        M_m0mP1[:3, :3] = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        M_m0mP1[:3, 3] = np.array((0.06025, -0.08475, 0.000))
        M_m0mP1[3, :] = np.array([0, 0, 0, 1])

        M_m0mP2 = np.zeros((4, 4))
        M_m0mP2[:3, :3] = np.array([[0.5, -0.8658, 0], [0.8658, 0.5, 0], [0, 0, 1]])
        M_m0mP2[:3, 3] = np.array((0.05, -0.10, 0.039))
        M_m0mP2[3, :] = np.array([0, 0, 0, 1])

        M_m0mP3 = np.zeros((4, 4))
        M_m0mP3[:3, :3] = np.array([[0.5, 0.8658, 0], [-0.8658, 0.5, 0], [0, 0, 1]])
        M_m0mP3[:3, 3] = np.array((0.05, -0.10, 0.039))
        M_m0mP3[3, :] = np.array([0, 0, 0, 1])
        M_m0mP = [M_m0mP1, M_m0mP2, M_m0mP3]
    else:
        M_m0mP1 = np.zeros((4, 4))
        M_m0mP1[:3, :3] = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        M_m0mP1[:3, 3] = np.array((0.06025, -0.08475, 0.000))
        M_m0mP1[3, :] = np.array([0, 0, 0, 1])

        M_m0mP2 = np.zeros((4, 4))
        M_m0mP2[:3, :3] = np.array([[0.5, -0.8658, 0], [0.8658, 0.5, 0], [0, 0, 1]])
        M_m0mP2[:3, 3] = np.array((0.05, -0.10, 0.07))
        M_m0mP2[3, :] = np.array([0, 0, 0, 1])

        M_m0mP3 = np.zeros((4, 4))
        M_m0mP3[:3, :3] = np.array([[0.5, 0.8658, 0], [-0.8658, 0.5, 0], [0, 0, 1]])
        M_m0mP3[:3, 3] = np.array((0.05, -0.10, 0.07))
        M_m0mP3[3, :] = np.array([0, 0, 0, 1])
        M_m0mP = [M_m0mP1, M_m0mP2, M_m0mP3]
    return M_m0mP


def get_M_Cmp(gray, M_Cm0, M_m0mP, visualize=False):
    markerLength = 0.063
    M_Cmp = []
    for tmp_M in M_m0mP:
        tmp_M_Cmp = np.dot(M_Cm0, tmp_M)
        if visualize:
            rvec, _ = cv2.Rodrigues(tmp_M_Cmp[:3, :3])
            tvec = tmp_M_Cmp[:3, 3]
            aruco.drawAxis(gray, cameraMatrix, distCoeffs, rvec, tvec, markerLength)
        M_Cmp.append(tmp_M_Cmp)
        if OBJ_POSE_ONLY: break
    return M_Cmp


def get_M_bmp(M_CL, M_Cmp):
    rvec_bL = ([1, 0, 0], [0, 1, 0], [0, 0, 1])
    M_bL = np.zeros((4, 4))
    M_bL[:3, :3] = rvec_bL
    M_bL[:3, 3] = ([0.30, 0.30, -0.125])
    M_bL[3, :] = np.array([0, 0, 0, 1])

    M_bC = np.dot(M_bL, np.linalg.inv(M_CL))
    M_bmp = []
    for tmp_M in M_Cmp:
        tmp_M_bmp = np.dot(M_bC, tmp_M)
        M_bmp.append(tmp_M_bmp)
        if OBJ_POSE_ONLY: break
    return M_bmp, M_bL


def get_M_b_ee_image(image_init, M_CL, M_bL, visualize=True):
    # parameters
    markerLength = 0.03
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(image_init, aruco_dict, parameters=parameters)

    # get M_C_mx
    M_Cm0 = np.eye(4, dtype=float)
    M_b_ee = []

    foundID = -1
    if ids is not None:
        foundFlag = False
        for index, id in enumerate(ids):
            rvec_Cm0, tvec_Cm0, _objPoints_Cm0 = aruco.estimatePoseSingleMarkers(corners[index], markerLength,
                                                                                 cameraMatrix, distCoeffs)
            dst_Cm0, jacobian = cv2.Rodrigues(rvec_Cm0)
            M_Cm0[:3, :3] = dst_Cm0
            M_Cm0[:3, 3] = tvec_Cm0
            M_Cm0[3, :] = np.array([0, 0, 0, 1])
            M_Lm0 = np.dot(np.linalg.inv(M_CL), M_Cm0)
            eulerAngle = rotationMatrixToEulerAngles(M_Lm0[:3, :3])

            M_ee_ar = np.zeros((4, 4))
            if id[0] == 3 and abs(eulerAngle[0]) < 0.5 and abs(eulerAngle[1]) < 0.5:
                Euler_ee_ar = np.array([x1, y1, z1, R1, P1, Y1])
                M_ee_ar = tf.transformations.euler_matrix(Euler_ee_ar[3], Euler_ee_ar[4], Euler_ee_ar[5])
                M_ee_ar[:3, 3] = np.array([Euler_ee_ar[0], Euler_ee_ar[1], Euler_ee_ar[2]])
                foundFlag = True
                foundID = 3
                break
            elif id[0] == 4 and abs(eulerAngle[0]) < 0.5 and abs(eulerAngle[1]) < 0.5:
                Euler_ee_ar = np.array([x2, y2, z2, R2, P2, Y2])
                M_ee_ar = tf.transformations.euler_matrix(Euler_ee_ar[3], Euler_ee_ar[4], Euler_ee_ar[5])
                M_ee_ar[:3, 3] = np.array([Euler_ee_ar[0], Euler_ee_ar[1], Euler_ee_ar[2]])
                foundFlag = True
                foundID = 4
                break
            elif id[0] == 5 and abs(eulerAngle[0]) < 0.5 and abs(eulerAngle[1]) < 0.5:
                Euler_ee_ar = np.array([x3, y3, z3, R3, P3, Y3])
                M_ee_ar = tf.transformations.euler_matrix(Euler_ee_ar[3], Euler_ee_ar[4], Euler_ee_ar[5])
                M_ee_ar[:3, 3] = np.array([Euler_ee_ar[0], Euler_ee_ar[1], Euler_ee_ar[2]])
                foundFlag = True
                foundID = 5
                break
        if foundFlag:
            M_C_ee = np.dot(M_Cm0, np.linalg.inv(M_ee_ar))
            rvec, _ = cv2.Rodrigues(M_C_ee[:3, :3])
            tvec = M_C_ee[:3, 3]
            aruco.drawAxis(image_init, cameraMatrix, distCoeffs, rvec, tvec, markerLength)

            rvec, _ = cv2.Rodrigues(M_Cm0[:3, :3])
            tvec = M_Cm0[:3, 3]
            aruco.drawAxis(image_init, cameraMatrix, distCoeffs, rvec, tvec, markerLength)

            M_b_ee = np.dot(np.dot(M_bL, np.linalg.inv(M_CL)), M_C_ee)
        else:
            print('no 3 4 5 found')

    else:
        print('no 3x3 aruco tag detected!')

    return M_b_ee, M_Cm0, foundID


def write_M_b_a2file(M_CL, M_C_ar, M_bL, foundID, path):
    M_bC = np.dot(M_bL, np.linalg.inv(M_CL))
    M_b_ar = np.dot(M_bC, M_C_ar)
    quaternion_M_b_ar = tf.transformations.quaternion_from_matrix(M_b_ar)

    file = open(path, "w")
    if foundID == -1:
        file.write(str(1) + '\n' + str(3) + '\n')
        file.write(str(0.41151568633679786) + '\n')
        file.write(str(0.15926784398046456) + '\n')
        file.write(str(0.13827162013559502) + '\n')
        file.write(str(-0.04426421086057804) + '\n')
        file.write(str(-0.03467245114964361) + '\n')
        file.write(str(-0.8606309930297725) + '\n')
        file.write(str(0.5061153965300337) + '\n')

        file.write(str(0) + '\n' + str(4) + '\n')
        for tmp_n in xrange(7):
            file.write(str(-1) + '\n')

        file.write(str(0) + '\n' + str(5) + '\n')
        for tmp_n in xrange(7):
            file.write(str(-1) + '\n')

    elif foundID == 3:
        file.write(str(1) + '\n' + str(3) + '\n')
        file.write(str(M_b_ar[0, 3]) + '\n')
        file.write(str(M_b_ar[1, 3]) + '\n')
        file.write(str(M_b_ar[2, 3]) + '\n')
        file.write(str(quaternion_M_b_ar[0]) + '\n')
        file.write(str(quaternion_M_b_ar[1]) + '\n')
        file.write(str(quaternion_M_b_ar[2]) + '\n')
        file.write(str(quaternion_M_b_ar[3]) + '\n')

        file.write(str(0) + '\n' + str(4) + '\n')
        for tmp_n in xrange(7):
            file.write(str(-1) + '\n')

        file.write(str(0) + '\n' + str(5) + '\n')
        for tmp_n in xrange(7):
            file.write(str(-1) + '\n')

    elif foundID == 4:
        file.write(str(0) + '\n' + str(3) + '\n')
        for tmp_n in xrange(7):
            file.write(str(-1) + '\n')

        file.write(str(1) + '\n' + str(4) + '\n')
        file.write(str(M_b_ar[0, 3]) + '\n')
        file.write(str(M_b_ar[1, 3]) + '\n')
        file.write(str(M_b_ar[2, 3]) + '\n')
        file.write(str(quaternion_M_b_ar[0]) + '\n')
        file.write(str(quaternion_M_b_ar[1]) + '\n')
        file.write(str(quaternion_M_b_ar[2]) + '\n')
        file.write(str(quaternion_M_b_ar[3]) + '\n')

        file.write(str(0) + '\n' + str(5) + '\n')
        for tmp_n in xrange(7):
            file.write(str(-1) + '\n')

    elif foundID == 5:
        file.write(str(0) + '\n' + str(3) + '\n')
        for tmp_n in xrange(7):
            file.write(str(-1) + '\n')

        file.write(str(0) + '\n' + str(4) + '\n')
        for tmp_n in xrange(7):
            file.write(str(-1) + '\n')

        file.write(str(1) + '\n' + str(5) + '\n')
        file.write(str(M_b_ar[0, 3]) + '\n')
        file.write(str(M_b_ar[1, 3]) + '\n')
        file.write(str(M_b_ar[2, 3]) + '\n')
        file.write(str(quaternion_M_b_ar[0]) + '\n')
        file.write(str(quaternion_M_b_ar[1]) + '\n')
        file.write(str(quaternion_M_b_ar[2]) + '\n')
        file.write(str(quaternion_M_b_ar[3]) + '\n')

    file.close()

def write_graspLabel2file(gray, M_bmp, M_b_ee, tmpPathGT, visualize=True):
    file = open(tmpPathGT, "w")
    if len(M_b_ee) is not 0:
        dy_pos = 0
        for M_bmp_i in M_bmp:
            dy_pos += 40
            # eulerAngle_bmp = rotationMatrixToEulerAngles(M_bmp_i[:3, :3])
            # eulerAngle_b_ee = rotationMatrixToEulerAngles(M_b_ee[:3, :3])
            _, _, eulerAngle_bmp = tf.transformations.euler_from_matrix(M_bmp_i[:3, :3])
            _, _, eulerAngle_b_ee = tf.transformations.euler_from_matrix(M_b_ee[:3, :3])
            valid_grasp = grasp_check(M_bmp_i[:3, 3], eulerAngle_bmp, M_b_ee[:3, 3], eulerAngle_b_ee)
            if visualize:
                if valid_grasp:
                    cv2.putText(gray, 'valid', (30, 30 + dy_pos), 2, 1.5, (0, 0, 255))
                    file.write(str(1))
                    file.write("\n")
                else:
                    cv2.putText(gray, 'false', (30, 30 + dy_pos), 2, 1.5, (0, 0, 255))
                    file.write(str(0))
                    file.write("\n")
    else:
        for i in range(3):
            file.write(str(0))
            file.write("\n")
    file.close()


def write_graspPose2file(M_bmp, tmpPathGT_pose):
    file = open(tmpPathGT_pose, "w")
    for M_bmp_i in M_bmp:
        _, _, eulerAngle_bmp = tf.transformations.euler_from_matrix(M_bmp_i[:3, :3])
        angle = (eulerAngle_bmp) / PI * 180.0
        for i in range(3):
            file.write(str(M_bmp_i[i,3]))
            file.write("\n")
        file.write(str(angle))
    file.close()

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def validSurrounding(M_Cmp_matrix, M_mpmx):
    # given Camera to a pose, check if the center is within camera view

    M_Cmx = np.dot(M_Cmp_matrix, M_mpmx)
    imagePts = np.dot(cameraMatrix,M_Cmx[:3,3])
    imagePts = imagePts[:2]/imagePts[2]

    valid = False
    print imagePts
    if imagePts[0] <= CAMERA_DIM[0] and imagePts[0] >= 0 and imagePts[1] <= CAMERA_DIM[1] and imagePts[1] >= 0:
        valid = True
    return valid


def validSurroundings(gray, M_Cmp):
    # given Camera to object pose, check if four surrounding tags are within the camera view

    if len(M_Cmp) == 4:
        print ("OBJ_POSE_ONLY flag has to be set to True for this function")
        return False, False, False, False

    if len(M_Cmp) == 0:
        print ("no object detected")
        return False, False, False, False


    M_Cmp_matrix = M_Cmp[0]

    # mp to 7:
    M_mpmx = np.array([[1, 0, 0, 0.06025],
                       [0, 1, 0, -0.08475],
                       [0, 0, 1, 0.000],
                       [0, 0, 0, 1]])

    valid_7 = validSurrounding(M_Cmp_matrix, M_mpmx)

    # mp to 8:
    M_mpmx = np.array([[1, 0, 0, -0.06025],
                       [0, 1, 0, -0.08475],
                       [0, 0, 1, 0.000],
                       [0, 0, 0, 1]])

    valid_8 = validSurrounding(M_Cmp_matrix, M_mpmx)

    # mp to 9:
    M_mpmx = np.array([[1, 0, 0, 0.06025],
                       [0, 1, 0, 0.08475],
                       [0, 0, 1, 0.000],
                       [0, 0, 0, 1]])

    valid_9 = validSurrounding(M_Cmp_matrix, M_mpmx)

    # mp to 10:
    M_mpmx = np.array([[1, 0, 0, -0.06025],
                       [0, 1, 0, 0.08475],
                       [0, 0, 1, 0.000],
                       [0, 0, 0, 1]])

    valid_10 = validSurrounding(M_Cmp_matrix, M_mpmx)

    return valid_7, valid_8, valid_9, valid_10,


def aruco_pose(frame_current):


    # change format
    gray = frame_current.astype(np.uint8)

    # get M_CL: camera to aruco 466 (world reference)
    # the second argument is the image for visualization, if init image exist, put init image
    M_CL = get_M_CL(gray, gray, visualize=True)
    #print(M_CL)

    # get M_Cm0: camera to object pose
    # the second argument is the image for visualization, if init image exist, put init image
    M_Cm0 = get_M_Cm0(gray, gray, visualize=True)

    # get M_m0mp: object pose to grasp pose, according to object type
    M_m0mP = get_M_m0mP(OBJECT_TYPE)

    # get camera to grasp pose list
    M_Cmp = get_M_Cmp(gray, M_Cm0, M_m0mP, visualize=True)

    # check if 4 surrounding aruco tags are in the camera view
    valid_7, valid_8, valid_9, valid_10 = validSurroundings(gray, M_Cmp)
    print ("valid_7 is " + str(valid_7))
    print ("valid_8 is " + str(valid_8))
    print ("valid_9 is " + str(valid_9))
    print ("valid_10 is " + str(valid_10))
    print valid_7 and valid_8 and valid_9 and valid_10

    # get base to grasp pose list
    M_bmp, M_bL = get_M_bmp(M_CL, M_Cmp)

    # get M_b_ee: base to end-effector (one of them), also return camera to the tag on ee, id of the tag
    M_b_ee, M_Car, foundID = get_M_b_ee_image(gray, M_CL, M_bL, visualize=True)

    # write base to aruco transformation
    write_M_b_a2file(M_CL, M_Car, M_bL, foundID, tmpPathGT_ar)

    # write success label for each grasp pose
    write_graspLabel2file(gray, M_bmp, M_b_ee, tmpPathGT, visualize=True)

    write_graspPose2file(M_bmp, tmpPathGT_pose)


    return gray


#TILT_MAX = 90
#TILT_STEP = 5
#TILT_MIN = -90
#TILT = 0
#
## reset the camera tilt
#ctx = freenect.init()
#print('ONE')
#dev = freenect.open_device(ctx, freenect.num_devices(ctx) - 1)
#print('TWO')
#freenect.set_tilt_degs(dev, TILT)
#print('THREE')
## python wrapper dont register for video/depth so pointer needs to be shutdown before inquiring video
#freenect.shutdown(ctx)
#
#count = START_COUNT
#while 1:
#    # get a frame from RGB camera
#    frame_current = get_video()
#    # get a frame from depth sensor
#    depth, depth_raw = get_depth()
#    # display RGB image
#    cv2.imshow('RGB image', frame_current)
#    # display depth image
#    cv2.imshow('Depth image', depth)
#
#    frame_processed = aruco_pose(frame_current)
#    # display processed image
#    cv2.imshow('Processed image', frame_processed)
#
#
#    # quit program when 'esc' key is pressed
#    k = cv2.waitKey(5) & 0xFF
#    if k == 27:
#        break
#    # wait for 's' key to save
#    elif k == ord('s'):
#        save_name = IMG_SAVEPATH + "/pic_" + str(count) + ".png"
#        cv2.imwrite(save_name, frame_current)
#
#        save_name = IMG_SAVEPATH + "/pic_" + str(count) + "_d.png"
#        cv2.imwrite(save_name, depth)
#
#        save_name = IMG_SAVEPATH + "/pic_" + str(count) + "_raw.png"
#        cv2.imwrite(save_name, depth_raw)
#
#        count += 1
#
#    elif k == ord('t'):
#        # conflit registeration between video and tilt, annoying
#        freenect.sync_stop()
#        time.sleep(0.1)
#
#        ctx = freenect.init()
#        dev = freenect.open_device(ctx, freenect.num_devices(ctx) - 1)
#
#        TILT = min(TILT + TILT_STEP, TILT_MAX)
#        print "Setting TILT: ", TILT
#        freenect.set_tilt_degs(dev, TILT)
#
#        freenect.shutdown(ctx)
#        time.sleep(0.1)
#
#
#    elif k == ord('g'):
#        # conflit registeration between video and tilt, annoying
#        freenect.sync_stop()
#        time.sleep(0.1)
#
#        ctx = freenect.init()
#        dev = freenect.open_device(ctx, freenect.num_devices(ctx) - 1)
#
#        TILT = max(TILT - TILT_STEP, TILT_MIN)
#        print "Setting TILT: ", TILT
#        freenect.set_tilt_degs(dev, TILT)
#
#        freenect.shutdown(ctx)
#        time.sleep(0.1)
#
#cv2.destroyAllWindows()
