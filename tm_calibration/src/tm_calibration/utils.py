import math
import numpy as np

# Slimmed down and cleaned up from tmr_description/script/_modify_urdf.py

_DoF = 6
_A = 0
_ALPHA = 1
_BETA = 2
_D = 3
_THETA = 4
_LLIM = 5
_ULIM = 6


def is_rotation_matrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def T_a_alpha(a, alpha):
    return np.array(
        [
            [1, 0, 0, a],
            [0, math.cos(alpha), -math.sin(alpha), 0],
            [0, math.sin(alpha), math.cos(alpha), 0],
            [0, 0, 0, 1],
        ]
    )


def T_beta(beta):
    return np.array(
        [
            [math.cos(beta), 0, math.sin(beta), 0],
            [0, 1, 0, 0],
            [-math.sin(beta), 0, math.cos(beta), 0],
            [0, 0, 0, 1],
        ]
    )


def T_d_theta(d, theta):
    return np.array(
        [
            [math.cos(theta), -math.sin(theta), 0, 0],
            [math.sin(theta), math.cos(theta), 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1],
        ]
    )


# Calculates rotation matrix to euler angles
def euler_angles_from_rotation_matrix(R):
    assert is_rotation_matrix(R)

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


# URDF DH ((5+2) x 6) from TM DH Table (7x6) and Delta DH (5x6)
# a-alpha-beta-d-theta <-- theta-alpha-a-d-t-l-u + delta(theta-alpha-a-d-beta)
def urdf_DH_from_tm_DH(tm_DH, tm_DeltaDH):
    assert len(tm_DH) == 7 * _DoF and len(tm_DeltaDH) == 5 * _DoF

    urdf_DH = np.zeros([_DoF + 1, 7])
    for i in range(_DoF):
        urdf_DH[i, _D] = 0.001 * (tm_DH[7 * i + 3] + tm_DeltaDH[5 * i + 3])
        urdf_DH[i, _THETA] = math.radians(tm_DH[7 * i + 0] + tm_DeltaDH[5 * i + 0])
        urdf_DH[i, _LLIM] = math.radians(tm_DH[7 * i + 5])
        urdf_DH[i, _ULIM] = math.radians(tm_DH[7 * i + 6])
        urdf_DH[i + 1, _A] = 0.001 * (tm_DH[7 * i + 2] + tm_DeltaDH[5 * i + 2])
        urdf_DH[i + 1, _ALPHA] = math.radians(tm_DH[7 * i + 1] + tm_DeltaDH[5 * i + 1])
        urdf_DH[i + 1, _BETA] = math.radians(tm_DeltaDH[5 * i + 4])
    return urdf_DH


def xyzrpys_from_urdf_DH(udh):
    xyzs = np.zeros([_DoF + 1, 3])
    rpys = np.zeros([_DoF + 1, 3])
    for i in range(_DoF + 1):
        Ta = T_a_alpha(udh[i, _A], udh[i, _ALPHA])
        Tb = T_beta(udh[i, _BETA])
        Tc = T_d_theta(udh[i, _D], udh[i, _THETA])
        T = np.dot(Ta, np.dot(Tb, Tc))
        xyzs[i] = T[0:3, 3]
        rpys[i] = euler_angles_from_rotation_matrix(T[0:3, 0:3])

    return xyzs, rpys
