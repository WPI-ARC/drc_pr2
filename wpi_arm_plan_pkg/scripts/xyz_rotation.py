#   Author: Jim Mainprice
#
# -*- coding: utf-8 -*-
from numpy import *

'''Basic XYZ rotation
Input: 1x3 array of rotations about x, y, and z
Output: 3x3 rotation matrix'''
def xyz_rotation(r):
    Sx = sin(r[0]);
    Cx = cos(r[0]);
    Sy = sin(r[1]);
    Cy = cos(r[1]);
    Sz = sin(r[2]);
    Cz = cos(r[2]);
    SyCz = Sy*Cz;
    SySz = Sy*Sz;

    M = eye(3);

    # lin 1
    M[0][0] = Cy*Cz;
    M[0][1] = Sx*SyCz - Cx*Sz;
    M[0][2] = Cx*SyCz + Sx*Sz;

    # lin 1
    M[1][0] = Cy*Sz;
    M[1][1] = Sx*SySz + Cx*Cz;
    M[1][2] = Cx*SySz - Sx*Cz;

    # lin 2
    M[2][0] = -Sy;
    M[2][1] = Sx*Cy;
    M[2][2] = Cx*Cy;
    return mat(M)

'''Basic Yaw Pitch Roll rotation
Input: 1x3 array of rotations about x, y, and z
Output: 3x3 rotation matrix'''
def yaw_pitch_roll_rotation(r):
    Sa = sin(r[0]);
    Ca = cos(r[0]);
    Sb = sin(r[1]);
    Cb = cos(r[1]);
    Sc = sin(r[2]);
    Cc = cos(r[2]);

    M = eye(3);

    # lin 0
    M[0][0] = Ca*Cb;
    M[0][1] = Ca*Sb*Sc - Sa*Cc;
    M[0][2] = Ca*Sb*Cc + Sa*Sc;

    # lin1
    M[1][0] = Sa*Cb;
    M[1][1] = Sa*Sb*Sc + Ca*Cc;
    M[1][2] = Sa*Sb*Cc - Ca*Sc;

    # lin 2
    M[2][0] = -Sb;
    M[2][1] = Cb*Sc;
    M[2][2] = Cb*Cc;
    return mat(M)
