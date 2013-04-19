# Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
#   Author: Dmitry Berenson <dberenso@cs.cmu.edu>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor Carnegie Mellon University,
#       nor the names of their contributors, may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
#   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-
'''Rodrigues formula
Input: 1x3 array of rotations about x, y, and z
Output: 3x3 rotation matrix'''
from numpy import *

def rodrigues(r):
    def S(n):
        Sn = array([[0,-n[2],n[1]],[n[2],0,-n[0]],[-n[1],n[0],0]])
        return Sn
    theta = linalg.norm(r)
    if theta > 1e-30:
        n = r/theta
        Sn = S(n)
        R = eye(3) + sin(theta)*Sn + (1-cos(theta))*dot(Sn,Sn)
    else:
        Sr = S(r)
        theta2 = theta**2
        R = eye(3) + (1-theta2/6.)*Sr + (.5-theta2/24.)*dot(Sr,Sr)
    return mat(R)

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
