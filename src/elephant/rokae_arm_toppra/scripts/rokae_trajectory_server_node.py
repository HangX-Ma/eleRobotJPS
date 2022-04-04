#!/usr/bin/env python

from __future__ import print_function
from rokae_arm_toppra.srv import ToppRa, ToppRaResponse
import rospy

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np

def toppra_handle(req):
    way_pts_array = np.array(req.way_points)
    way_pts_array = way_pts_array.reshape(req.N,req.dof)
    alim_array    = np.array(req.alim)
    path          = ta.SplineInterpolator(np.linspace(0, 1, req.N), way_pts_array)
    alim_bi       = np.vstack((-alim_array, alim_array)).T
    pc_acc        = constraint.JointAccelerationConstraint(alim_bi, discretization_scheme = constraint.DiscretizationType.Interpolation)
    instance      = algo.TOPPRA([pc_acc], path, parametrizer="ParametrizeConstAccel")
    jnt_traj      = instance.compute_trajectory()     
    return ToppRaResponse(jnt_traj(req.t))

def get_traj_server():
    rospy.init_node('rokae_arm_toppra')
    s = rospy.Service('rokae_trajectory_server', ToppRa, toppra_handle)
    print("Ready for tranjctory output")
    rospy.spin()

if __name__ == "__main__":
    get_traj_server()