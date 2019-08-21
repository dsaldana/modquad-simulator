#!/usr/bin/env python
import rospy
import numpy as np

from modsim.datatype.structure import Structure

def gen_strucs_from_split(ret):
    ids1 = ["modquad{:02d}".format(id_val) for id_val in ret.ids1]
    xx1  = ret.xx1
    yy1  = ret.yy1
    f1   = ret.faults1
    ids2 = ["modquad{:02d}".format(id_val) for id_val in ret.ids2]
    xx2  = ret.xx2
    yy2  = ret.yy2
    f2   = ret.faults2
    struc1 = Structure(ids1, xx1, yy1, f1)
    struc2 = Structure(ids2, xx2, yy2, f2)
    return [struc1, struc2]
