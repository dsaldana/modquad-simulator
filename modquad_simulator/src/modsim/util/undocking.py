#!/usr/bin/env python
import rospy
import numpy as np

from modsim.datatype.structure import Structure

def gen_strucs_from_split(ret):
    ids1 = ["modquad{:02d}".format(id_val) for id_val in ret.ids1]
    xx1  = ret.xx1
    yy1  = ret.yy1
    mf1  = ret.mfaults1
    rf1  = ret.rfaults1
    ids2 = ["modquad{:02d}".format(id_val) for id_val in ret.ids2]
    xx2  = ret.xx2
    yy2  = ret.yy2
    mf2   = ret.mfaults2
    rf2   = ret.rfaults2
    #print("Generating new structures based on service return")
    #print("\tids1 = {}".format(ids1))
    #print("\txx1  = {}".format(xx1 ))
    #print("\tyy1  = {}".format(yy1 ))
    #print("\tmf1  = {}".format(mf1 ))
    #print("\trf1  = {}".format(rf1 ))
    #print('---')
    #print("\tids2 = {}".format(ids2))
    #print("\txx2  = {}".format(xx2 ))
    #print("\tyy2  = {}".format(yy2 ))
    #print("\tmf2  = {}".format(mf2 ))
    #print("\trf2  = {}".format(rf2 ))

    #print("Running service, get these fault sets")
    print("Making new structure consisting of: {}".format(ids1))
    struc1 = Structure(ids1, xx1, yy1, zip(mf1, rf1))
    print("Making new structure consisting of: {}".format(ids2))
    struc2 = Structure(ids2, xx2, yy2, zip(mf2, rf2))
    #print("Output structures")
    #print("\tids1 = {}".format(ids1))
    #print("\txx1  = {}".format(xx1 ))
    #print("\tyy1  = {}".format(yy1 ))
    #print("\tmf1  = {}".format(mf1 ))
    #print("\trf1  = {}".format(rf1 ))
    #print("\thash1= {}".format(struc1.gen_hashstring()))
    #print('---')
    #print("\tids2 = {}".format(ids2))
    #print("\txx2  = {}".format(xx2 ))
    #print("\tyy2  = {}".format(yy2 ))
    #print("\tmf2  = {}".format(mf2 ))
    #print("\trf2  = {}".format(rf2 ))
    #print("\thash2= {}".format(struc2.gen_hashstring()))
    return [struc1, struc2]

def split_srv_input_format(structure, split_dim, breakline, split_ind):
    #print("Check rotor failure in split service")
    #print([int(x[0]) for x in structure.motor_failure])
    #print([int(x[1]) for x in structure.motor_failure])
    #print("------See above-----")
    return [int(string[7:]) for string in structure.ids], \
            list(structure.xx), \
            list(structure.yy), \
            [int(x[0]) for x in structure.motor_failure], \
            [int(x[1]) for x in structure.motor_failure], \
            split_dim, breakline, split_ind
