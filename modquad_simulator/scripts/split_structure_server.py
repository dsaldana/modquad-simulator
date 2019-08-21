#!/usr/bin/env python
import rospy
import numpy as np
import networkx as nx

from modsim import params
from modsim.datatype.structure import Structure
from modquad_simulator.srv import SplitStructure, SplitStructureResponse
from compiled_scheduler.modset import modset
from compiled_scheduler.mqmod import mqmod

def _struc_to_mat(ids, xx, yy):
    """
    :param ids: modquad module ID #s involved in this structure
    :param xx: x pos of each module relative to struc center of mass
    :param yy: y pos of each module relative to struc center of mass
    :return struc: numpy matrix with each cell matching id of mod in it
    """
    # Step 1: find out shape of structure using positions of mods
    xx = np.round(np.array(xx) / params.cage_width)
    yy = np.round(np.array(yy) / params.cage_width)
    xx.astype(int)
    yy.astype(int)
    xx += -1 * np.min(xx)
    yy += -1 * np.min(yy)
    xx = [int(x) for x in list(xx)]
    yy = [int(y) for y in list(yy)]
    minx = min(xx)
    maxx = max(xx)
    miny = min(yy)
    maxy = max(yy)
    struc = np.zeros((int(maxx - minx) + 1, int(maxy - miny) + 1)) - 1
    for i in range(0, len(xx)):
        struc[xx[i], yy[i]] = ids[i]
    return struc

def handle_split_structure(args):
    """
    :param ids: modquad module ID #s involved in this structure
    :param xx: x pos of each module relative to struc center of mass
    :param yy: y pos of each module relative to struc center of mass
    :param dim: which dim(x,y) to split the structure along
    :param loc: split to occur at line just before loc index mod in dim
    :param split_ind: if multi splits possible @ dim,loc: do split_ind^{th} one
    """
    ids        = args.ids
    xx         = args.xx
    yy         = args.yy
    mod_faults = args.mod_faults
    rot_faults = args.rot_faults
    dim        = args.dim
    loc        = args.loc
    split_ind  = args.split_ind
    ids = [int(i) for i in ids]
    struc = _struc_to_mat(ids, xx, yy)

    # Create networkx graph that replicates the structure shape
    G = nx.grid_graph(dim=[struc.shape[0], struc.shape[1]])
    rmlist = [pos for pos in G if struc[pos[0], pos[1]] == -1]
    [G.remove_node(n) for n in rmlist]

    # Generate split
    s1nodes = []
    s2nodes = []
    s1nodes = [pos for pos in G if pos[dim] <  loc]
    s2nodes = [pos for pos in G if pos[dim] >= loc]
    s1 = G.subgraph(s1nodes)
    s2 = G.subgraph(s2nodes)
    s1c = [c for c in nx.connected_components(s1)]
    s2c = [c for c in nx.connected_components(s2)]
    if len(s1c) > 1 and len(s2c) > 1:
        raise ValueError("Cannot have more than 2 subgraphs generated on both sides of split")
    retainers = set()
    splitters = set()
    if len(s1c) > 1:
        for i in range(0, len(s1c)):
            if i == split_ind:
                splitters = s1c[split_ind]
            else:
                retainers = retainers.union(s1c[i])
        retainers = retainers.union(s2)
    elif len(s2c) > 1:
        for i in range(0, len(s2c)):
            if i == split_ind:
                splitters = s2c[split_ind]
            else:
                retainers = retainers.union(s2c[i])
        retainers = retainers.union(s1)
    else: # only 2 substrucs created
        retainers = s1.nodes
        splitters = s2.nodes
    s1 = G.subgraph(retainers)
    s2 = G.subgraph(splitters)

    indices = np.nonzero(struc + 1)
    indices = list(zip(*indices))
    
    Gnodes = list(G.nodes)
    ids1 = [struc[Gnodes[i][0], Gnodes[i][1]] for i in range(0, len(indices)) if Gnodes[i] in s1.nodes]
    ids2 = [struc[Gnodes[i][0], Gnodes[i][1]] for i in range(0, len(indices)) if Gnodes[i] in s2.nodes]
    xx1  = [ xx[ids.index(struc[Gnodes[i][0], Gnodes[i][1]])] for i in range(0, len(indices)) if Gnodes[i] in s1.nodes]
    xx2  = [ xx[ids.index(struc[Gnodes[i][0], Gnodes[i][1]])] for i in range(0, len(indices)) if Gnodes[i] in s2.nodes]
    yy1  = [ yy[ids.index(struc[Gnodes[i][0], Gnodes[i][1]])] for i in range(0, len(indices)) if Gnodes[i] in s1.nodes]
    yy2  = [ yy[ids.index(struc[Gnodes[i][0], Gnodes[i][1]])] for i in range(0, len(indices)) if Gnodes[i] in s2.nodes]

    #faults1 = [f for f in failures if f[0] in ids1]
    #faults2 = [f for f in failures if f[0] in ids2]
    # TODO Properly handle faults in splitting
    faults1 = []
    faults2 = []
    ret = {'ids1':ids1, 'xx1':xx1, 'yy1':yy1, 'faults1':faults1, 'ids2':ids2, 'xx2':xx2, 'yy2':yy2, 'faults2':faults2}
    print(ret)
    return ret

def split_structure_server():
    """
    Server that listens for splitting requests of structures 
    and handles them
    """
    rospy.init_node("split_structure_server")
    s = rospy.Service('SplitStructure', SplitStructure, handle_split_structure)
    rospy.spin()

if __name__ == '__main__':
    split_structure_server()
