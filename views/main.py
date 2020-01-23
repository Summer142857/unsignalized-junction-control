import traci
from traci import constants as tc
import numpy as np
import subprocess
import sys
from scheduleMaking.BasicTreeSearch import BasicTree
from scheduleMaking.caseJudging import caseClassify, read_config

LANE_ID = ["l_0", "b_0", "r_0", "u_0"]   # left, below, right, up
LANE_LENGTH = 200

def get_vehicles(dist = 50.0):
    '''
    Get the vehicles in dist size Buffered Area
    '''
    vids = traci.vehicle.getIDList()
    poiVid = dict()
    for vid in vids:
        if traci.vehicle.getLaneID(vid) in LANE_ID:
            if traci.vehicle.getLanePosition(vid) > LANE_LENGTH - dist:
                poiVid[vid] = (traci.vehicle.getLaneID(vid), traci.vehicle.getLanePosition(vid))
    return poiVid

def treeGenerate():
    vids = get_vehicles()
    t = BasicTree(vids)
    t.build()
    t.show()
    print("the number of vehicles is %d" %len(vids))
    print("the leaves of the tree is %d" %t.leaf_num())
    print("========================================")
    return t

if __name__ == "__main__":
    PORT = 8813
    sumoBinary = "D:\\sumo\\bin\\sumo-gui"
    cfg_filepath = "D:\\sumo_projects\\intersection\\intersection.sumocfg"
    sumoProcess = subprocess.Popen([sumoBinary, "-c", cfg_filepath, "--remote-port", str(PORT)],
        stdout=sys.stdout, stderr=sys.stderr)
    traci.init(PORT)
    collisionDict = read_config()
    for sim_step in range(1000):
        vlist = get_vehicles()
        traci.simulationStep()
        pattern = caseClassify(vlist, collisionDict)
        print("Current intersection's circumstance is Case %d" %pattern)
    traci.close()