import traci
import subprocess
import sys
from utils.LoadConfig import read_config
from scheduleMaking.BasicTreeSearch import BasicTree
from scheduleMaking.PrePruneTree import PrePruneTree
import time
import numpy as np

LANE_ID = ["l_0", "b_0", "r_0", "u_0", "l_1", "b_1", "r_1", "u_1", "l_2", "b_2", "r_2", "u_2"]   # left, below, right, up
JUNCTION_ID = [":gneJ11_0_0", ":gneJ11_1_0", ":gneJ11_2_0", ":gneJ11_3_0",":gneJ11_4_0", ":gneJ11_5_0",":gneJ11_6_0", ":gneJ11_7_0", ":gneJ11_8_0",":gneJ11_9_0", ":gneJ11_10_0",":gneJ11_11_0"]
LANE_LENGTH = 200

vehCount = []
cost = []

class BasicPlanning:

    def __init__(self, dist=50.0):
        self.dist = dist
        self.vehsInfo = dict()
        self.tree = None

        self.conflictDict = read_config()

    def _getVehsInfo(self):
        '''
        Get the vehicles in dist size Buffered Area
        '''
        vids = traci.vehicle.getIDList()
        for vid in vids:
            if traci.vehicle.getLaneID(vid) in LANE_ID:
                if traci.vehicle.getLanePosition(vid) > LANE_LENGTH - self.dist:
                    self.vehsInfo[vid] = [traci.vehicle.getLaneID(vid), traci.vehicle.getLanePosition(vid), None]
        # if a vehicle pass the junction successfully, delete it from the dict
        for vid in list(self.vehsInfo.keys()):
            if traci.vehicle.getLaneID(vid) not in (LANE_ID + JUNCTION_ID):
                del self.vehsInfo[vid]


    def treeGenerate(self):
        if self.vehsInfo:
            # constructor: vehsInfo
            self.tree = PrePruneTree(self.vehsInfo)
            start = time.time()
            self.tree.build()
            end = time.time()
            vehCount.append(len(self.vehsInfo.keys()))
            cost.append(end - start)
            self.tree.show()
            order = self.tree.legal_orders()
            print(order)
            print("There're %d possible orders." %len(order))
            print("========================================")


    def _testLeader(self, vid):
        '''
        test whether the vehicle vid exists a leader in Buffed Area
        :return:boolean  (if exists a leader, return True)
        '''
        vPos = self.vehsInfo[vid][1]    # current vehicle's position on the lane
        leader = traci.vehicle.getLeader(vid)
        flag = False
        if leader is not None:
            interval = leader[1]            # the safe distance between vehicle and its leader
            vbPos = vPos - (LANE_LENGTH - self.dist)
            if vbPos + interval < self.dist:
                flag = True
        return flag


    def _testConflict(self, vid):
        '''
        :return: boolean  (if exists conflict, return True)
        '''
        flag = False
        flowIds = set([veh.split('.')[0] for veh in self.vehsInfo.keys()])
        # print(flowIds)
        for flow in flowIds:
            if vid.split('.')[0] in self.conflictDict[flow]:
                flag = True
        return flag


    def _caseClassify(self):
        '''
        Add case information to vehsInfo
        :return:
        '''
        for vid in self.vehsInfo.keys():
            if not self._testLeader(vid):
                # no leader vehicle
                if not self._testConflict(vid):
                    # vehsInfo[vid]: [lane_id, lane_position, case]
                    self.vehsInfo[vid][2] = "A"
                else:
                    self.vehsInfo[vid][2] = "C"
            else:
                if not self._testConflict(vid):
                    self.vehsInfo[vid][2] = "B"
                else:
                    self.vehsInfo[vid][2] = "D"

    def simOneStep(self):
        self._getVehsInfo()
        self._caseClassify()
        self.treeGenerate()
        print(self.vehsInfo)


if __name__ == "__main__":
    PORT = 8813
    sumoBinary = "D:\\sumo\\bin\\sumo-gui"      # your sumo binary path
    cfg_filepath = "../sumoFiles\\no_signal\\intersection.sumocfg"
    sumoProcess = subprocess.Popen([sumoBinary, "-c", cfg_filepath, "--remote-port", str(PORT)],
        stdout=sys.stdout, stderr=sys.stderr)
    traci.init(PORT)
    t = BasicPlanning()
    for sim_step in range(2000):
        traci.simulationStep()
        t.simOneStep()
    traci.close()
