import traci
import subprocess
import sys
from utils.LoadConfig import read_config
from scheduleMaking.BasicTreeSearch import BasicTree
from scheduleMaking.PrePruneTree import PrePruneTree
from domain.VirtualVehicle import VirtualVehicle
from Controller.dynamicController import speedUp, slowDown
import time
import copy
from Controller.areaController import laneChangeBan
import numpy as np
from domain.const import *



vehCount = []
cost = []

class BasicPlanning:

    def __init__(self, dist=50.0):
        self.dist = dist
        self.vehsInfo = dict()
        self.tree = None
        self.conflictDict = read_config()
        self.ignore_rightTurn = False
        self.delay = 0.5    # communication delay in vehicle mapping
        self.LANE_ID = LANE_ID


    def setIgnoreTurnRight(self, option):
        if option:
            self.ignore_rightTurn = True
            self.LANE_ID = list(set(LANE_ID) - set(["l_0", "b_0", "r_0", "u_0"]))

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
            # if traci.vehicle.getLaneID(vid) not in (self.LANE_ID + JUNCTION_ID):
            if traci.vehicle.getLaneID(vid) not in (self.LANE_ID):
                del self.vehsInfo[vid]


    def treeGenerate(self):
        if self.vehsInfo:
            # constructor: vehsInfo
            if len(self.vehsInfo) > 4:
                self.tree = PrePruneTree(self.vehsInfo)
            else:
                self.tree = BasicTree(self.vehsInfo)
            start = time.time()
            self.tree.build()
            end = time.time()
            vehCount.append(len(self.vehsInfo.keys()))
            cost.append(end - start)
            self.tree.show()
            order = self.tree.legal_orders()
            print("There're %d possible orders." %len(order))


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
        # if a vehicle has conflict pattern, but it's position beyond all the conflict vehicle, it's flag should be false
        conflictFlows = self.conflictDict[vid.split('.')[0]]
        for veh in self.vehsInfo.keys():
            if veh != vid:
                for flow in conflictFlows:
                    if veh.startswith(flow):
                        if self.vehsInfo[veh][1] > self.vehsInfo[vid][1]:
                            return True
        return False


    def _testTwoVehConflict(self, v1, v2):
        flag = False
        flow1 = v1.split('.')[0]
        flow2 = v2.split('.')[0]
        if flow2 in self.conflictDict[flow1]:
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


    def _testTurn(self, vehId):
        '''
        test whether the vehicle needed to turn in the junction
        :param vehId: vid
        :return: 1: go straight; 2: long turn; 3: short turn
        '''
        dir1, dir2 = vehId.split('.')[0].split('To')
        # judge if the vehicle needs to turn in the intersection
        straight_dict = {'left': 'Right', 'right':'Left', 'up': 'Below', 'below':'Up'}
        longTurn_dict = {'left': 'Up', 'right':'Below', 'up':'Right', 'below':'Left'}
        if straight_dict[dir1] == dir2:
            return 1
        elif longTurn_dict[dir1] == dir2:
            return 2
        else:
            return 3


    def _calcPassTime(self, order):
        timeCost = 0
        for vid in order:
            # obtain every car's passing time cost and get max value
            turnFlag = self._testTurn(vid)

            if self.vehsInfo[vid][2] == "A":
                if turnFlag == 1:
                    timeCost = max(timeCost, (200 - self.vehsInfo[vid][1] + 35) / traci.vehicle.getAllowedSpeed(vid))
                elif turnFlag == 2:
                    timeCost = max(timeCost, (200 - self.vehsInfo[vid][1] + 30) / 4.0)
                else:
                    timeCost = max(timeCost, (200 - self.vehsInfo[vid][1] + 28) / 4.0)

            elif self.vehsInfo[vid][2] == "B":
                leader = traci.vehicle.getLeader(vid, 100.0)[0]
                this_leader_dist = traci.vehicle.getLanePosition(leader) - traci.vehicle.getLanePosition(vid)
                if turnFlag == 1:
                    deltaT = this_leader_dist / traci.vehicle.getAllowedSpeed(vid)
                    timeCost = max(timeCost, deltaT + (200 - self.vehsInfo[vid][1] + 35) / traci.vehicle.getAllowedSpeed(vid))
                else:
                    # slow to turn speed
                    maxDecel = traci.vehicle.getDecel(vid)
                    v = traci.vehicle.getSpeed(vid)
                    t = (v - 4.0) / maxDecel
                    x = (v*v- 16) / (2*maxDecel)
                    if turnFlag == 2:
                        timeCost = max(timeCost, t + (200 - self.vehsInfo[vid][1] + 30 - x) / 4.0)
                    else:
                        timeCost = max(timeCost, t + (200 - self.vehsInfo[vid][1] + 28 - x) / 4.0)

            elif self.vehsInfo[vid][2] in ["C", "D"]:
                info = copy.deepcopy(self.vehsInfo)
                del info[vid]
                if turnFlag == 1:
                    for possibleVeh in [k for k,v in info.items() if v[2] in ["C", "D"] and k != vid]:
                        if self._testTwoVehConflict(vid, possibleVeh) and self.vehsInfo[vid][1] < self.vehsInfo[possibleVeh][1]:
                            dummyVeh = VirtualVehicle(vid, possibleVeh, self.delay, self.vehsInfo)
                            mapLocation, _ = dummyVeh.location()
                            if mapLocation > traci.vehicle.getLanePosition(vid):
                                timeCost = max(timeCost, dummyVeh.timeEstimate(self.vehsInfo[vid][2]))
        return timeCost


    def simulationTest(self):
        '''
        make one simulation test for a certain passing order
        :return:
        '''
        if self.vehsInfo:
            orders = self.tree.legal_orders()
            timeCost = [self._calcPassTime(order) for order in orders]
            if timeCost:
                index = timeCost.index(max(timeCost))
                return orders[index]


    def controlByOrder(self, order):
        for vid in order:
            turnFlag = self._testTurn(vid) == 1
            if self.vehsInfo[vid][2] == "A":
                if turnFlag == 1:
                    speedUp(vid)
                else:
                    slowDown(vid, 4.0)
            elif self.vehsInfo[vid][2] == "B":
                if turnFlag != 1:
                    slowDown(vid, 4.0)
            elif self.vehsInfo[vid][2] in ["C", "D"]:
                for possibleVeh in [k for k, v in self.vehsInfo.items() if k != vid]:
                    if self._testTwoVehConflict(vid, possibleVeh) and self.vehsInfo[vid][1] <self.vehsInfo[possibleVeh][1]:
                        dummyVeh = VirtualVehicle(vid, possibleVeh, self.delay, self.vehsInfo)
                        dummyVeh.control(self.vehsInfo[vid][2])


    def simOneStep(self):
        self._getVehsInfo()
        laneChangeBan()
        self._caseClassify()
        self.treeGenerate()
        print(self.vehsInfo)
        bestOrder = self.simulationTest()
        if bestOrder:
            self.controlByOrder(bestOrder)



if __name__ == "__main__":
    PORT = 8813
    sumoBinary = "D:\\sumo\\bin\\sumo-gui"      # your sumo binary path
    cfg_filepath = "../sumoFiles\\no_signal\\intersection.sumocfg"
    sumoProcess = subprocess.Popen([sumoBinary, "-c", cfg_filepath, "--remote-port", str(PORT)],
        stdout=sys.stdout, stderr=sys.stderr)
    traci.init(PORT)
    t = BasicPlanning()
    t.setIgnoreTurnRight(1)
    for sim_step in range(2000):
        traci.simulationStep()
        t.simOneStep()
    traci.close()
    # save to result file
    # np.savetxt('../results/pruneVehsCount.txt', vehCount)
    # np.savetxt('../results/pruneCost.txt', cost)
