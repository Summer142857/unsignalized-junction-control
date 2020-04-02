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
from collections import defaultdict
from utils.visualize import travelTimeVis
import socket

vehCount = []
cost = []

class BasicPlanning:

    def __init__(self, dist=50.0):
        self.dist = dist
        self.vehsInfo = dict()
        self.tree = None
        self.in_loop = ['u_1', 'l_1', 'b_1', 'r_1', 'u_2', 'l_2', 'b_2', 'r_2','u_3', 'l_3', 'b_3', 'r_3']
        self.exit_loop = ['uo_1', 'bo_1', 'ro_1', 'lo_1', 'uo_2', 'bo_2', 'ro_2', 'lo_2', 'uo_3', 'bo_3', 'ro_3', 'lo_3']
        self.conflictDict = read_config()
        self.ignore_rightTurn = False
        self.delay = 0.5    # communication delay in vehicle mapping
        self.LANE_ID = LANE_ID
        self.travelDict = defaultdict(list)
        self.setIgnoreTurnRight(1)

    def __str__(self):
        return "planning"

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
            try:
                if traci.vehicle.getLaneID(vid) not in (self.LANE_ID):
                    del self.vehsInfo[vid]
            except Exception as e:
                print(e)


    def safeLabel(self, orders):

        labeled_orders = []

        for order in orders:
            a = []
            labeled_order = []
            for i in range(1, len(order)):
                a.append(order[i - 1])
                if self._testTwoVehConflict(order[i], order[i - 1]):
                    labeled_order.append(a)
                    a = []
            if self._testTwoVehConflict(order[len(order) - 1], order[len(order) - 2]):
                a = [order[len(order) - 1]]
                labeled_order.append(a)
            else:
                a.append(order[len(order) - 1])
                labeled_order.append(a)
            labeled_orders.append(labeled_order)
        tmp_list = list(set([str(order) for order in labeled_orders]))
        return [ast.literal_eval(item) for item in tmp_list]


    def treeGenerate(self):
        if self.vehsInfo:
            # constructor: vehsInfo
            self.tree = PrePruneTree(self.vehsInfo)
            self.tree.build()
            # self.tree.show()
            label_result = self.safeLabel(self.tree.legal_orders())
            # remove equal orders: the same safe labeled items
            orders = list(set([sorted(order) for order in label_result if all([type(item)==type('str') for item in order])]))
            orders_hat = [order for order in label_result if not all([type(item)==type('str') for item in order])]
            orders.extend(orders_hat)
            print("There're %d possible orders." %len(orders))
            return orders
        else:
            return None


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

    # do case classify according to safe label result
    def _caseClassify(self, order):
        '''
        Add case information to vehsInfo
        :return:
        '''
        if all([type(item) == type('str') for item in order]):
            for vid in order:
                if not self._testLeader(vid):
                    self.vehsInfo[vid][2] = "A"
                else:
                    self.vehsInfo[vid][2] = "B"
        else:
            for vid in order[0]:
                if not self._testLeader(vid):
                    self.vehsInfo[vid][2] = "A"
                else:
                    self.vehsInfo[vid][2] = "B"
            for vid in sum(order[1:], []):
                if not self._testLeader(vid):
                    self.vehsInfo[vid][2] = "C"
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


    def calc_case_AB(self, turn_flag, lastVeh):
        leader = traci.vehicle.getLeader(lastVeh, LANE_LENGTH - traci.vehicle.getLanePosition(lastVeh))
        if leader and traci.vehicle.getLaneID(leader[0]) == self.vehsInfo[lastVeh][0]:
            this_leader_dist = traci.vehicle.getLanePosition(leader[0]) - traci.vehicle.getLanePosition(lastVeh)
            if turn_flag == 1:
                deltaT = this_leader_dist / traci.vehicle.getAllowedSpeed(lastVeh)
                return deltaT + (200 - self.vehsInfo[lastVeh][1] + STRAIGHT) / traci.vehicle.getAllowedSpeed(lastVeh)
            else:
                # slow to turn speed
                maxDecel = traci.vehicle.getDecel(lastVeh)
                v = traci.vehicle.getSpeed(lastVeh)
                t = (v - 4.0) / maxDecel
                x = (v * v - 16) / (2 * maxDecel)
                if turn_flag == 2:
                    return t + (200 - self.vehsInfo[lastVeh][1] + LEFTTURN - x) / 4.0
                else:
                    return t + (200 - self.vehsInfo[lastVeh][1] + RIGHTTURN - x) / 4.0
        else:
            if turn_flag == 1:
                return (200 - self.vehsInfo[lastVeh][1] + STRAIGHT) / traci.vehicle.getAllowedSpeed(lastVeh)
            elif turn_flag == 2:
                return (200 - self.vehsInfo[lastVeh][1] + LEFTTURN) / 4.0
            else:
                return (200 - self.vehsInfo[lastVeh][1] + RIGHTTURN) / 4.0


    def new_calc(self, order, flag = 0):
        '''
        to estimate the pasiing time of vehicles
        :param order: illegal order of vehicles
        :param flag: identify if the function is called by recursion
        :return:
        '''
        if not flag:
            self._caseClassify(order)
        # if all vehicle formulate a CSG
        if all([type(item)==type('str') for item in order]):
            lastVeh = order[-1]
            turn_flag = self._testTurn(lastVeh)

            case = self.vehsInfo[lastVeh][2]

            if case in ["B", "A"]:
                return self.calc_case_AB(turn_flag, lastVeh)

            else:
                timeCost = 0
                info = copy.deepcopy(self.vehsInfo)
                del info[lastVeh]
                for vid in info.keys():
                    if self._testTwoVehConflict(lastVeh, vid):
                        dummyVeh = VirtualVehicle(lastVeh, vid, self.delay, self.vehsInfo)
                        mapLocation, _ = dummyVeh.location()
                        if mapLocation < self.vehsInfo[lastVeh][1]:
                            timeCost = max(dummyVeh.timeEstimate(self.vehsInfo[lastVeh][2]), timeCost)
                return timeCost
        else:
            timeCost = 0
            last_order = order[-1]
            for veh in last_order:
                # print("递归调用得到车辆{}耗费时间{}".format(veh, self.new_calc([veh])))
                timeCost = max(self.new_calc([veh], flag = 1), timeCost)
            return timeCost


    def simulationTest(self, orders):
        '''
        make one simulation test for a certain passing order
        :return:
        '''
        if orders:
            print(orders)
            timeCost = [self.new_calc(order) for order in orders]
            if timeCost:
                for order, cost in zip(orders, timeCost):
                    print(order, cost)


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
        orders = self.treeGenerate()
        self._collectTravelTime()
        self.simulationTest(orders)


    def _collectTravelTime(self):
        '''
        calculate the travel time between the start of incoming lane's Buffered Area and the exit of the junction
        :return: travel time list for all vehicles
        '''
        for ieid in self.in_loop:
            # [(veh_id, veh_length, entry_time, exit_time, vType), ...]
            for vehData in traci.inductionloop.getVehicleData(ieid):
                veh_id = vehData[0]
                exit_time = vehData[3]
                if exit_time != -1.0:
                    self.travelDict[veh_id].append(exit_time)
        for oeid in self.exit_loop:
            for vehData in traci.inductionloop.getVehicleData(oeid):
                veh_id = vehData[0]
                entry_time = vehData[2]
                if entry_time != -1.0 and veh_id in list(self.travelDict.keys()):
                    if len(self.travelDict[veh_id]) == 1:
                        self.travelDict[veh_id].append(entry_time)

    def _statistic(self, all_direction = True):
        '''
        Statistics of travel time in simulation
        :param all_direction: determine whether direction division is needed
        :return: if all_direction is True, return a list including all vehicles' travel time; otherwise, return a dict
        with different direction's travel time information
        '''
        straight_dict = {'left': 'Right', 'right':'Left', 'up': 'Below', 'below':'Up'}
        longTurn_dict = {'left': 'Up', 'right':'Below', 'up':'Right', 'below':'Left'}
        dirTravelTime = defaultdict(list)
        for vid in list(self.travelDict.keys()):
            flow_name = vid.split('.')[0]
            dir1, dir2 = flow_name.split('To')
            if len(self.travelDict[vid]) == 2:
                if straight_dict[dir1] == dir2:
                    dirTravelTime[flow_name].append(max(self.travelDict[vid][1] - self.travelDict[vid][0] - 8.0, 0.0001))
                elif longTurn_dict[dir1] == dir2:
                    dirTravelTime[flow_name].append(max(self.travelDict[vid][1] - self.travelDict[vid][0] - 19.1428, 0.0001))
                else:
                    dirTravelTime[flow_name].append(max(self.travelDict[vid][1] - self.travelDict[vid][0] - 12.8928, 0.0001))
        if all_direction:
            return sum([v for k, v in dirTravelTime.items()], [])
        return dirTravelTime


    def travelTimePlot(self, all_direction):
        statistic = self._statistic(all_direction=all_direction)
        travelTimeVis(statistic, "planning based strategy")


if __name__ == "__main__":
    socket.setdefaulttimeout(120)
    sys.setrecursionlimit(100000)
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
    # save to result file
    # np.savetxt('../results/pruneVehsCount.txt', vehCount)
    # np.savetxt('../results/pruneCost.txt', cost)
    t.travelTimePlot(all_direction=True)
