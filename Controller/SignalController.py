import traci
from collections import defaultdict
import sys
import subprocess
from utils.visualize import travelTimeVis

LANE_ID = ["l_0", "b_0", "r_0", "u_0", "l_1", "b_1", "r_1", "u_1", "l_2", "b_2", "r_2", "u_2"]   # left, below, right, up
LANE_LENGTH = 200
JUNCTION_ID = [":gneJ11_0_0", ":gneJ11_1_0", ":gneJ11_2_0", ":gneJ11_3_0",":gneJ11_4_0", ":gneJ11_5_0",":gneJ11_6_0", ":gneJ11_7_0", ":gneJ11_8_0",":gneJ11_9_0", ":gneJ11_10_0",":gneJ11_11_0"]

class SignalController(object):

    def __init__(self):
        self.in_loop = ['u_1', 'l_1', 'b_1', 'r_1', 'u_2', 'l_2', 'b_2', 'r_2','u_3', 'l_3', 'b_3', 'r_3']
        self.exit_loop = ['uo_1', 'bo_1', 'ro_1', 'lo_1', 'uo_2', 'bo_2', 'ro_2', 'lo_2', 'uo_3', 'bo_3', 'ro_3', 'lo_3']
        self.travelDict = defaultdict(list)

    def __str__(self):
        return "signal"

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

    def simOneStep(self):
        self._collectTravelTime()

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
        travelTimeVis(statistic, "Signal Control")


if __name__ == "__main__":
    PORT = 8813
    sumoBinary = "D:\\sumo\\bin\\sumo-gui"      # your sumo binary path
    cfg_filepath = "../sumoFiles\\signal_intersection\\junction.sumocfg"
    sumoProcess = subprocess.Popen([sumoBinary, "-c", cfg_filepath, "--remote-port", str(PORT)],
        stdout=sys.stdout, stderr=sys.stderr)
    traci.init(PORT)
    t = SignalController()
    for sim_step in range(600):
        traci.simulationStep()
        t.simOneStep()
    traci.close()
    t.travelTimePlot(all_direction=True)

