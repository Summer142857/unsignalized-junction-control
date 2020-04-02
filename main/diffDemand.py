from Controller.FCFS import FcfsController
from Controller.BacisPlanning import BasicPlanning
from Controller.SignalController import SignalController
from utils.alterXML import alterDemand
import pandas as pd
import numpy as np
import subprocess
import traci
import sys
import os


# simulation port and software
PORT = 8813
sumoBinary = "D:\\sumo\\bin\\sumo-gui"
# simulation step
EPOCH = 3600

# config file
rou_path = '../sumoFiles/no_signal/intersection.rou.xml'
auto_cfg_filepath = '../sumoFiles/no_signal/intersection.sumocfg'
signal_cfg_filepath = '../sumoFiles/signal_intersection/junction.sumocfg'
result_file = '../results/demand_compare.csv'

def simOnce(demand_factor, controller):
    if str(controller) != 'signal':
        sumoProcess = subprocess.Popen([sumoBinary, "-c", auto_cfg_filepath, "--remote-port", str(PORT)],
            stdout=sys.stdout, stderr=sys.stderr)
    else:
        sumoProcess = subprocess.Popen([sumoBinary, "-c", signal_cfg_filepath, "--remote-port", str(PORT)],
            stdout=sys.stdout, stderr=sys.stderr)
    traci.init(PORT)
    for sim_step in range(EPOCH):
        traci.simulationStep()
        controller.simOneStep()
    travel_time_list = controller._statistic()
    method = [str(controller)] * len(travel_time_list)
    demand = [str(demand_factor)] * len(travel_time_list)
    data_segment = pd.DataFrame({'travelTime': travel_time_list, 'method': method, 'demand': demand})
    traci.close()
    sumoProcess.kill()
    return data_segment

if __name__ == "__main__":

    # if os.path.exists(result_file):c
    #     os.remove(result_file)

    for i in np.linspace(0.04, 0.04, 1):
        alterDemand(rou_path, np.around(i, 3))

        # signal_controller = SignalController()
        # fcfs_controller = FcfsController()
        plan_controller = BasicPlanning()

        # for c in [signal_controller, fcfs_controller, plan_controller]:
        # for c in [fcfs_controller, plan_controller]:
        data_once = simOnce(np.around(i, 3), plan_controller)
        data_once.to_csv(result_file, header=False, mode='a', index=False)


