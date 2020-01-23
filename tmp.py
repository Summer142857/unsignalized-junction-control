import traci
import subprocess
import sys

PORT = 8813
sumoBinary = "D:\\sumo\\bin\\sumo-gui"  # your sumo binary path
cfg_filepath = "sumoFiles\\intersection.sumocfg"
sumoProcess = subprocess.Popen([sumoBinary, "-c", cfg_filepath, "--remote-port", str(PORT)],
                               stdout=sys.stdout, stderr=sys.stderr)
traci.init(PORT)
for step in range(2000):
    traci.simulationStep()
    for vid in traci.vehicle.getIDList():
        print(vid)
        print(traci.vehicle.getLaneID(vid))
        print("=============")