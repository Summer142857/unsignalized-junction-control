import traci

LANE_LENGTH = 200
DIST = 50

def _testLeader(vid):
    '''
    test whether the vehicle vid exists a leader in Buffed Area
    :return:boolean  (if exists a leader, return True)
    '''
    vPos = traci.vehicle.getLanePosition(vid)  # current vehicle's position on the lane
    leader = traci.vehicle.getLeader(vid)
    flag = False
    if leader is not None:
        interval = leader[1]  # the safe distance between vehicle and its leader
        vbPos = vPos - (LANE_LENGTH - DIST)
        if vbPos + interval < DIST:
            flag = True
    return flag

def laneChangeBan():
    '''
    Forbidden lane change in Buffered Area
    '''
    vids = traci.vehicle.getIDList()
    for vid in vids:
        traci.vehicle.setLaneChangeMode(vid, 0x000001000101)
    return None

def moveToBar(vehsInfo):
    '''
    For the vehicles stop in front of the stop bar, let them move to the bar
    :param vehsInfo: The vehicles' information dict generate by strategy
    '''
    for vid in vehsInfo.keys():
        if traci.vehicle.getSpeed(vid) == 0.0:
            if not _testLeader(vid):
                position = traci.vehicle.getLanePosition(vid)
                if LANE_LENGTH - position > 10.0:
                    traci.vehicle.setSpeed(vid, 3.0)
                    traci.vehicle.slowDown(vid, 0, 2*(LANE_LENGTH - position) / 3.0)
