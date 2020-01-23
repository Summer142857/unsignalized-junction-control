import traci

def laneChangeBan(vehsInfo):
    '''
    Forbidden lane change in Buffered Area
    :param vehsInfo: The vehicles' information dict generate by strategy
    '''
    vids = list(vehsInfo.keys())
    for vid in vids:
        traci.vehicle.couldChangeLane(vid, 0)
    return None