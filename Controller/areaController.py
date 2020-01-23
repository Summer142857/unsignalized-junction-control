import traci

def laneChangeBan():
    '''
    Forbidden lane change in Buffered Area
    :param vehsInfo: The vehicles' information dict generate by strategy
    '''
    vids = traci.vehicle.getIDList()
    for vid in vids:
        traci.vehicle.couldChangeLane(vid, 0)
    return None