import traci
import numpy as np

def speedUp(vid):
    '''
    let vehicle speed up to its max allowed speed
    :param vid: vehicle id in simulation
    :return: the time when vehicle entry the intersection
    '''
    lane_id = traci.vehicle.getLaneID(vid)
    maxAllowedSpeed = traci.lane.getMaxSpeed(lane_id)
    nowSpeed = traci.vehicle.getSpeed(vid)
    maxA = traci.vehicle.getAccel(vid)
    time_interval1 = (maxAllowedSpeed - nowSpeed) / maxA
    traci.vehicle.slowDown(vid, maxAllowedSpeed, time_interval1)
    try:
        time_interval2 = (200 - traci.vehicle.getLanePosition()) / maxAllowedSpeed
        arrTime = traci.simulation.getTime() + time_interval1 + time_interval2
    except:
        arrTime = traci.simulation.getTime() + time_interval1
    return arrTime

def slowDown(vid, turnVelocity):
    '''
    :param vid: vehicle id in simulation
    :param turnVelocity: the expected velocity when vehicle pass the intersection with turning
    :return: the time when vehicle entry the intersection
    '''
    nowSpeed = traci.vehicle.getSpeed(vid)
    lanePostion = traci.vehicle.getLanePosition(vid)
    remainder_min = (nowSpeed * nowSpeed - turnVelocity * turnVelocity) / (2 * traci.vehicle.getDecel(vid))
    remainder = 200 - lanePostion   # the leftover distance between now position and the intersection
    if remainder_min < remainder:
        expectedA = (nowSpeed * nowSpeed - turnVelocity * turnVelocity) / (2 * remainder)
        time_interval = (nowSpeed - turnVelocity) / expectedA
        traci.vehicle.slowDown(vid, turnVelocity, time_interval)
    else:
        maxA = traci.vehicle.getDecel(vid)
        time_interval = (nowSpeed - turnVelocity) / maxA
        traci.vehicle.slowDown(vid, turnVelocity, time_interval)
        vExpected = np.sqrt(nowSpeed * nowSpeed - 2 * maxA * remainder)
        time_interval = (nowSpeed - vExpected) / maxA
    arrTime = traci.simulation.getTime() + time_interval
    return arrTime