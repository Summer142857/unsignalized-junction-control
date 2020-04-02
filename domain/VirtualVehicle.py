import traci
import sympy
import numpy as np
from domain.const import *

class VirtualVehicle(object):

    def __init__(self, thisVeh, mappingVeh, delay, vehsInfo):
        '''
        :param thisVeh: vid belong to the vehicle on the lane
        :param mappingVeh: vid belong to the vehicle needed to be mapping to other lane
        :param delay: consider the time cost cause by communication latency
        '''
        self.thisVeh = thisVeh
        self.mappingVeh = mappingVeh
        self.thisVehDir = None
        self.mappingVehDir = None
        self.delay = delay
        self.LANE_LENGTH = 200
        self.safeGap = traci.vehicle.getMinGap(self.thisVeh)
        self.vehsInfo = vehsInfo
        self.dirJudge()


    def dirJudge(self):
        dir1, dir2 = self.mappingVeh.split('.')[0].split('To')
        # judge if the vehicle needs to turn in the intersection
        straight_dict = {'left': 'Right', 'right':'Left', 'up': 'Below', 'below':'Up'}
        longTurn_dict = {'left': 'Up', 'right':'Below', 'up':'Right', 'below':'Left'}
        if straight_dict[dir1] == dir2:
            # go straight
            self.mappingVehDir = 1
        elif longTurn_dict[dir1] == dir2:
            # long turn
            self.mappingVehDir = 2
        else:
            # short turn
            self.mappingVehDir = 3
        dir1, dir2 = self.thisVeh.split('.')[0].split('To')
        if straight_dict[dir1] == dir2:
            # go straight
            self.thisVehDir = 1
        elif longTurn_dict[dir1] == dir2:
            # long turn
            self.thisVehDir = 2
        else:
            # short turn
            self.thisVehDir = 3


    def location(self):
        '''
        :return: mapLocation: the mapping vehicle's map location
                 flag: identify if two vehicle have safe gap
        '''
        delayGap = traci.vehicle.getSpeed(self.mappingVeh) * self.delay
        cruiseTimeGap = 0
        if self.mappingVehDir == 1:
            cruiseTimeGap = STRAIGHT / traci.vehicle.getAllowedSpeed(self.mappingVeh)
        elif self.mappingVehDir == 2:
            cruiseTimeGap = LEFTTURN / 4.0
        elif self.mappingVehDir == 3:
            cruiseTimeGap = RIGHTTURN / 4.0

        cruiseGap = traci.vehicle.getSpeed(self.mappingVeh) * cruiseTimeGap
        totalGap = delayGap + cruiseGap
        mapLocation = traci.vehicle.getLanePosition(self.mappingVeh) - totalGap
        # calculate the minimal location for mappingVeh

        thisVehLocation = traci.vehicle.getLanePosition(self.thisVeh)
        minLocation = thisVehLocation + self.safeGap
        flag = True             # identify whether the two vehicle have safe gap
        if minLocation > mapLocation:
            flag = False
        return mapLocation, flag


    def _adjustGapSim(self, mapLocation, thisVehLocation):
        maxDecel = traci.vehicle.getDecel(self.thisVeh)
        deltaX = self.safeGap - (mapLocation - thisVehLocation)
        v0 = traci.vehicle.getSpeed(self.thisVeh)
        t = sympy.symbols('t')
        try:
            decelT = np.min(np.array([x for x in sympy.solve(v0 * t - maxDecel * t * t / 2 - deltaX, t) if x > 0]))
        except:
            decelT = 0.1
        v = v0 - maxDecel * decelT
        decelX = (v0 * v0 - v * v) / (2 * maxDecel)
        return decelT, decelX, v


    def adjustGapSim(self):
        maxDecel = traci.vehicle.getDecel(self.thisVeh)
        # the mapping vehicle is at the behind of thisVeh
        deltaT = traci.simulation.getDeltaT()
        expectV = traci.vehicle.getSpeed(self.thisVeh) - maxDecel * deltaT
        traci.vehicle.setSpeed(self.thisVeh, max(expectV, 0))


    def _adjustToFollow(self, thisSpeed, expectSpeed):
        if thisSpeed < expectSpeed:
            maxAcc = traci.vehicle.getAccel(self.thisVeh)
            t = (expectSpeed - thisSpeed) / maxAcc
            x = thisSpeed * t + (maxAcc * t * t) / 2
        else:
            maxDecel = traci.vehicle.getDecel(self.thisVeh)
            t = (thisSpeed - expectSpeed) / maxDecel
            x = thisSpeed * t - (maxDecel * t * t) / 2
        return t, x


    def adjustToFollow(self, thisVehLocation, mapLocation):
        deltaX = mapLocation - self.safeGap - thisVehLocation
        if deltaX > 0:
            speed = traci.vehicle.getSpeed(self.thisVeh)
            deltaT = traci.simulation.getDeltaT()
            a = 2 * (deltaX - speed * deltaT) / deltaT * deltaT
            traci.vehicle.slowDown(self.thisVeh, speed + a * deltaT, deltaT)


    def _slow4Turn(self, v):
        maxDecel = traci.vehicle.getDecel(self.thisVeh)
        t = (v - 4.0) / maxDecel
        x = (v*v- 16) / (2*maxDecel)
        return t, x


    def slow4Turn(self, v):
        maxDecel = traci.vehicle.getDecel(self.thisVeh)
        t = abs(v - 4.0) / maxDecel
        traci.vehicle.slowDown(self.thisVeh, 4.0, t)


    def _estimateCaseC(self, thisVehLocation, thisSpeed, mapLocation, flag):

        if self.mappingVehDir == 1:
            if self.thisVehDir == 1:
                if not flag:
                    decelT, decelX, v = self._adjustGapSim(mapLocation, thisVehLocation)
                    passT = (200 - thisVehLocation - decelX + STRAIGHT) / v
                    return decelT + passT
                else:
                    return (200 - thisVehLocation + STRAIGHT) / traci.vehicle.getAllowedSpeed(self.thisVeh)

            elif self.thisVehDir == 2:
                if not flag:
                    decelT, decelX, v = self._adjustGapSim(mapLocation, thisVehLocation)
                    slowT, slowX = self._slow4Turn(v)
                    passT = (200 - thisVehLocation - decelX - slowX + LEFTTURN) / 4.0
                    return decelT + slowT + passT
                else:
                    slowT, slowX = self._slow4Turn(traci.vehicle.getSpeed(self.thisVeh))
                    passT = (200 - thisVehLocation - slowX + LEFTTURN) / 4.0
                    return passT + slowT

            elif self.thisVehDir == 3:
                if not flag:
                    decelT, decelX, v = self._adjustGapSim(mapLocation, thisVehLocation)
                    slowT, slowX = self._slow4Turn(v)
                    passT = (200 - thisVehLocation - decelX - slowX + RIGHTTURN) / 4.0
                    return decelT + slowT + passT
                else:
                    slowT, slowX = self._slow4Turn(traci.vehicle.getSpeed(self.thisVeh))
                    passT = (200 - thisVehLocation - slowX + RIGHTTURN) / 4.0
                    return passT + slowT
        else:
            # thisVeh is supposed to follow mapping vehicle all th time
            if not flag:
                decelT, decelX, v = self._adjustGapSim(mapLocation, thisVehLocation)
                adjustT, adjustX = self._adjustToFollow(thisSpeed, 4.0)
                if self.thisVehDir == 1:
                    passT = (200 - thisVehLocation - decelX - adjustX + STRAIGHT) / 4.0
                    return passT + adjustT + decelT

                elif self.thisVehDir == 2:
                    passT = (200 - thisVehLocation - decelX - adjustT + LEFTTURN) / 4.0
                    return passT + adjustT + decelT

                else:
                    passT = (200 - thisVehLocation - decelX - adjustT + RIGHTTURN) / 4.0
                    return passT + adjustT + decelT
            else:
                decelT, decelX = self._slow4Turn(thisSpeed)
                if self.thisVehDir == 1:
                    passT = (200 - thisVehLocation - decelX + STRAIGHT) / 4.0
                elif self.thisVehDir == 2:
                    passT = (200 - thisVehLocation - decelX - + LEFTTURN) / 4.0
                else:
                    passT = (200 - thisVehLocation - decelX + RIGHTTURN) / 4.0
                return decelT + passT



    def _estimateCaseD(self, thisVehLocation, thisSpeed, mapLocation, flag):
        this_map_gap = mapLocation - traci.vehicle.getLanePosition(self.thisVeh)
        potentialLeader = traci.vehicle.getLeader(self.thisVeh)[0]
        if traci.vehicle.getLaneID(potentialLeader) != self.vehsInfo[self.thisVeh][0]:
            return self._estimateCaseC(thisVehLocation, thisSpeed, mapLocation, flag)
        leaderLocation = traci.vehicle.getLanePosition(potentialLeader)
        this_leader_distance = leaderLocation - thisVehLocation
        if this_leader_distance < this_map_gap:
            # follow the leader
            if self.vehsInfo[potentialLeader]:
                # if the leader is on the lane instead of the junction
                if self.vehsInfo[potentialLeader][2] in ['A', 'B']:
                    if self.thisVehDir == 1:
                        passT = (STRAIGHT + 200 - thisVehLocation) / traci.vehicle.getAllowedSpeed(self.thisVeh)
                        return passT
                    elif self.thisVehDir == 2:
                        decelT, decelX = self._slow4Turn(thisSpeed)
                        passT = (LEFTTURN + 200 - thisVehLocation - decelX) / 4.0
                        return passT + decelT
                    else:
                        decelT, decelX = self._slow4Turn(thisSpeed)
                        passT = (RIGHTTURN + 200 - thisVehLocation) / 4.0
                        return passT + decelT
                else:
                    leaderSpeed = traci.vehicle.getSpeed(potentialLeader)
                    tmp = self.thisVeh
                    self.thisVeh = potentialLeader
                    mapLocation = self.location()
                    self.thisVeh = tmp
                    leaderT = self._estimateCaseC(leaderLocation, leaderSpeed, mapLocation, flag)
                    if self.thisVehDir == 1:
                        if thisSpeed != 0:
                            deltaT = this_leader_distance / thisSpeed
                        else:
                            deltaT = 0
                    else:
                        t, x = self._slow4Turn(thisSpeed)
                        if x < this_leader_distance:
                            deltaT = t + this_leader_distance / 4.0
                        else:
                            t = sympy.symbols('t')
                            deltaT = min([x for x in sympy.solve(
                                thisSpeed * t - traci.vehicle.getDecel(self.thisVeh) * t * t / 2 - this_leader_distance, t) if x > 0])
                    return deltaT + leaderT

            else:
                # free pass
                if self.thisVehDir == 1:
                    passT = (STRAIGHT + 200 - thisVehLocation) / traci.vehicle.getAllowedSpeed(self.thisVeh)
                    return passT
                elif self.thisVehDir == 2:
                    decelT, decelX = self._slow4Turn(thisSpeed)
                    passT = (LEFTTURN + 200 - thisVehLocation - decelX) / 4.0
                    return passT + decelT
                else:
                    decelT, decelX = self._slow4Turn(thisSpeed)
                    passT = (RIGHTTURN + 200 - thisVehLocation) / 4.0
                    return passT + decelT
        else:
            # follow mapping vehicle
            return self._estimateCaseC(thisVehLocation, thisSpeed, mapLocation, flag)


    def timeEstimate(self, case):

        thisVehLocation = traci.vehicle.getLanePosition(self.thisVeh)
        thisSpeed = traci.vehicle.getSpeed(self.thisVeh)
        mapLocation, flag = self.location()

        if case == 'C':
            return self._estimateCaseC(thisVehLocation, thisSpeed, mapLocation, flag)

        elif case == 'D':
            return self._estimateCaseD(thisVehLocation, thisSpeed, mapLocation, flag)


    def controlCcase(self, thisVehLocation, thisSpeed, mapLocation, flag):
        if not flag:
            # In every step, thisVeh slow down to adjust gap
            self.adjustGapSim()
        else:
            self.adjustToFollow(thisVehLocation, mapLocation)
        if self.thisVehDir != 1:
            self.slow4Turn(thisSpeed)


    def controlDcase(self, thisVehLocation, thisSpeed, mapLocation, flag):
        potentialLeader = traci.vehicle.getLeader(self.thisVeh)[0]
        leaderLocation = traci.vehicle.getLanePosition(potentialLeader)
        this_leader_distance = leaderLocation - thisVehLocation
        this_map_distance =  mapLocation - thisVehLocation

        if this_map_distance <= this_leader_distance:
            self.controlCcase(thisVehLocation, thisSpeed, mapLocation, flag)
        else:
            if self.thisVehDir != 1:
                self.slow4Turn(thisSpeed)


    def control(self, case):

        thisVehLocation = traci.vehicle.getLanePosition(self.thisVeh)
        thisSpeed = traci.vehicle.getSpeed(self.thisVeh)
        mapLocation, flag = self.location()

        if case == 'C':
            self.controlCcase(thisVehLocation, thisSpeed, mapLocation, flag)
        if case == 'D':
            self.controlDcase(thisVehLocation, thisSpeed, mapLocation, flag)
