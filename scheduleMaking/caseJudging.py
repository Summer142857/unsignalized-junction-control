import pandas as pd
from scheduleMaking.BasicTreeSearch import BasicTree
import json

def read_config():
    filename = "../noConflictConfig.txt"
    file = open(filename, 'r')
    js = file.read()
    dic = json.loads(js)
    file.close()
    return dic

def case1Match(schemes, ruleDic):
    flag = True    # identify if all the flow can match legal pattern
    flowIds = [veh.split('.')[0] for veh in schemes[0].split('-') if veh != '']
    for flow1 in flowIds:
        for flow2 in flowIds:
            if flow2 != flow1 and flow2 not in ruleDic[flow1]:
                flag = False
    return flag

def case2Match(schemes, ruleDic):
    flag = True
    flow_list = [flow.split('.')[0] for flow in schemes[0].split('-') if flow != '']
    flow_set = list(set(flow_list))
    for flow1 in flow_set:
        for flow2 in flow_list:
            if flow2 != flow1 and flow2 not in ruleDic[flow1]:
                flag = False
    return flag

def case3Match(schemes, ruleDic):
    flag = True
    flow_list = [flow.split('.')[0] for flow in schemes[0].split('-') if flow != '']
    print(flow_list)
    flow_set = list(set(flow_list))
    print(flow_set)
    if sorted(flow_list) != sorted(flow_set):
        flag = False
    return flag

def caseClassify(vehsInfo, ruleDic):
    '''
    :param vehsInfo: The information of vehicles in Buffered Area
    :param rule: The legal phase for different flow
    :return:
    '''
    vehTree = BasicTree(vehsInfo)
    vehTree.build()
    schemes = vehTree.leaves()
    print(schemes)
    if len(schemes) > 1 or (len(schemes) == 1 and schemes[0] != 'Root'):
        if len(schemes) == 1 or case1Match(schemes, ruleDic):
            # just one car or match free pass pattern: case1 (free pass)
            return 1
        elif case2Match(schemes, ruleDic):
            # A leading vehicle and no foe vehicles to map: case2 (car following)
            return 2
        elif case3Match(schemes, ruleDic):
            # No leading vehicle but foe vehicles
            return 3
        else:
            return 4
    return 0

# !! A bug: 相同车道上不同去向的车，判断case4时会判为case3