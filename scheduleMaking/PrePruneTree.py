import traci
import sys
import subprocess
from treelib import Node, Tree
import numpy as np
from operator import itemgetter
import re

class PrePruneTree:
    def __init__(self, vehsInfo):
        self.tree = Tree()
        self.root = self.tree.create_node("Root", "root")    # root node
        self.vehsInfo = vehsInfo
        self.vehList = list(vehsInfo.keys())
        self.pruneList = None

    def _build(self, currentNode, vehList, flag0 = 0):
        '''
        :param vehList: A dict, keys is the set of vehicles, value is a tuple which represents (lane, position)
        :param currentNode: The current node in the tree
        '''
        s = [currentNode.tag.find(vid) for vid in vehList]    # the quit condition in recursion
        if (np.array(s) >= 0).all() or self._testIllegel(currentNode.tag):
            return
        for vehId in vehList:
            if vehId not in currentNode.tag:
                if currentNode.is_root:
                    prefix = currentNode.tag.replace("Root", "")
                else:
                    prefix = currentNode.tag
                self.tree.create_node(prefix + vehId + "-", prefix + vehId, parent=currentNode)
        # self.show()
        for node in self.tree.all_nodes():
            if node.is_leaf and not self._testPrePrune(node.tag):
                try:
                    self._build(currentNode=node, vehList=vehList, flag0 = 0)
                except:
                    pass

    def _testIllegel(self, tag):
        '''
        test whether need to stop recursion
        :param tag: Node tag
        :return: boolean (if true, the recursion will stop)
        '''
        # upToRight.1-leftToBelow.18-belowToRight.2-belowToRight.3-
        flag = False
        tmp = tag.split("-")
        try:
            tmp.remove('')
        except:
            pass
        if len(tmp) < len(list(self.vehsInfo.keys())) - 1:
            return flag
        surplusVeh = list(set(self.vehList) - set(tmp))
        for veh1 in surplusVeh:
            for veh2 in list(set(self.vehsInfo.keys()) - set(tmp)):
                for subList in self.pruneList:
                    if surplusVeh in subList and veh2 in subList:
                        if subList.index(veh2) > subList.index(veh1):
                            flag = True
        return flag

    def _testPrePrune(self, tag):
        '''
        test whether need to preprune
        :param tag:
        :return:
        '''
        flag = False
        vehs = tag.split("-")
        try:
            vehs.remove('')
        except:
            pass
        for veh1 in vehs:
            for veh2 in list(set(self.vehsInfo.keys()) - set(vehs)):
                for subList in self.pruneList:
                    if veh1 in subList and veh2 in subList:
                        if subList.index(veh2) < subList.index(veh1):
                            flag = True
        return flag

    def obtainPruneList(self):
        laneId = [value[0] for value in self.vehsInfo.values()]
        sortedList = []
        for i in list(set(laneId)):
            lane_info = {k: v[1] for k, v in self.vehsInfo.items() if v[0] == i}
            # Vehicles in front are at the front of the lane
            sortedList.append([vid[0] for vid in sorted(lane_info.items(), key=itemgetter(1), reverse=True)])
        pruneList = [sublist for sublist in sortedList if len(sublist) > 1]
        return pruneList

    def build(self):
        self.pruneList = self.obtainPruneList()
        self._build(self.root, self.vehList)

    def show(self):
        self.tree.show()

    def _leaves(self):
        '''
        :return: All the plan for vehicle passing currently.
        '''
        all_nodes = self.tree.all_nodes()
        return [node for node in all_nodes if node.is_leaf()]

    def legal_orders(self):
        leaves = self._leaves()
        orders = []
        for pattern in leaves:
            # upToRight.1-leftToBelow.18-belowToRight.2-belowToRight.3-
            tmp = pattern.tag.split("-")
            try:
                tmp.remove('')
            except:
                pass
            if len(tmp) == self.tree.depth():
                orders.append(tmp)
        return orders


if __name__ == "__main__":
    vehsInfo = {'upToBelow.0': ['u_1', 198.86, 'C'], 'leftToUp.0': ['l_2', 199.88047864152406, 'C'], 'leftToUp.1': ['l_2', 183.38930919344475, 'C'], 'leftToRight.0': ['l_1', 153.85795824218027, 'C']}
    VTree = PrePruneTree(vehsInfo)
    VTree.build()
    VTree.show()
