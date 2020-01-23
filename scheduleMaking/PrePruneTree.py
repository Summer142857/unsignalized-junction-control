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

    def _build(self, currentNode, vehList, flag0 = 0):
        '''
        :param vehList: A dict, keys is the set of vehicles, value is a tuple which represents (lane, position)
        :param currentNode: The current node in the tree
        :return: The tree after building
        '''
        pruneList = self.obtainPruneList()
        s = [currentNode.tag.find(vid) for vid in vehList]    # the quit contidion in recursion
        if (np.array(s) >= 0).all():
            return
        for vehId in vehList:
            flag = 0     # identify whether build this node
            if vehId not in currentNode.tag:
                if currentNode.is_root:
                    prefix = currentNode.tag.replace("Root", "")
                else:
                    prefix = currentNode.tag
                for subList in pruneList:
                    for index in range(1, len(subList)):
                        if prefix + vehId == subList[index]:
                            flag = 1
                        else:
                            pattern = subList[index] + ".*" + subList[0]
                            print(pattern)
                            if re.search(pattern, prefix + vehId):
                                print(prefix + vehId)
                                print("================")
                                flag = 1
                if flag == 1:
                    continue
                else:
                    self.tree.create_node(prefix + vehId + "-", prefix + vehId, parent=currentNode)
        self.show()
        for node in self.tree.all_nodes():
            if node.is_leaf:
                try:
                    self._build(currentNode=node, vehList=vehList, flag0 = 0)
                except:
                    pass

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
        self._build(self.root, self.vehList)

    def show(self):
        self.tree.show()

    def leaves(self):
        '''
        :return: All the plan for vehicle passing currently.
        '''
        all_nodes = self.tree.all_nodes()
        return [node.tag for node in all_nodes if node.is_leaf()]

    def leaf_num(self):
        return len(self.leaves())

if __name__ == "__main__":
    vehsInfo = {"A":("u", 100), "B":("u", 90), "C":("b", 80), "D":("b", 74), "E":("u", 98)}
    vehList = list(vehsInfo.keys())
    VTree = PrePruneTree(vehsInfo)
    VTree.build()
    VTree.show()