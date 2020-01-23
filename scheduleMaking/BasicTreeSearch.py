from treelib import Node, Tree
import numpy as np
from operator import itemgetter
import re

class BasicTree:
    def __init__(self, vehsInfo):
        self.tree = Tree()
        self.root = self.tree.create_node("Root", "root")    # root node
        self.vehsInfo = vehsInfo
        self.vehList = list(vehsInfo.keys())
    def _build(self, currentNode, vehList):
        '''
        :param vehList: A dict, keys is the set of vehicles, value is a tuple which represents (lane, position)
        :param currentNode: The current node in the tree
        :return: None
        '''
        s = [currentNode.tag.find(vid) for vid in vehList]    # the quit contidion in recursion
        if (np.array(s) >= 0).all():
            return
        for vehId in vehList:
            if vehId not in currentNode.tag:
                if currentNode.is_root:
                    prefix = currentNode.tag.replace("Root", "")
                else:
                    prefix = currentNode.tag
                self.tree.create_node(prefix + vehId + "-", prefix + vehId, parent=currentNode)
        for node in self.tree.all_nodes():
            if node.is_leaf:
                try:
                    self._build(currentNode=node, vehList=vehList)
                except:
                    pass

    def _prune(self):
        laneId = [value[0] for value in self.vehsInfo.values()]
        sortedList = []
        for i in list(set(laneId)):
            lane_info = {k: v[1] for k, v in self.vehsInfo.items() if v[0] == i}
            # Vehicles in front are at the front of the lane
            sortedList.append([vid[0] for vid in sorted(lane_info.items(), key=itemgetter(1), reverse=True)])
        pruneList = [sublist for sublist in sortedList if len(sublist) > 1]
        print(pruneList)
        for subList in pruneList:
            for index in range(1, len(subList)):
                # first, prune th subtree which begin with illegal vehicle id
                self.tree.remove_subtree(subList[index])
                # second, delete the nodes which match the illegal pattern
                pattern = subList[index] + ".*" + subList[0]
                for node in self.tree.all_nodes():
                    if re.search(pattern, node.tag):
                        try:
                            self.tree.remove_node(node.identifier)
                        except:
                            pass

    def build(self):
        self._build(self.root, self.vehList)
        self._prune()

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
    VTree = BasicTree(vehsInfo)
    VTree.build()
    VTree.show()
