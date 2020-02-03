from treelib import Node, Tree
import numpy as np
from operator import itemgetter
import re
import numba

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
    vehsInfo = {'upToBelow.0': ['u_1', 198.86, 'C'], 'leftToUp.0': ['l_2', 199.88047864152406, 'C'],
                'leftToUp.1': ['l_2', 183.38930919344475, 'C'], 'leftToRight.0': ['l_1', 153.85795824218027, 'C']}
    VTree = BasicTree(vehsInfo)
    VTree.build()
    VTree.show()
