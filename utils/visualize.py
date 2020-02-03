import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
from sklearn.preprocessing import MinMaxScaler

def travelTimeVis(data):
    sns.set_style("whitegrid")
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False

    # all directions' data
    if isinstance(data, list):
        sns.distplot(data, kde=False)

    # plot every direction's data with facet grid
    if isinstance(data, dict):
        newDf = pd.DataFrame(columns=['flowID', 'travelTime'])
        for fid in data.keys():
            timeData = data[fid]
            fidCopy = [fid for i in range(len(timeData))]
            tmp = pd.DataFrame({'flowID':fidCopy, 'travelTime':timeData})
            newDf = newDf.append(tmp, ignore_index=True)
        newDf['input'] = newDf.flowID.apply(lambda x : x.split('To')[0])
        newDf['output'] = newDf.flowID.apply(lambda x : x.split('To')[1])
        newDf.input = newDf.input.apply(replaceDir)
        newDf.output = newDf.output.apply(replaceDir)
        g = sns.FacetGrid(newDf, row="output", col="input", margin_titles=True)
        bins = np.linspace(5, 25, 36)
        g.map(plt.hist, "travelTime", color="steelblue", bins=bins)
    plt.show()

def replaceDir(dir):
    if dir.lower() == "up":
        str = dir.replace(dir, "North")
    elif dir.lower() == "below":
        str = dir.replace(dir, "South")
    elif dir.lower() == "left":
        str = dir.replace(dir, "West")
    elif dir.lower() == "right":
        str = dir.replace(dir, "East")
    else:
        raise Exception("argument format error!")
    return str

def treeBuildTime(prunePath, basicPath, countPath):
    '''
    :param prunePath: The pre-prune tree's time cost result path
    :param basicPath: The basic tree's time cost result path
    :param countPath:  The count of vehicles in each simulation step
    '''
    vehCount = []
    with open(countPath) as f1:
        for line in f1:
            vehCount.append(float(line))
    f1.close()

    prune_cost = []
    with open(prunePath) as f2:
        for line in f2:
            prune_cost.append(float(line))
    f2.close()

    basic_cost = []
    with open(basicPath) as f3:
        for line in f3:
            basic_cost.append(float(line))
    f3.close()

    build_method = ['Basic' for i in range(len(basic_cost))] + ['Pre-prune' for j in range(len(prune_cost))]
    data = pd.DataFrame({'vehCount':vehCount + vehCount.copy(), 'timeCost': basic_cost + prune_cost, "method": build_method})

    sns.boxplot(x="vehCount", y="timeCost", hue="method", data=data)
    plt.show()
    return data

if __name__ == "__main__":
    dd = treeBuildTime('../results/pruneCost.txt',
                  '../results/basicCost.txt',
                  '../results/pruneVehsCount.txt')
