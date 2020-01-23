import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np

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
        g = sns.FacetGrid(newDf, row="output", col="input", margin_titles=True)
        bins = np.linspace(5, 25, 36)
        g.map(plt.hist, "travelTime", color="steelblue", bins=bins)
    plt.show()

