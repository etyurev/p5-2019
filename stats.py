import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
import seaborn as sns

log_path = "logs/"


npArr = np.zeros(0)
print(npArr)
for f in os.listdir(log_path):
    file = np.load((log_path + f))
    npArr = np.concatenate((npArr, file))


print(npArr.shape)
data = {"V": npArr}

df = pd.DataFrame(data=data)
print(df)

print("mean: ", df.mean())
print("std: ", df.std())
print("median: ", df.median())
print("min: ", df.min())
print("max: ", df.max())
#sns.distplot(npArr, kde=True, rug=False);
df.hist(bins=100)
plt.title("Error distribution of predictions")
plt.xlabel("Error (mm)")
plt.ylabel("Frequency")
plt.savefig("stats.pdf", format='pdf')
plt.show()

#sns.catplot(data=df, s=3)
sns.catplot(data=df, kind="box")
#plt.ylabel("Error (mm)")
#plt.hlines(-DEVIATION_LIMIT, 0, 4,colors = 'r',linestyles = 'dashed')
#plt.hlines(DEVIATION_LIMIT, 0, 4,colors = 'r',linestyles = 'dashed')
#plt.title("Deviation error test")
plt.show()
