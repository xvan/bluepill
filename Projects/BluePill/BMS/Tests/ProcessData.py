import os
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


df = pd.read_excel('data/thomas.xlsx', index_col= 0)

df.index = df.index - df.index[0]

x=df.index.to_numpy()
y=df.to_numpy()

seconds = np.arange(np.ceil(x[-1]))
current = np.interp(seconds, x, y[:,0])
voltage = np.interp(seconds, x, y[:,1])

#plot seconds vs voltage and current
kk=kk[:7200,:]
kk=np.array([current,voltage],dtype=np.float32).T

kk.tofile('data/thomas.bin')
print(kk.shape)

# to tenerate the c file
# bin2c thomas.bin  -C thomas.c

# cast byte array to float array
#const float (* thomas)[2] = (void *) bin2c_thomas_bin;
