import os
import io
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
kk=np.array([current,voltage],dtype=np.float32).T
kk=kk[:1000,:]

kk.tofile('data/thomas.bin')

os.system('bin2c data/thomas.bin -H data/thomas.h')
