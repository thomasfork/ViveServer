
import numpy as np

label = 'T_1'

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

from vive_server.vive_client import ViveClient, ViveConfig

config = ViveConfig(label = label)
client = ViveClient(config)

pts = 4000

data = np.zeros((pts,3))

for j in range(pts):
    state = client.step()
    if state:
        data[j] = np.array([state.xi, state.xj, state.xk])
        
    print('%4d/%4d'%(j,pts), end = '\r')
  


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(data[:,0], data[:,1], data[:,2])
plt.show()

np.savez('track_shape.npz',data = data)
