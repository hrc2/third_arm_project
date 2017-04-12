import numpy as np
import math
import pylab
import matplotlib as mpl
from matplotlib import colors as mcolors
from scipy.spatial import ConvexHull
from matplotlib.collections import LineCollection
from matplotlib.collections import PolyCollection
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.tri as mtri
import ConcaveHull as concave_hull

def find_nearest(array,value):
    idx = (np.abs(array-value)).argmin()
    return array[idx]

patches = []
fname = 'human_concave_hull2.csv'
X = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)
hmid = find_nearest(X[:,2],0.5)
hindex = (np.where(X[:,2] == hmid))
hplot = Polygon(X[hindex[0],0:2] , ls = 'None')
patches.append(hplot)

fname = 'v1_concave_hull.csv'
Xp = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)
v1mid = find_nearest(Xp[:,2],0.5)
v1index = (np.where(Xp[:,2] == v1mid))
v1plot = Polygon(Xp[v1index[0],0:2] , ls = 'None')
patches.append(v1plot)

fname = 'v2_concave_hull.csv'
Xr = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)
v2mid = find_nearest(Xr[:,2],0.5)
v2index = (np.where(Xr[:,2] == v2mid))
v2plot = Polygon(Xr[v2index[0],0:2] , ls = 'None')
patches.append(v2plot)

fig= plt.figure()
ax = fig.gca()
#ax = fig.gca(projection='3d')
p = PatchCollection(patches,facecolors=['r','b','y'], alpha=0.3)
#poly1 = PolyCollection(patches)
#poly1.set_alpha(0.7)
#ax.add_collection3d(poly1, zs=0.5)
ax.add_collection(p)
ax.autoscale()
ax.grid(True)
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
#ax.view_init(azim=0, elev=0)
plt.show()