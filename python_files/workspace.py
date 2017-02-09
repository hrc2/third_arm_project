
from scipy.spatial import Delaunay
import numpy as np
import math
import pylab
from matplotlib import colors as mcolors
from matplotlib.collections import LineCollection
from matplotlib.collections import PolyCollection
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.tri as mtri
import ConcaveHull as concave_hull

#tri = Delaunay(pts)

def unpack(fname):
    pts = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)
    pts = pts[pts[:, 2].argsort()]
    xs = pts[:,0]
    ys = pts[:,1]
    zs = pts[:,2]
    return xs,ys,zs

def PolyArea(x,y):
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

def cc(arg):
    return mcolors.to_rgba(arg, alpha=0.6)

xr,yr,zr = unpack("v2.csv")
xh,yh,zh = unpack("human.csv")

#Calculation of v2 robot WS volume:
s = 15#**4 #Number of steps
h = (zr[-1] - zr[0])/s

# x = xr[650:2280]
# y = yr[650:2280]
# tri = mtri.Triangulation(x,y)
Area = 0.0

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
verts = []
zs = []

for i in range(1,s+1):
    args = np.where(abs(zr - (zr[0] + (i-1)*h + h/2) ) <= h)
    x = xr[args]
    y = yr[args]
    zs.append(zr[0]+(i-0.5)*h)
    hull = concave_hull.concaveHull(np.column_stack((x,y)),5)
    hull = np.vstack(hull)
    verts.append(list(hull))
    Area += PolyArea(hull[:,0], hull[:,1])
    if i==1 or i==s:
        Area *= 0.5


poly = PolyCollection(verts)
poly.set_alpha(0.7)
ax.add_collection3d(poly, zs=zs)

Volume = h*Area

print Volume

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
plt.show()

# fig = plt.figure()
# plt.gca().set_aspect('equal')
#
# #plt.triplot(tri,'go--', lw=1.0)
# plt.plot(x,y,'go')
# plt.plot(hull[:,0], hull[:,1], 'k-')
# plt.show()



# Nr = xr.size
# cr = np.array([0.0, 0.0, 1.0, 0.1])
# cr = np.tile(cr, (Nr,1))

# Nh = xh.size
# ch = np.array([1.0, 0.0, 0.0, 0.1])
# ch = np.tile(ch, (Nh,1))

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(xr, yr, zr, c='green', alpha=0.07, marker='o', depthshade=False)
# ax.scatter(xh, yh, zh, c='red', alpha=1, marker='o', depthshade=False)
# ax.set_xlabel('X (m)')
# ax.set_ylabel('Y (m)')
# ax.set_zlabel('Z (m)')
#
# plt.show()