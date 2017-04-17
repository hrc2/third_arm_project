import numpy as np
import math
import pylab
import matplotlib as mpl
from matplotlib import colors as mcolors
from scipy.spatial import ConvexHull
from matplotlib.collections import LineCollection
from matplotlib.collections import PolyCollection
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.tri as mtri
import ConcaveHull as concave_hull

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


def get_hull_points(xr,yr,zr,s):
    #s = 70 #Number of steps
    h = (zr[-1] - zr[0])/s

    Area = 0.0

    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection='3d')
    verts = []

    for i in range(1,s+1):
        args = np.where(abs(zr - (zr[0] + (i-1)*h + h/2) ) <= h)
        x = xr[args]
        y = yr[args]
        zs=(zr[0]+(i-0.5)*h)
        hull = concave_hull.concaveHull(np.column_stack((x,y)),5)
        hull = np.vstack(hull)
        sh = hull.shape
        verts.append(list( np.append(hull, np.vstack(np.tile(zs,sh[0])), axis=1)))
        Area += PolyArea(hull[:,0], hull[:,1])
        if i==1 or i==s:
            Area *= 0.5

    Volume = h*Area
    print Volume

    return np.vstack(verts)
    # poly = PolyCollection(verts)
    # poly.set_alpha(0.7)
    # ax.add_collection3d(poly, zs=zs)
    #


    # print Volume
    #
    # ax.set_xlabel('X (m)')
    # ax.set_ylabel('Y (m)')
    # ax.set_zlabel('Z (m)')
    # ax.set_xlim([np.amin(xr),np.amax(xr)])
    # ax.set_ylim([np.amin(yr),np.amax(yr)])
    # ax.set_zlim([np.amin(zr),np.amax(zr)])
    # plt.show()

# compute the convex hull of the points
# xr,yr,zr = unpack("v2.csv")
# sr=70
# X = get_hull_points(xr,yr,zr,sr)
# xh,yh,zh = unpack("human.csv")
# X = get_hull_points(xh,yh,zh,70)
# np.savetxt('human_concave_hull2.csv', X, delimiter=',')

#xp,yp,zp = unpack("v1.csv")
#Xp = get_hull_points(xp,yp,zp,40)
#np.savetxt('v1_concave_hull.csv', Xp, delimiter=',')

# xr,yr,zr = unpack("v2.csv")
# Xr = get_hull_points(xr,yr,zr,70)
# np.savetxt('v2_concave_hull.csv', Xr, delimiter=',')

#X = np.random.rand(100,3)
#X = np.column_stack((xh,yh,zh))

fname = 'human_concave_hull2.csv'
X = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)
fname = 'v1_concave_hull.csv'
Xp = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)
fname = 'v2_concave_hull.csv'
Xr = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)

cvx = ConvexHull(X)
cvx1 = ConvexHull(Xp)
cvx2 = ConvexHull(Xr)

x, y, z = X.T
x1, y1, z1 = Xp.T
x2, y2, z2 = Xr.T
#
# fname = 'human_concave_hull.csv'
# Xh = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)#

# # cvx.simplices contains an (nfacets, 3) array specifying the indices of
# # the vertices for each simplical facet
tri = mtri.Triangulation(x, y, triangles=cvx.simplices) #Human
tri1 = mtri.Triangulation(x1, y1, triangles=cvx1.simplices) #Model 1
tri2 = mtri.Triangulation(x2, y2, triangles=cvx2.simplices) #Model 2
# # c = np.array([1,2,3,4])
# # np.tile(c,(4,1))
#
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.hold(True)
ax.plot_trisurf(tri, z, linewidth=0.001, antialiased=True, color='r',alpha=0.4)
#ax.plot_trisurf(tri1, z1, linewidth=0.001, antialiased=True, color='b',alpha=0.2)
ax.plot_trisurf(tri2, z2, linewidth=0.001, antialiased=True, color='y',alpha=0.1)
# #ax.plot_trisurf(x,y, z, linewidth=0.2, antialiased=True, color='g',alpha=0.2)
# #ax.plot_wireframe(x, y, z, color='r')
# #ax.scatter(x, y, z, color='r')
#
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')

#ax.set_zticks([])
ax.view_init(azim=135, elev=45)
plt.show()