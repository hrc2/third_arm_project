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

def get_hull_points(xr,yr,zr,s):
    #s = 70 #Number of steps
    h = (zr[-1] - zr[0])/s

    #Area = 0.0

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
        # Area += PolyArea(hull[:,0], hull[:,1])
        # if i==1 or i==s:
        #     Area *= 0.5

    return np.vstack(verts)



def surf_point(u,v,delt):
    r1 = 0.31
    r2 = 0.375
    l = r2+delt
    x = (r1+(l)*np.cos(v))*np.cos(u)+ 0.306
    y = (r1+(l)*np.cos(v))*np.sin(u) -0.052
    z = (l)*np.sin(v) + 0.41
    return x,y,z

def make_curved():
    N =100
    #u = (np.linspace(math.pi/2,2*math.pi)*np.ones((10, 1))).flatten()
    #v = (np.linspace(-math.pi,(30/180)*math.pi)*np.ones((10, 1))).flatten()
    u, v = np.mgrid[-math.pi:math.pi/2:100j, -math.pi/2:math.pi/6:100j]
    delt = np.linspace(0,0.15)

    xi,yi,zi = surf_point(u,v,delt[0])
    xo,yo,zo = surf_point(u,v,delt[-1])

    return xi,yi,zi,xo,yo,zo


def make_straight():
    #N =100
    #u = (np.linspace(math.pi/2,2*math.pi)*np.ones((10, 1))).flatten()
    #v = (np.linspace(-math.pi,(30/180)*math.pi)*np.ones((10, 1))).flatten()

    #Edge 1:
    v = math.pi/6
    u, delt = np.mgrid[-math.pi:math.pi/2:100j, 0:0.15:100j]
    x1,y1,z1 = surf_point(u,v,delt)

    #Edge 2:
    u = -math.pi
    v, delt = np.mgrid[-math.pi/2:math.pi/6:100j, 0:0.15:100j]
    x2,y2,z2 = surf_point(u,v,delt)

    #Edge 3:
    u = math.pi/2
    v, delt = np.mgrid[-math.pi/2:math.pi/6:100j, 0:0.15:100j]
    x3,y3,z3 = surf_point(u,v,delt)

    #Edge 4:
    v = -math.pi/2
    u, delt = np.mgrid[-math.pi:math.pi/2:100j, 0:0.15:100j]
    x4,y4,z4 = surf_point(u,v,delt)


    return x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4



# xr,yr,zr = unpack("v2.csv")
# sr=70
# X = get_hull_points(xr,yr,zr,sr)

# fname = 'human_concave_hull.csv'
# X = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)
fname = 'v1_fixed_human.csv'
Xp = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)
fname = 'v2_fixed_human.csv'
Xr = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)

#cvx = ConvexHull(X)
#cvx1 = ConvexHull(Xp)
cvx2 = ConvexHull(Xr)

#x, y, z = X.T
x1, y1, z1 = Xp.T
x2, y2, z2 = Xr.T
#
# fname = 'human_concave_hull.csv'
# Xh = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)#

# # cvx.simplices contains an (nfacets, 3) array specifying the indices of
# # the vertices for each simplical facet
#tri = mtri.Triangulation(x, y, triangles=cvx.simplices)
#tri1 = mtri.Triangulation(x1, y1, triangles=cvx1.simplices)
tri2 = mtri.Triangulation(x2, y2, triangles=cvx2.simplices)
# # c = np.array([1,2,3,4])
# # np.tile(c,(4,1))

Np = len(x2)
cp = []
#dmax = np.linalg.norm((np.array([np.amax(xp),np.amax(yp),np.amax(zp)])))
for i in np.arange(Np):
    #alph = 0.05/(1+np.exp(-0.2*(np.absolute(xr1[i]) - 3*xmax/4)))
    #dist = ((np.linalg.norm(np.array([xp[i],yp[i],zp[i]])))/dmax)
    #alph = 0.9 * dist**2
    cp.append([1.0,0.6,0,0.05])

Nr = len(x1)
cr = []
#dmax = np.linalg.norm((np.array([np.amax(xp),np.amax(yp),np.amax(zp)])))
for i in np.arange(Nr):
    #alph = 0.05/(1+np.exp(-0.2*(np.absolute(xr1[i]) - 3*xmax/4)))
    #dist = ((np.linalg.norm(np.array([xp[i],yp[i],zp[i]])))/dmax)
    #alph = 0.9 * dist**2
    cr.append([1.0,0.0,0,0.7])


xi,yi,zi,xo,yo,zo = make_curved()
ex1,ey1,ez1,ex2,ey2,ez2,ex3,ey3,ez3,ex4,ey4,ez4 = make_straight()
alphval = 0.2

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.hold(True)
#ax.plot_trisurf(tri, z, linewidth=0.001, antialiased=True, color='r',alpha=0.4)
#ax.plot_trisurf(hull[:,0], hull[:,1],z2,linewidth=0.001, antialiased=True, color='b',alpha=0.2)
#ax.plot_trisurf(tri2, z2, linewidth=0.001, antialiased=True, color='y',alpha=0.1)
#ax.scatter(x2,y2,z2, c=cp, marker='o', depthshade=False, edgecolors='none',label='Human + Model II')
ax.scatter(x1,y1,z1, c=cr, marker='o', depthshade=False, edgecolors='none',label='Human + Model I')
#ax.scatter(0.306,-0.052,0.41, c='b', marker='x', depthshade=False)
ax.plot_surface(xi, yi, zi,  color='y', alpha=alphval, linewidth=0)
ax.plot_surface(xo, yo, zo,  color='y', alpha=alphval, linewidth=0)
ax.plot_surface(ex1, ey1, ez1,  color='y', alpha=alphval, linewidth=0)
ax.plot_surface(ex2, ey2, ez2,  color='y', alpha=alphval, linewidth=0)
ax.plot_surface(ex3, ey3, ez3,  color='y', alpha=alphval, linewidth=0)
ax.plot_surface(ex4, ey4, ez4,  color='y', alpha=alphval, linewidth=0)
#ax.plot_trisurf(xo, yo, zo, triangles=trio.triangles, c='g')
#ax.plot_trisurf(xi, yi, zi, triangles=trii.triangles, c='g')
# #ax.plot_trisurf(x,y, z, linewidth=0.2, antialiased=True, color='g',alpha=0.2)
#ax.plot_wireframe(x2, y2, z2, color='g', linewidth=0.01)
# #ax.scatter(x, y, z, color='r')
#
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')

ax.set_zticks([])
ax.view_init(azim=0, elev=90)
plt.show()