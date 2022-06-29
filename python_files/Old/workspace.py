
#from scipy.spatial import Delaunay
import numpy as np
import math
import pylab
import matplotlib as mpl
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


def calculation(xr,yr,zr):
    s = 70#**4 #Number of steps
    h = (zr[-1] - zr[0])/s

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
    ax.set_xlim([np.amin(xr),np.amax(xr)])
    ax.set_ylim([np.amin(yr),np.amax(yr)])
    ax.set_zlim([np.amin(zr),np.amax(zr)])
    plt.show()


def cutaway(xr,yr,zr):
    xp = []
    yp = []
    zp = []

    for i in range(0,len(xr)):
        if (xr[i]*yr[i]) <=0:
            xp.append(xr[i])
            yp.append(yr[i])
            zp.append(zr[i])

        if xr[i] <=0 and yr[i] <=0:
            xp.append(xr[i])
            yp.append(yr[i])
            zp.append(zr[i])

    return xp,yp,zp



def plot_cutaway(xr,yr,zr,xh,yh,zh,xp,yp,zp):
    fig2 = plt.figure()
    ax = fig2.add_subplot(111, projection='3d')

    xr1,yr1,zr1 = cutaway(xr, yr, zr)
    xh1,yh1,zh1 = cutaway(xh, yh, zh)
    xp1,yp1,zp1 = cutaway(xp, yp, zp)

    xmin = min(np.amin(xr),np.amin(xp),np.amin(xh))
    ymin = min(np.amin(yr),np.amin(yh),np.amin(yp))
    zmin = min(np.amin(zh),np.amin(zp),np.amin(zr))

    xmax = max(np.amax(xr),np.amax(xp),np.amax(xh))
    ymax = max(np.amax(yr),np.amax(yp),np.amax(yh))
    zmax = max(np.amax(yr),np.amax(yp),np.amax(yh))

    Nh = len(xh1)
    ch = np.array([1.0, 0.0, 0.0, 0.4])
    ch = np.tile(ch, (Nh,1))


    Nr = len(xr1)
    cr = []
    for i in np.arange(Nr):
        #alph = 0.05/(1+np.exp(-0.2*(np.absolute(xr1[i]) - 3*xmax/4)))
        dist = ((np.linalg.norm(np.array([xr1[i],yr1[i],zr1[i]])))/(np.linalg.norm(np.array([xmax,ymax,zmax]))))
        alph = 0.5 * dist**4
        cr.append([0,1.0,0,alph])


    Np = len(xp1)
    cp = []
    for i in np.arange(Nr):
        #alph = 0.05/(1+np.exp(-0.2*(np.absolute(xr1[i]) - 3*xmax/4)))
        dist = ((np.linalg.norm(np.array([xp1[i],yp1[i],zp1[i]])))/(np.linalg.norm(np.array([xmax,ymax,zmax]))))
        alph = 0.7 * dist**4
        cp.append([1.0,0.6,0,alph])


    ax.scatter(xh1, yh1, zh1, c=ch, marker='o', depthshade=False,edgecolors='none',label='Human')
    ax.scatter(xp1,yp1,zp1, c = cp , marker='o', depthshade=False,edgecolors='none',label='Human + Model I')
    ax.scatter(xr1,yr1,zr1, c=cr, marker='o', depthshade=False, edgecolors='none',label='Human + Model II')

    scatter1_proxy = mpl.lines.Line2D([0],[0], linestyle="none", c=[1,0,0], marker = 'o')
    scatter2_proxy = mpl.lines.Line2D([0],[0], linestyle="none", c=[1,0.6,0], marker = 'o')
    scatter3_proxy = mpl.lines.Line2D([0],[0], linestyle="none", c=[0,1,0], marker = 'o')
    ax.legend([scatter1_proxy, scatter2_proxy,scatter3_proxy], ['Human', 'Human + Model I', 'Human + Model II'], numpoints = 1)

    ax.set_xlim([xmin,xmax])
    ax.set_ylim([ymin,ymax])
    ax.set_zlim([zmin,zmax])

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.view_init(azim=0, elev=90)

    plt.show()


def full_plot(xr,yr,zr,xh,yh,zh,xp,yp,zp):
    fig2 = plt.figure()
    ax = fig2.add_subplot(111, projection='3d')

    xmin = min(np.amin(xr),np.amin(xp),np.amin(xh))
    ymin = min(np.amin(yr),np.amin(yh),np.amin(yp))
    zmin = min(np.amin(zh),np.amin(zp),np.amin(zr))

    xmax = max(np.amax(xr),np.amax(xp),np.amax(xh))
    ymax = max(np.amax(yr),np.amax(yp),np.amax(yh))
    zmax = max(np.amax(yr),np.amax(yp),np.amax(yh))

    Nh = len(xh)
    ch = np.array([1.0, 0.0, 0.0, 0.2])
    ch = np.tile(ch, (Nh,1))


    Nr = len(xr)
    cr = []
    dmax = (np.linalg.norm(np.array([np.amax(xr),np.amax(yr),np.amax(zr)])))
    for i in np.arange(Nr):
        #alph = 0.05/(1+np.exp(-0.2*(np.absolute(xr1[i]) - 3*xmax/4)))
        dist = ((np.linalg.norm(np.array([xr[i],yr[i],zr[i]])))/dmax)
        alph = 0.5 * dist**2
        cr.append([0,1.0,0,alph])


    Np = len(xp)
    cp = []
    dmax = np.linalg.norm((np.array([np.amax(xp),np.amax(yp),np.amax(zp)])))
    for i in np.arange(Nr):
        #alph = 0.05/(1+np.exp(-0.2*(np.absolute(xr1[i]) - 3*xmax/4)))
        dist = ((np.linalg.norm(np.array([xp[i],yp[i],zp[i]])))/dmax)
        alph = 0.9 * dist**2
        cp.append([1.0,0.6,0,0.1])


    ax.scatter(xh, yh, zh, c=ch, marker='o', depthshade=False,edgecolors='none',label='Human')
    ax.scatter(xp,yp,zp, c = cp , marker='o', depthshade=False,edgecolors='none',label='Human + Model I')
    ax.scatter(xr,yr,zr, c=cr, marker='o', depthshade=False, edgecolors='none',label='Human + Model II')

    scatter1_proxy = mpl.lines.Line2D([0],[0], linestyle="none", c=[1,0,0], marker = 'o')
    scatter2_proxy = mpl.lines.Line2D([0],[0], linestyle="none", c=[1,0.6,0], marker = 'o')
    scatter3_proxy = mpl.lines.Line2D([0],[0], linestyle="none", c=[0,1,0], marker = 'o')
    #ax.legend([scatter1_proxy, scatter2_proxy,scatter3_proxy], ['Human', 'Human + Model I', 'Human + Model II'], numpoints = 1)

    ax.set_xlim([xmin,xmax])
    ax.set_ylim([ymin,ymax])
    ax.set_zlim([zmin,zmax])

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    ax.set_zticks([])
    ax.view_init(azim=0, elev=90)
    plt.show()

xr,yr,zr = unpack("v2.csv") #Point cloud for Model II
xh,yh,zh = unpack("human.csv") #Point cloud for human
xp,yp,zp = unpack("v1.csv") #Point cloud for Model I

#calculation(xr,yr,zr)
calculation(xh,yh,zh)
#calculation(xp,yp,zp)

# Plotting the full point clouds
#full_plot(xr,yr,zr,xh,yh,zh,xp,yp,zp)

# Plotting the Cutaway
#plot_cutaway(xr,yr,zr,xh,yh,zh,xp,yp,zp)




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

#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
#ax.scatter(xr, yr, zr, c='green', alpha=0.5, marker='o', depthshade=False)
# ax.scatter(xh, yh, zh, c='red', alpha=0.5, marker='o', depthshade=False)
#ax.scatter(xp, yp, zp, c='blue', alpha=0.5, marker='o', depthshade=False)
#
# xmin = min(np.amin(xr),np.amin(xp),np.amin(xh))
# ymin = min(np.amin(yr),np.amin(yh),np.amin(yp))
# zmin = min(np.amin(zh),np.amin(zp),np.amin(zr))
#
# xmax = max(np.amax(xr),np.amax(xp),np.amax(xh))
# ymax = max(np.amax(yr),np.amax(yp),np.amax(yh))
# zmax = max(np.amax(yr),np.amax(yp),np.amax(yh))
#
# ax.set_xlim([xmin,xmax])
# ax.set_ylim([ymin,ymax])
# ax.set_zlim([zmin,zmax])
#
# ax.set_xlabel('X (m)')
# ax.set_ylabel('Y (m)')
# ax.set_zlabel('Z (m)')
#plt.show()





# fig3 = plt.figure()
# xx, yy, zz = np.meshgrid(xh1, yh1, zh1, sparse=True)
# ax = fig3.add_subplot(111, projection='3d')
# ax.plot_wireframe(xx,yy,zz)
# plt.show()