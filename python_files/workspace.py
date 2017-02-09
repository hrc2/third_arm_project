
from scipy.spatial import Delaunay
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.tri as mtri


def sq_norm(v): #squared norm
    return np.linalg.norm(v)**2


def circumcircle(points,simplex):
    A=[points[simplex[k]] for k in range(3)]
    M=[[1.0]*4]
    M+=[[sq_norm(A[k]), A[k][0], A[k][1], 1.0 ] for k in range(3)]
    M=np.asarray(M, dtype=np.float32)
    S=np.array([0.5*np.linalg.det(M[1:,[0,2,3]]), -0.5*np.linalg.det(M[1:,[0,1,3]])])
    a=np.linalg.det(M[1:, 1:])
    b=np.linalg.det(M[1:, [0,1,2]])
    return S/a,  np.sqrt(b/a+sq_norm(S)/a**2) #center=S/a, radius=np.sqrt(b/a+sq_norm(S)/a**2)


def get_alpha_complex(alpha, points, simplexes):
    #alpha is the parameter for the alpha shape
    #points are given data points
    #simplexes is the  list of indices in the array of points
    #that define 2-simplexes in the Delaunay triangulation

    return filter(lambda simplex: circumcircle(points,simplex)[1]<alpha, simplexes)


def unpack(fname):
    pts = np.loadtxt(open(fname, "rb"), delimiter=",", skiprows=1)
    xs = pts[:,0]
    ys = pts[:,1]
    zs = pts[:,2]
    return xs,ys,zs


#tri = Delaunay(pts)

xr,yr,zr = unpack("v2.csv")
xh,yh,zh = unpack("human.csv")

Nr = xr.size

Nh = xh.size

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(xr, yr, zr)
plt.show()