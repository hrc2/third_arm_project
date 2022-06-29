import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.tri as mtri

def cylinder(r,n,L):
    '''
    Returns the unit cylinder that corresponds to the curve r.
    INPUTS:  r - a vector of radii
             n - number of coordinates to return for each element in r

    OUTPUTS: x,y,z - coordinates of points
    '''

    # ensure that r is a column vector
    r = np.atleast_2d(r)
    r_rows,r_cols = r.shape

    if r_cols > r_rows:
        r = r.T

    # find points along x and y axes
    points  = np.linspace(0,2*np.pi,n+1)
    x = np.cos(points)*r
    y = np.sin(points)*r

    # find points along z axis
    rpoints = np.atleast_2d(np.linspace(0,1,len(r)))
    z = L*np.ones((1,n+1))*rpoints.T

    return x,y,z



# set curve vector r and set n to 20
r = np.linspace(2,2,100)
n = 100
l = 5

# get points from cylinder and plot
[x,y,z] = cylinder(r,n,l)

# make plots
fig=plt.figure()
ax = p3.Axes3D(fig)
#tri = mtri.Triangulation(x,y)
ax.plot_surface(x, y, z)
#ax.plot_wireframe(x,y,z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()