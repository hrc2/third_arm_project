import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

fig1 = plt.figure()
ax1 = fig1.add_subplot(111, aspect='equal')
center = (0, 0)
r = 0.80
theta1 = 30
theta2 = 150
width = 0.160

wedge = patches.Wedge(center, r, theta1, theta2, width, hatch='.' , linestyle='dashdot', facecolor='#ff9999')
dot = patches.Circle((0,0), radius=0.01,fill=False)

ax1.add_patch(wedge)
ax1.add_patch(dot)
# fig1 = plt.figure()
# ax1 = fig1.add_subplot(111, aspect='equal')
# ax1.add_patch(
#     patches.Rectangle(
#         (0.1, 0.1),   # (x,y)
#         0.5,          # width
#         0.5,          # height
#     )
# )

ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')

plt.show()
