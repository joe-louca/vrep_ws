from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib.animation import FuncAnimation
import numpy as np
import rospy

fig = plt.figure()
ax = fig.gca(projection='3d')

ft = rospy.get_param('ft')
x = ft[0]
y = ft[1]
z = ft[2]

quivers = ax.quiver(0,0,0,x,y,z)

ax.set_xlim([-200,200])
ax.set_ylim([-200,200])
ax.set_zlim([-200,200])

def animate(i):
    ft = rospy.get_param('ft')
    x = ft[0]
    y = ft[1]
    z = ft[2]

    new_segs = [[0,0,0],[x,y,z]]
    
    quivers.set_segments(new_segs)
    return quivers


ani = FuncAnimation(fig, animate, interval = 30, blit=False)
#ani.save('update_3d_quiver.gif', writer='imagemagick')
fig.tight_layout()
plt.show()
