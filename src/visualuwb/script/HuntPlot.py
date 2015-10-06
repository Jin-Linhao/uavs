#from HuntController import *
from matplotlib.pyplot import cm
import matplotlib.pyplot as plt
import numpy as np

# Now we set up the data to plot. First, `mgrid()` will create a dense
# mesh-grid with the coordinates we are going to use to compute the vector
# field. The `X` and `Y` arrays are the base points for each arrow, the `U`,
# `V` are the end points. The auxiliary arrays `UN` and `VN` contain the
# normalized values of `U` and `V` to draw all arrows with length 1.
# The length of the arrows is stored in the `speed` array.
Y, X = np.mgrid[-3:3:15j, -3:3:15j]
U = -1 - np.cos(X**2 + Y)
V = 1 + X - Y
speed = np.sqrt(U**2 + V**2)
UN = U/speed
VN = V/speed

# ---

# ### Quiver Plot, dynamic colours

# Now we use the `quiver()` function to create the arrow plot (also know and
# quiver plot or velocity plot). The arrays `X` and `Y` contain the coordinates
# of the start points for each arrow, the array `UN` and `VN` contain the
# coordinates of the corresponding end points. The array `U` is used to colour
# the plot.
plot1 = plt.figure()
plt.quiver(X, Y, UN, VN,        # data
           U,                   # colour the arrows based on this array
           cmap=cm.seismic,     # colour map
           headlength=7)        # length of the arrows

plt.colorbar()                  # adds the colour bar

plt.title('Quive Plot, Dynamic Colours')
plt.show(plot1)                 # display the plot

# ---

# ### Quiver Plot, single colour

# To use a single colour we omit the fifth argument in the previous example and
# use the `colour` parameter instead.
plot2 = plt.figure()
plt.quiver(X, Y, UN, VN,
           color='Teal',
           headlength=7)

plt.title('Quiver Plot, Single Colour')
plt.show(plot2)

# ---

# ### Stream Plot, dynamic colour

# The next example plots the stream lines of the vector field by means of the
# `streamplot()` function. A fixed line width is used.

plot3 = plt.figure()
plt.streamplot(X, Y, U, V,          # data
               color=speed,         # array that determines the colour
               cmap=cm.cool,        # colour map
               linewidth=2,         # line thickness
               arrowstyle='->',     # arrow style
               arrowsize=1.5)       # arrow size

plt.colorbar()                      # add colour bar on the right

plt.title('Stream Plot, Dynamic Colour')
plt.show(plot3)                     # display the plot


# ---

# ### Stream Plot, dynamic line width

# This example also uses the `streamplot()` functions. It shows how to set a
# fixed colour, dynamic line width and customized lines density. The default
# density uses a 25x25 mesh in the computations, the values assigned to the
# `density` parameter are proportional to this default value. In this case half
# the density is used in the x-axis but the y-axis is kept unchanged.
plot4 = plt.figure()
lw = 5*speed/speed.max()            # line width proportional to speed

plt.streamplot(X, Y, U, V,          # data
               density=[0.5, 1],
               color='DarkRed', 
               linewidth=lw)

plt.title('Stream Plot, Dynamic Line Width')
plt.show(plot4)
