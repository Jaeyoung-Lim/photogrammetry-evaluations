import matplotlib.pyplot as plt
import matplotlib as mpl

fig, ax = plt.subplots(figsize=(1, 6))
fig.subplots_adjust(right=0.5)

cmap = mpl.cm.gist_rainbow
norm = mpl.colors.Normalize(vmin=0, vmax=1.0)

fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),
             cax=ax, orientation='vertical', label='Uncertainty [m]')

plt.show()