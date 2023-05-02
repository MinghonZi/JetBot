import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
from smbus2 import SMBus
from matplotlib.animation import FuncAnimation
from matplotlib.axes import Axes
from vl53l5cx_ctypes import VL53L5CX, RESOLUTION_8X8

vl53l5cx = VL53L5CX(i2c_dev=SMBus(0))

def init():
    vl53l5cx.set_resolution(RESOLUTION_8X8)
    vl53l5cx.start_ranging()

def vis(frame):
    if not vl53l5cx.data_ready():
        return
    data = vl53l5cx.get_data()
    distance_mm = np.array(data.distance_mm).reshape(8, 8)
    plt.clf()
    ax: Axes = sns.heatmap(distance_mm,
        vmin=0,
        cmap = "coolwarm",
        square=True,
        linewidth=.5,
        annot=True,
        annot_kws={"fontsize": 6},
    )

fig = plt.figure()
ani = FuncAnimation(fig=fig,
    func=vis,
    init_func=init,
    interval=67,  # 8x8 resolution at 15 Hz ~= 67 ms
    cache_frame_data=False,
)

plt.show()
