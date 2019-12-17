import numpy as np
import pyqtgraph as pg
from bg_gurney import BasalGanglia

import time

CHANNELS = 6
T = 5
DT = 0.01
EXP_LEN = int(T / DT)

c = np.zeros(CHANNELS)
bg = BasalGanglia(CHANNELS)

plotWidget = pg.plot(title='BG output')
y = list()

for i in range(CHANNELS):
    y.append(np.zeros(EXP_LEN))

# z = {}
# for i in bg.pop.keys():
#     z[i] = list()
#     for j in range(CHANNELS):
#         z[i].append(np.zeros(EXP_LEN))

# Transient values from Humphries & Gurney, 2002
ch1_onset = 1
ch1_size = 0.4

ch2_onset = 2
ch2_size = 0.6

transient_onset = 3
transient_off = 4
transient_size = 0.2


for n in range(EXP_LEN):
    if (n * DT) == ch1_onset:
        c[0] = ch1_size

    if (n * DT) == ch2_onset:
        c[1] = ch2_size

    if (n * DT) == transient_onset:
        c[0] = c[0] + transient_size

    if (n * DT) == transient_off:
        c[0] = ch1_size

    bg.step(c)

    # for i in bg.pop.keys():
    #     for j in range(CHANNELS):
    #         z[i][j] = bg.pop[i]['o'][j]

    for i in range(CHANNELS):
        # y[i][n] = c[i]
        y[i][n] = bg.pop['GPi']['o'][i]

x = np.arange(EXP_LEN)
for i in range(CHANNELS):
    plotWidget.plot(x, y[i], pen=(i, CHANNELS))



# for i in z.keys():
#     for j in range(CHANNELS):
#         plotWidget.plot(x, z[i][j], pen=(i, 6))

time.sleep(3)