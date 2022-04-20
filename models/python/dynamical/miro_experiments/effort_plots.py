import numpy as np
import matplotlib.pyplot as plt

with open('data_miro_s0.npy', 'rb') as f:
    E1_0 = np.load(f)
    E2_0 = np.load(f)


with open('data_miro_s1.npy', 'rb') as f:
    E1_1 = np.load(f)
    E2_1 = np.load(f)



plt.scatter( E1_0, E2_0 )
plt.scatter( E1_1, E2_1 )

plt.show()