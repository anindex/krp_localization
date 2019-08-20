from utils.load import load

import rospkg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np

rospack = rospkg.RosPack()

evaluation_data_prefix = "max_gaussian_mean"
evaluation_data_path   = rospack.get_path('krp_localization') + '/data/evaluations/'

eval_train = load(evaluation_data_path + evaluation_data_prefix + '_train.p')
eval_test = load(evaluation_data_path + evaluation_data_prefix + '_test.p')
criteria = eval_train.pop(0)
eval_test.pop(0)

eval_train = np.array(eval_train)
eval_test = np.array(eval_test)

mean_train,  std_train = np.mean(eval_train, axis=0), np.std(eval_train, axis=0)
mean_test,  std_test = np.mean(eval_test, axis=0), np.std(eval_test, axis=0)

x = np.arange(len(criteria))  # the label locations
width = 0.35  # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(x - width / 2, mean_train, width, label='Train Data', color='blue', ecolor='black')
rects2 = ax.bar(x + width / 2, mean_test, width, label='Test Data', color='red', ecolor='black')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Particle Percentage')
ax.set_xlabel('Distance to robot pose')
ax.set_title('Particle Percentage near robot pose')
ax.set_xticks(x)
ax.set_xticklabels(criteria)
ax.legend()

fig.tight_layout()

plt.show()
