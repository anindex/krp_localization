from utils.load import load_data
from data_preprocessing import PreprocessedData

import rospkg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np

rospack = rospkg.RosPack()

map_path = rospack.get_path('krp_localization') + '/maps/test'
train_data_prefix = "test4"
train_data_path   = rospack.get_path('krp_localization') + '/data/'

print('Loading train data path: {}'.format(train_data_path))
raw_rssi, poses = load_data(file_name=train_data_prefix, file_path=train_data_path) #odometry is not needed

print('Number of RSSI dataset         : {}'.format(len(raw_rssi)))
print('Number of AMCL poses           : {}'.format(len(poses)))

traindata = PreprocessedData(raw_rssi, flag_negative_db         = True,
                                       flag_min_distance        = False,
                                       flag_fuse_measurements   = True,
                                       flag_min_points_per_AP   = False,
                                       flag_mode_filter         = False,
                                       flag_discard_non_pose    = True,
                                       poses                    = poses,
                                       filter_fuse_measurements = 1)

distance = np.sum(traindata.data['X'][1:]**2 + traindata.data['X'][:-1]**2 - 2*traindata.data['X'][1:]*traindata.data['X'][:-1],axis=1)**.5
avg_distance = np.mean(distance)

print('Number of processed measurements: {}'.format(traindata.data["Y"].shape[0]))
print('Number of macs considered (all) : {}'.format(len(traindata.all_mac_dict)))
print('Average distance between points : {}'.format(np.round(avg_distance,2)))


x0   = np.min(traindata.data['X'][:,0])
x1   = np.max(traindata.data['X'][:,0])
y0   = np.min(traindata.data['X'][:,1])
y1   = np.max(traindata.data['X'][:,1])

macs = (8, 11)
X = traindata.data['X'][:,0]
Y = traindata.data['X'][:,1]

fig = plt.figure(figsize=(8,8))

for mac in range(macs[0], macs[1] + 1):
    ax = fig.add_subplot(2, 2, mac - macs[0] + 1, projection='3d')
    ax.scatter(X, Y, traindata.data['Y'][:, mac], c='b', marker='o')

plt.show()
