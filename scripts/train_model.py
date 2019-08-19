from localization.models import hGP
from utils.load import load_data
from data_preprocessing import PreprocessedData
from localization.sampling import accept_reject_by_regions, accept_reject_by_regions_map, accept_reject_uniform

import rospkg
import numpy as np

rospack = rospkg.RosPack()

train_data_prefix = "test4"
train_data_path   = rospack.get_path('krp_localization') + '/data/'
save_path         = rospack.get_path('krp_localization') + '/models/model_test4.p'

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

#Train model
print('Training model')
model = hGP(traindata.data,all_mac_dict=traindata.all_mac_dict, sampling=accept_reject_uniform, Xtest_num=75)
model.optimize()
print('Model trained successfully')

model.save(save_path)
