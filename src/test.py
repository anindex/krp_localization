from __future__ import print_function

from data_preprocessing import PreprocessedData, Measurement
from utils.structures import mesh, pose_from_array
from utils.load import load_data, save
from localization.models import hGP
from krp_localization.msg import RSSIData

import rospkg


rospack = rospkg.RosPack()
train_data_path = rospack.get_path('krp_localization') + "/data/"
raw_rssi, poses = load_data(file_name="test", file_path=train_data_path)

#print("Num rssi: ", len(raw_rssi))
#print("Num poses: ", len(poses))
#print("Num odom: ", len(odom))
#print("Sample rssi: ", raw_rssi[0])
#print("Sample pose: ", poses[0])
#print("Sample odom: ", odom[0])

#print(raw_rssi[0])
#m = Measurement(raw_rssi[0], flag_negative_db=True)
#print(m)

#print("Num rssi before fuse: ", len(raw_rssi))
#data = PreprocessedData(raw_rssi, poses=poses, flag_negative_db=True, flag_min_distance=True, flag_fuse_min_points_per_AP=True)
#print("Num rssi after fuse: ", data.nm)
#print(data.data['Y'][0])
#print(data.data['Var'][0])
#print(data.data['n'][0])

start, end = poses[0].header.stamp.secs, poses[-1].header.stamp.secs
print("Duration: {}s".format(end - start))
print("Len RSSI: {}".format(len(raw_rssi)))

inc = (end - start) / float(len(raw_rssi))
print(inc)
for i, m in enumerate(raw_rssi):
    m.start_time_ns += (start + i * inc) * 1e9

save(raw_rssi, "/home/anindex/robotics_ws/src/krp_localization/data/testrssi_fixed.p")
