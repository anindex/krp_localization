from __future__ import print_function

from data_preprocessing import PreprocessedData, Measurement
from utils.structures import mesh, pose_from_array
from utils.load import load_data
from localization.models import hGP
from krp_localization.msg import RSSIData, RssData

import rospkg


rospack = rospkg.RosPack()
train_data_path = rospack.get_path('krp_localization') + "/data/"
raw_rssi, poses, odom = load_data(file_name="test", file_path=train_data_path)

#print("Num rssi: ", len(raw_rssi))
#print("Num poses: ", len(poses))
#print("Num odom: ", len(odom))
#print("Sample rssi: ", raw_rssi[0])
#print("Sample pose: ", poses[0])
#print("Sample odom: ", odom[0])

#print(raw_rssi[0])
#m = Measurement(raw_rssi[0], flag_negative_db=True)
#print(m)

print("Num rssi before fuse: ", len(raw_rssi))
data = PreprocessedData(raw_rssi, poses=poses, flag_negative_db=True, flag_min_distance=True, flag_fuse_min_points_per_AP=True)
print("Num rssi after fuse: ", data.nm)
print(data.data['Y'][0])
print(data.data['Var'][0])
print(data.data['n'][0])
