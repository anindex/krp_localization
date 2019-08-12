from __future__ import print_function

from data_preprocessing import PreprocessedData, Measurement
from utils.structures import mesh, pose_from_array
from utils.load import load_data, save
from utils.data_processing import rosbag_inspect, rosbag_croptime, rosbag_concatenate
from localization.models import hGP
from krp_localization.msg import RSSIData

import rospkg

rospack = rospkg.RosPack()
file_path = rospack.get_path('krp_localization') + '/bags/test.bag'
#rosbag_inspect(file_path)

output_bag1 = rospack.get_path('krp_localization') + '/bags/krp1-1.bag'
output_bag2 = rospack.get_path('krp_localization') + '/bags/krp1-2.bag'
output_bag3 = rospack.get_path('krp_localization') + '/bags/krp1-3.bag'

krp_bag1    = rospack.get_path('krp_localization') + '/bags/krp1.bag'
krp_bag2    = rospack.get_path('krp_localization') + '/bags/krp2.bag'

#rosbag_croptime(file_path, output_bag1, tend=1565603530.0)
#rosbag_croptime(file_path, output_bag2, tstart=1565603580, tend=1565603650)
#rosbag_croptime(file_path, output_bag3, tstart=1565603680, tend=1565603720)
#rosbag_inspect(output_bag1)
#rosbag_inspect(output_bag2)
#rosbag_inspect(output_bag3)


rosbag_concatenate(output_bag1, output_bag2, krp_bag1)
rosbag_concatenate(output_bag1, output_bag3, krp_bag2)
rosbag_inspect(krp_bag1)
rosbag_inspect(krp_bag2)
