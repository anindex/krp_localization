from __future__ import print_function

from utils.data_processing import rosbag_inspect, rosbag_croptime, rosbag_concatenate
import rospkg

rospack = rospkg.RosPack()
file_path = rospack.get_path('krp_localization') + '/bags/test4.bag'
#rosbag_inspect(file_path)

output_bag1 = rospack.get_path('krp_localization') + '/bags/krp1-1.bag'
output_bag2 = rospack.get_path('krp_localization') + '/bags/krp1-2.bag'
output_bag3 = rospack.get_path('krp_localization') + '/bags/krp1-3.bag'

krp_bag1    = rospack.get_path('krp_localization') + '/bags/krp1.bag'
krp_bag2    = rospack.get_path('krp_localization') + '/bags/krp2.bag'

rosbag_croptime(file_path, output_bag1, tend=1566228135)
rosbag_croptime(file_path, output_bag2, tstart=1566228160, tend=1566228230)
rosbag_croptime(file_path, output_bag3, tstart=1566228250, tend=1566228310)
rosbag_inspect(output_bag1)
rosbag_inspect(output_bag2)
rosbag_inspect(output_bag3)


rosbag_concatenate(output_bag1, output_bag2, krp_bag1)
rosbag_concatenate(output_bag1, output_bag3, krp_bag2)
rosbag_inspect(krp_bag1)
rosbag_inspect(krp_bag2)
