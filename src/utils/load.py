import cPickle as pickle
import rospkg


def save(var,filepath='var.p'):
    with open(filepath, 'wb') as f:
        pickle.dump(var,f,2)
    return True

def load(filepath='last_model.p'):
    with open(filepath, 'rb') as f:
        var = pickle.load(f)
    return var

def load_data(**kwargs):
    """
    Load a previously pickled data from file_path/file_name.
    Optional Parameters [default]
        file_name ['b3test1']: Core name of the files to get data from -> <file_name>rss.p and <file_name>poses.p
        file_path ['~/catkin_ws/src/tests/bags/processed_data/']: Path where to find files
    """
    rospack = rospkg.RosPack()
    file_name = kwargs.get('file_name','floor2')
    file_path = kwargs.get('file_path',None)
    if file_path is None:
        file_path=rospack.get_path('krp_localization') + '/data/'

    f=file_path+file_name

    file1 = f+'rssi.p'
    file2 = f+'poses.p'
    file3 = f+'odom.p'

    with open(file1, 'rb') as f:
      rssi = pickle.load(f)
    with open(file2, 'rb') as f:
      poses = pickle.load(f)
    with open(file3, 'rb') as f:
      odom = pickle.load(f)

    return (rssi,poses,odom)
