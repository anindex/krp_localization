import numpy as np
import copy
from subprocess import check_output, call
import rosbag, rospy
import tf
from geometry_msgs.msg import Transform, Quaternion
import yaml




def validate_data(din):
    """
    Script to verify if din is a dictionary compatible with Localization classes and functions.
    If not compatible, an error is raised. Else, a formated (if necessary) deep copy is returned

    data:
        structure used for Localization classes and functions
        mandatory keys => 'X': numpy array [p,2]
                          'Y': numpy array [p,n] or [p,] - [p,n] should be formatted to [p,1]
                          'Var': numpy array [p,n] or [p,] - [p,n] should be formatted to [p,1]

    out:
        formated deep copy of data
    """

    if not all(k in din for k in ('X','Y','Var')):
        raise IOError       # If all keys are not present in dict, raise error

    assert din['X'].shape[1] == 2
    assert din['Y'].shape[0] == din['Y'].shape[0]
    assert din['Var'].shape[0] == din['Var'].shape[0]

    data = copy.deepcopy(din)

    if data['Y'].ndim == 1:
        data['Y'] = np.reshape(data['Y'],(data['Y'].shape[0],1))       #casting into [p,1]

    if data['Var'].ndim == 1:
        data['Var'] = np.reshape(data['Var'],(data['Var'].shape[0],1)) #casting into [p,1]

    return data


def rosbag_inspect(input_bag):
    bag_path = input_bag
    info_dict = yaml.load(rosbag.bag.Bag(bag_path, 'r')._get_yaml_info())
    print(bag_path)
    print('Start    \t: '+str(info_dict['start']))
    print('End      \t: '+str(info_dict['end']))
    duration = info_dict['end']-info_dict['start']
    print('Messages \t: '+str(info_dict['messages']))
    topics = info_dict['topics']
    #Uncomment the following if you want to see information about the topics
    print('(#messages) \ttopic')
    for topic in topics:
        print('  ('+str(topic['messages'])+')    \t'+topic['topic'])

def rosbag_croptime(input_bag, output_bag, tstart=0, tend=2e9):
    filter_cmd = "({}<=t.secs)and(t.secs<{})".format(tstart,tend)
    call(['rosbag','filter',input_bag,output_bag,filter_cmd])

def rosbag_concatenate(input_bag1,input_bag2,output_bag):
    #Find end of the first bag and beginning of the second
    #First bag must end before the beginning of the second
    info_dict = yaml.load(rosbag.bag.Bag(input_bag1, 'r')._get_yaml_info())
    bag1_tend = info_dict['end']
    info_dict = yaml.load(rosbag.bag.Bag(input_bag2, 'r')._get_yaml_info())
    bag2_tstart = info_dict['start']
    offset_time = bag2_tstart-bag1_tend+0.01
    offset_time = rospy.rostime.Time.from_sec(offset_time)

    pr = 1
    seq = 0
    with rosbag.bag.Bag(output_bag, 'w') as outbag:
        #The first input bag is directly copied into output bag
        for topic, msg, t in rosbag.bag.Bag(input_bag1):
            outbag.write(topic, msg, t)
            if topic=='rosout':
                seq = msg.header.seq

            if topic=='tf':
                if t.to_sec()>bag1_tend-1.:
                    tr1 = msg.transforms[0].transform
            outbag.write(topic, msg, t)

        #The second input bag's time is modified as well as its odometry TF
        for topic, msg, t in rosbag.bag.Bag(input_bag2).read_messages(topics=['tf', 'tf_static', 'rssi', 'amcl_pose', 'base_scan', 'odom']):
            #new time
            tnew = rospy.rostime.Time.from_sec((t-offset_time).to_sec())
            if topic=='tf':
                # considerations: only odometry is corrected and
                #                 odometry is the only transform,
                if pr:
                    # first tf
                    tr2 = msg.transforms[0].transform
                    pr = 0
                trn = msg.transforms[0].transform
                msg.transforms[0].transform = transform_fix(tr1,tr2,trn)

                for transform in msg.transforms:
                    transform.header.stamp = tnew
            else:
                try:
                    msg.header.stamp = tnew
                except:
                    pass

            if topic=='rosout':
                seq += 1
                msg.header.seq = seq
            if topic=='rssi':
                msg.start_time_ns = msg.start_time_ns-long(offset_time.to_nsec())

            outbag.write(topic, msg, tnew)


def transform_fix(t1,t2,tn):
    """
    t1,t2,tn
    """
    #translation
    dx = tn.translation.x - t2.translation.x
    dy = tn.translation.y - t2.translation.y

    #rotation
    euler1 = tf.transformations.euler_from_quaternion([t1.rotation.x, t1.rotation.y, t1.rotation.z, t1.rotation.w])
    euler2 = tf.transformations.euler_from_quaternion([t2.rotation.x, t2.rotation.y, t2.rotation.z, t2.rotation.w])
    eulern = tf.transformations.euler_from_quaternion([tn.rotation.x, tn.rotation.y, tn.rotation.z, tn.rotation.w])

    dyaw = euler2[2]-euler1[2]

    ndx = dx*np.cos(dyaw)+dy*np.sin(dyaw)
    ndy = -dx*np.sin(dyaw)+dy*np.cos(dyaw)

    tout = Transform()
    tout.translation.x = ndx+t1.translation.x
    tout.translation.y = ndy+t1.translation.y

    qout = tf.transformations.quaternion_from_euler(0,0,eulern[2]-euler2[2]+euler1[2])
    tout.rotation = Quaternion(*qout)

    return tout
