from __future__ import print_function
import sys
#standard libraries
import numpy as np
import pickle
import copy
#Data types
from krp_localization.msg import RSSIData




class Measurement():
    def __init__(self, mdata, **kwargs):
        """
        Class around RSSIData message. Provides functions for analysing and filtering the messages.

        Input
            mdata: ROS RSSIData message

        Optional Parameters [default]
            filter_rss_factor [0.3]: Factor used to filter rss
            prior_var [5]  : Prior variances assigned when no measurement is found

        Functions
            filter_rssi
            pose_distance
            rssi_mean
            rssi_var
            rssi_len
            transform2vector
        """
        assert type(mdata) is RSSIData

        #filter params
        self.flag_mode_filter = kwargs.get('flag_mode_filter', True)
        self.filter_factor    = kwargs.get('filter_rssi_factor', 0.3) #30% lower than mode count

        #measurement param
        self.min_rssi  = kwargs.get('min_rssi', 0)
        self.prior_var = kwargs.get('prior_var', 5)

        #retrieve mdata
        self.time = long(mdata.start_time_ns)
        self.nap = len(mdata.macs)
        self.mac_dict = {mdata.macs[i]:i for i in np.arange(self.nap)}
        self.rssi  = [s.rssi for s in mdata.rssi]

        #Filter
        self.filtered_rssi = list()
        if self.flag_mode_filter:
            self.filter_rssi()

        self.pose = None

    def filter_rssi(self,**kwargs):
        """
        Filters rss data by finding the mode and eliminating rss values which have less than filter_rss_factor*mode of repetitions

        Parameters [default]
            filter_rss_factor [class filter_factor]
            min_repetition [class min_repetition_flag]
            min_repetition_rss [class min_repetition_rss]
        """

        filter_factor       = kwargs.get('filter_rss_factor', self.filter_factor) #30% lower than mode

        for rss in self.rssi:

            unique_rss = list(set(rss)) #list of unique rss values
            count_rss = [rss.count(u_rss) for u_rss in unique_rss] # number of times each unique rss appears
            # filter those that appearh less than factor times the mode
            mode_rss = np.max(count_rss)
            filtered_rss = [r for r,c in zip(unique_rss,count_rss) for i in range(c) if c > filter_factor*mode_rss]
            self.filtered_rssi.append(filtered_rss)

    def pose_distance(self,other):
        """
        Checks the distance (linear) between its pose and another measurement's pose
        """
        assert type(other) is type(self)
        if (self.pose is None) or (self.pose is None):
            return np.inf

        d_lin = ((self.pose[0]-other.pose[0])**2 + (self.pose[1]-other.pose[1])**2)**.5
        return d_lin

    def rssi_mean(self):
        """
        Outputs mean of rss measurements
        """
        if self.flag_mode_filter:
            rssi = self.filtered_rssi
        else:
            rssi = self.rssi

        return np.asarray([np.mean(s) for s in rssi])

    def rssi_var(self):
        """
        Outputs variance of rss measurements
        """
        if self.flag_mode_filter:
            rssi = self.filtered_rssi
        else:
            rssi = self.rssi

        #output rss variance if len(rss) > 1 if not output the default variance
        #var = np.asarray([np.var(s) if (len(s)>1) else self.prior_var for s in selected_rss])
        #if each sample is considered to be taken from a noisy function with gaussian noise of the same variance,
        #then the variance of a sequence of samples becomes prior_noise_variace/n_samples
        #if rss_len is zero, the prior variance is assigned hence, in practice the min_rss_len = 1
        var = self.prior_var/np.clip(self.rss_len(),1,np.inf)

        return var #np.clip(var,2e-5,np.inf)

    def rssi_len(self):
        """
        Outputs variance of rss measurements
        index='all' computes the mean value for all measurements
        """
        if self.flag_mode_filter:
            rssi = self.filtered_rssi
        else:
            rssi = self.rssi

        #output len(rss) if it is higher than 1, if 1 output 0 -> in the filtering process, instead of eliminating entries
        #a single min_rss value is put, so len=1 should be len=0
        return np.asarray([len(s) for s in rssi])

    def transform2vector(self,ref_mac_dict,fun):
        """
        Outputs a vector for training and prediction, using a mac_dict of all sensed vectors in the area
        """
        if fun=='mean':
            rssi_fun = self.rssi_mean()
            default_val = self.min_rssi
        elif fun=='var':
            rssi_fun = self.rssi_var()
            default_val = self.prior_var
        elif fun=='len':
            rssi_fun = self.rssi_len()
            default_val = 0
        else:
            print('Incorrect key: ', str(fun))

        vector = default_val*np.ones(len(ref_mac_dict))
        for mac,index in self.mac_dict.iteritems(): #for all macs in self
            if mac in ref_mac_dict:
                vector[ref_mac_dict[mac]] = rssi_fun[index] #put rss value in the adequate index from ref_mac_dict
        return vector

    #overloading addition
    def __add__(self,other):
        """
        Overloading __add__ to allow the addition of measurements
        When two measurements are added, all their signals are convined.
        The time is set to the average of both measurements.
        If the data comes from different access points, the new* access points are added to the dictionary
        If the data comes from already sensed access points, the new measurements are appended to current data
        """
        assert type(other) is type(self)
        result = copy.deepcopy(self)

        result.time = long(result.time*.5)+long(other.time*.5)

        #finding mac_addresses not sensed in self
        to_add_keys = [mac for mac in other.mac_dict if mac not in result.mac_dict]

        #updating
        result.mac_dict.update({to_add_keys[i]:(i+result.nap) for i in range(len(to_add_keys))})
        result.nap = len(result.mac_dict)

        result.rssi = result.rssi + [[] for i in range(len(to_add_keys))]
        for other_mac,other_index in other.mac_dict.iteritems():
            index = result.mac_dict[other_mac]
            result.rssi[index] = result.rssi[index]+other.rssi[other_index]

        # if position is known
        if result.pose is None:
            if other.pose is None:
                pass #No measure knows its position
            else:
                result.pose = other.pose #self.pose is not known but other is known
        else:
            if other.pose is None:
                pass #pose is knonw and other.pose is not, so pose remains the same
            else:
                result.pose[0] = .5*result.pose[0]+.5*other.pose[0]
                result.pose[1] = .5*result.pose[1]+.5*other.pose[1]
        return result

    def __str__(self):
        """
        Overloading __str__ to make print statement meaningful

        <format>
        Mac RSS
        """
        to_print = 'Time: {} ns\n'.format(self.time)
        if self.pose is not None:
            to_print+='Pose: {}\n'.format(str(self.pose))
        to_print += '{} {}\n'.format('mac address'.ljust(17), 'RSS')
        for mac,index in self.mac_dict.iteritems():
            to_print += '{} {}\n'.format(mac.ljust(17),self.rssi[index])

        return to_print

############################################## PROCESSED DATA CLASS ##############################################
class PreprocessedData():
    def __init__(self, raw_measurements, **kwargs):
        """
        Class that groups all Measurement taken in an environment and provides further filtering and processing
        Input
            raw_measurements: list of ROS RSSIData measurements

        Optional Parameters [default]
            poses [None]          : ROS Trajectory message with the positions where rssi data was taken
            ref_mac_dict [None] : If available, compute_data is performed using this dict instead of self generated mac dict
            filter_min_distance [0.05]   : Minimum distance in meter between data points. If two measurements are too close
                                           they are fused together
            filter_fuse_measurements [5] : Fuses measurements together (performed after filter_min_distance, so effective
                                           min distance is filter_fuse_measurement*filter_min_distance)
            filter_min_points_per_AP [10]: Minimum number of different data points where an Access Point must be available

        Functions
            filter_distance
            filter_fuse
            filter_macs
            create_all_mac_dict
            compute_data

        """
        self.data       = None
        self.poses      = kwargs.get('poses',None)
        self.poses_type = kwargs.get('poses_type','PoseWithCovarianceStamped')
        ref_mac_dict    = kwargs.get('ref_mac_dict', None)

        flag_min_distance       = kwargs.get('flag_min_distance', False)
        flag_fuse_measurements  = kwargs.get('flag_fuse_measurements', True)
        flag_min_points_per_AP  = kwargs.get('flag_fuse_min_points_per_AP', False)

        self.measurements = [Measurement(m) for m in raw_measurements]
        self.compute_pose_correspondences(raw_measurements)

        if not self.poses:
            rospy.logwarn('No poses will associate to RSSI Measurements')

        if flag_min_distance:
            self.filter_min_distance = kwargs.get('filter_min_distance',0.05) #min distance between points in [m]
            if self.poses:
                self.filter_distance()
            else:
                rospy.logwarn('Min distance filter cannot perform on non-poses data')

        if flag_fuse_measurements:
            self.filter_fuse_measurements = kwargs.get('filter_fuse_measurements', 3) #number of consecutive measurements to fuse
            self.filter_fuse()


        self.nm = len(self.measurements)     #number of valid measurements

        # compute mac dict if ref macs are not specified
        if not ref_mac_dict:
            self.create_all_mac_dict()
            if flag_min_points_per_AP:
                self.filter_min_points_per_AP = kwargs.get('min_points_per_AP',10)
                self.filter_macs()
        else:
            self.all_mac_dict = ref_mac_dict

        self.compute_data()

    def compute_pose_correspondences(self, raw_measurements):
        if not self.poses:
            rospy.logwarn('No poses input, compute pose correspondences stopped!')
            return False

        poses_time = [long(p.header.stamp.secs*1e9+p.header.stamp.nsecs) for p in self.poses]
        for i, m in enumerate(raw_measurements): # not optimized still O(n^2)
            index_after  = poses_time.index(next(x for x in poses_time if x > m.start_time_ns))
            index_before = index_after-1

            if self.poses_type == 'PoseWithCovarianceStamped':
                xa = poses[index_after].pose.pose.position.x
                ya = poses[index_after].pose.pose.position.y
                xb = poses[index_before].pose.pose.position.x
                yb = poses[index_before].pose.pose.position.y

            elif self.poses_type == 'Path':
                xa = poses[index_after].pose.position.x
                ya = poses[index_after].pose.position.y
                xb = poses[index_before].pose.position.x
                yb = poses[index_before].pose.position.y

            #interpolation
            dt = float(poses_time[index_after] - poses_time[index_before])
            dta = float((poses_time[index_after] - m.start_time_ns)/dt)
            dtb = float((m.start_time_ns - poses_time[index_before])/dt)
            self.measurements[i].pose = [dta*xb+dtb*xa, dta*yb+dtb*ya]
        return True



    def filter_distance(self,**kwargs):
        """
        Fuses measurements so there is at least min_distance between consecutive measurements
        Optional Paramteres [default]
            min_distance [class filter_min_distance]: Minimum distance enforced between consecutive points
        """
        min_distance = kwargs.get('min_distance', self.filter_min_distance)

        m_out = list()
        for m in self.measurements:
            distance = np.inf
            if m_out: #checks M is not empty
                distance = m.pose_distance(m_out[-1])

            if distance < min_distance: #if distance between samples is smaller than min_distance, fuse points
                m_out[-1] = m_out[-1]+m
            else: #else add the measurement to the list if its pose is known
                if m.pose is not None:
                    m_out.append(m)
        self.measurements = m_out
        return True

    def filter_fuse(self,**kwargs):
        """
        Fuses fuse_measurements consecutive measurements

        Optional Parameters [default]:
            fuse_measurements [class filter_fuse_measurements] : Number of consecutive measurements to filter
        """
        fuse_measurements = kwargs.get('fuse_measurements',self.filter_fuse_measurements)
        m_out = list()
        for i in range(len(self.measurements)):
            if i%fuse_measurements == 0:
                m_out.append(self.measurements[i])
            else:
                m_out[-1] = m_out[-1] + self.measurements[i]
        self.measurements = m_out
        return True

    def create_all_mac_dict(self):
        """
        Creates a single mac_dict for all measurements
        """
        temp = [mac for i in range(self.nm) for mac in self.measurements[i].mac_dict]
        mac_keys = list(set(temp)) #eliminates mac duplicates
        self.all_mac_dict = {mac_keys[i]:i for i in range(len(mac_keys))} #create dictionary

    def filter_macs(self,**kwargs):
        """
        Filters macs which do not have a value in least min_points_per_AP locations

        Optional Parameters [default]:
            min_points_per_AP [class filter_min_points_per_AP]: Minimum number of locations a valid mac should be heard
        """
        min_points_per_AP = kwargs.get('min_points_per_AP',self.filter_min_points_per_AP)
        self.compute_data()

        del_list = list() #list with macs to delete

        for mac,index in self.all_mac_dict.iteritems():
            count = np.count_nonzero(self.data['Y'][:,index])
            if count < min_points_per_AP:
                del_list.append(mac)

        filtered_macs = [mac for mac in self.all_mac_dict.keys() if not mac in del_list]
        self.all_mac_dict = {mac:i for i,mac in enumerate(filtered_macs)}

        return True

    def compute_data(self):
        """
        Computes the data output of the class.
        Output
            data: Dictionary with keys:
                X: [px2] Locations (x-y)
                Y: [pxm] Rss measurements mean
                Var: [pxm] Rss measurements variance
                N: [pxm] Number of rss measurements
                *All outputs are 2d arrays with [] the dimension p:#positions, m:#access points
        """
        if not self.poses is None:
            dataX   = np.asarray([m.pose for m in self.measurements])
        else:
            dataX = None

        dataY   = np.asarray([m.transform2vector(self.all_mac_dict,'mean') for m in self.measurements]) / 100.0 #scaled Y
        dataVar = np.asarray([m.transform2vector(self.all_mac_dict,'var') for m in self.measurements]) / 100.0 ** 2. #scaled Var
        dataN   = np.asarray([m.transform2vector(self.all_mac_dict,'len') for m in self.measurements])

        self.data = {'X':dataX,'Y':dataY,'Var':dataVar,'n':dataN}
