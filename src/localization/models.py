import GPy
from utils import data_validation
import likelihood as lh
import numpy as np
import sampling
from utils.structures import mesh
from pathloss import FLog
import scipy

class SensorModel(object):
    """
    Sensor model
    """
    def __init__(self):
        super(SensorModel,self).__init__()
        self.name = 'Sensor Model'

    ####################          To implement by derived classes          ####################
    def sample(self,measurements):
        """
        (Optional) For generative models, given sensor measurements, sample the model
        """
        raise NotImplementedError


    def sensor_update(self,particles,measurements,**kwargs):
        """
        Computes the weights for the particles given the measurements
        """
        raise NotImplementedError


class GPcore(SensorModel):
    """
    Base class for GP-based mappings. Each inherited class needs to implement
    (1) predict,
    (2) optimize

    data(Optional):
        Dictionary with entries X, Y and Var
        X:      Location inputs X, numpy array [ndp,2]
        Y:      Readings at each position X, numpy array [ndp,nap] or [ndp,]
        Var:    Readings variance, numpy array [ndp,nap] or [ndp,]
        if no args are passed, data is set to None

    kwargs
        kwargs for PathLoss class - see PathLoss.
        debug:      print debug statements - default 0

    """
    def __init__(self,data,**kwargs):
        super(GPcore,self).__init__()
        #Main Data
        self.name = 'GPcore'
        self.data = data_validation.data_structure(data)
        self.all_mac_dict   = kwargs.get('all_mac_dict',None)
        self.ndp,self.nap   = self.data['Y'].shape  #data points, #access points
        #check data
        assert self.data['X'].shape[0]==self.ndp
        assert self.data['X'].shape[1]==2

        #Likelihood params
        self.likelihood     = kwargs.get('likelihood',lh.Gaussian(**kwargs))
        #raise error if likelihood is not a derived class of Likelihood
        assert isinstance(self.likelihood,lh.Likelihood)
        self.add_noise_var  = kwargs.get('add_noise_var',.0005)

        #Xtest
        self.Xtest          = kwargs.get('Xtest',None)
        self.Xtest_num      = kwargs.get('Xtest_num',75) #num of points for each Xtest dimension
        self.Xtest_factor   = kwargs.get('Xtest_factor',.25)  #Xtest = min/max x +- x_factor*span

        if self.Xtest is None:
            x0   = np.min(self.data['X'][:,0])
            x1   = np.max(self.data['X'][:,0])
            xspn = x1-x0
            self.xmin = x0-self.Xtest_factor*xspn
            self.xmax = x1+self.Xtest_factor*xspn

            y0   = np.min(self.data['X'][:,1])
            y1   = np.max(self.data['X'][:,1])
            yspn = y1-y0
            self.ymin = y0-self.Xtest_factor*yspn
            self.ymax = y1+self.Xtest_factor*yspn

            temp_x = np.linspace(self.xmin,self.xmax,self.Xtest_num)
            temp_y = np.linspace(self.ymin,self.ymax,self.Xtest_num)
            self.Xtest  = mesh(temp_x,temp_y)

        #Sampling params
        self.sampling   = kwargs.get('sampling',sampling.accept_reject_by_regions_map)
        self.nsamples   = kwargs.get('nsamples',800)

        #Debug params
        self.debug      = kwargs.get('debug',False)
        self.verbose    = kwargs.get('verbose',False)

        if self.debug:
            print 'class GP init works'

    def save(self,filepath='last_model.p'):
        """
        Pickle current model.
        If no filepath is given, it is saved at <currentpath>/last_model.p
        """
        try:
            import cPickle as pickle
        except ImportError:
            import pickle

        with open(filepath, 'wb') as f:
            pickle.dump(self.__dict__,f,2)

        return filepath

    def load(self,filepath='last_model.p'):
        """
        Load a previously pickled model from filepath.
        If no filepath is given, <currentpath>/lastmodel
        """
        try:
            import cPickle as pickle
        except ImportError:
            import pickle

        with open(filepath, 'rb') as f:
            tmp_dict = pickle.load(f)

        self.__dict__.update(tmp_dict)

        return True

    ####################                Sensor class function               ####################
    def sample(self,measurement,**kwargs):
        """
        (Optional) For generative models, given sensor measurements, sample the model
        """
        #Sample from the joint pdf
        def tmp_fun(x,**kwargs):
            return self.jointpdf(x,measurement,**kwargs).flatten()

        return self.sampling(tmp_fun,**kwargs)[0]


    def jointpdf(self,Xn,Yn,**kwargs):
        """
        Wrapper around likelihood.jointpdf function
        """
        return self.likelihood.jointpdf(self,Xn,Yn,**kwargs)


    ####################          To implement by derived classes          ####################
    def optimize(self,**kwargs):
        """
        Main optimization function. Must optimize all class parameters that need tunning.
        """
        raise NotImplementedError

    def predict(self,X,**kwargs):
        """
        Given the model computes the estimated value and variance at points X

        X:
            Points where the pathloss function is calculated
        """
        raise NotImplementedError

#################################################################################################
class GP(GPcore):
    """
    Derived class from GPcore. Implements a basic GP - identical to GPy but handling the same
    input formats used in this library.
    """
    def __init__(self,data,**kwargs):
        super(GP,self).__init__(data,**kwargs)

        self.name = 'GP'
        if self.data is None:
            self.gp = None
        else:
            self.gp    = '\n-> Run optimize()'

        if self.debug:
            print 'class GP init works'

    def optimize(self,**kwargs):
        self.gp = GPy.models.GPRegression(self.data['X'],self.data['Y'],**kwargs)
        self.gp.optimize()

    def predict(self,X,**kwargs):
        Y_gp,Yvar_gp = self.gp.predict(X)
        return np.clip(Y_gp,0,np.inf), Yvar_gp + self.add_noise_var

    def __str__(self):
        """
        Overloading __str__ to make print statement meaningful

        <format>
        Name                        :
        data
            X                       :
            Y                       :
            Var                     :

        Gaussian Process
            --GPy print

        """
        to_print = '{}  : {}\n'.format('Name'.ljust(34),self.name)
        to_print = to_print + 'data\n'
        to_print = to_print + '  {}  : [{},{}]\n'.format('X'.ljust(32),
                        str(self.data['X'].shape[0]).rjust(4),
                        str(self.data['X'].shape[1]).rjust(4))
        to_print = to_print + '  {}  : [{},{}]\n'.format('Y'.ljust(32),
                        str(self.data['Y'].shape[0]).rjust(4),
                        str(self.data['Y'].shape[1]).rjust(4))
        to_print = to_print + '  {}  : [{},{}]\n'.format('Var'.ljust(32),
                        str(self.data['Var'].shape[0]).rjust(4),
                        str(self.data['Var'].shape[1]).rjust(4))
        to_print = to_print + '\nGaussian Process'
        to_print = to_print + self.gp.__str__().replace('\n','\n  ')

        return to_print

class hGP(GPcore):
    """
    Derived class from GPcore. Implements a hybrid pathloss-fingerprinting technique.
    Additionally uses for its implementation a pathloss.classes.PathLoss class
    """
    def __init__(self,data,**kwargs):
        super(hGP,self).__init__(data,**kwargs)
        self.name = 'GP with pathloss'
        self.pathloss = kwargs.get('pathloss',FLog(self.data))
        #raises error if pathloss is not a derived class of PathLoss
        #assert isinstance(self.pathloss,PathLoss)

        self.nsd = kwargs.get('nsd',2.) #Number of standard deviations used to recalculate pathloss variance, 3 = 99.5%, 2 = 95%

        self.ndata = None
        self.gp    = '\n-> Run optimize()'

        if self.debug:
            print 'class hGP init works'

    def optimize(self,**kwargs):
        AP = kwargs.get('AP','all')
        ntimes = kwargs.get('ntimes',1)
        self.pathloss.optimize(AP=AP,ntimes=ntimes,verbose=self.verbose)
        self.ndata = self.pathloss.new_data()

        self.gp = GPy.models.GPRegression(self.ndata['X'],self.ndata['Y'],**kwargs)
        self.gp.optimize()

    def predict(self,X,**kwargs):
        flag_pl = kwargs.get('pl',False)    #returns path loss component
        flag_gp = kwargs.get('gp',False)    #returns GP component

        Y_gp,Yvar_gp = self.gp.predict(X)
        if flag_gp:
            return Y_gp, Yvar_gp

        Y_pl = self.pathloss.mean_val(X,bounded=False)
        if flag_pl:
            return Y_pl

        Ymean = Y_gp+Y_pl
        Ysd   = (1./self.nsd)*np.clip(self.nsd*Yvar_gp**0.5+Ymean-
                    np.clip(Ymean,0,np.inf), 1e-2, np.inf)

        return np.clip(Ymean,0,np.inf), Ysd**2 + self.add_noise_var

    def __str__(self):
        """
        Overloading __str__ to make print statement meaningful

        <format>
        Name                        :
        data
            X                       :
            Y                       :
            Var                     :

        PathLoss
            Name                    :
            Log-likelihood          :
            Number of parameters    :

        Gaussian Process
            --GPy print

        """
        to_print = '{}  : {}\n'.format('Name'.ljust(34),self.name)
        to_print = to_print + 'data\n'
        to_print = to_print + '  {}  : [{},{}]\n'.format('X'.ljust(32),
                        str(self.data['X'].shape[0]).rjust(4),
                        str(self.data['X'].shape[1]).rjust(4))
        to_print = to_print + '  {}  : [{},{}]\n'.format('Y'.ljust(32),
                        str(self.data['Y'].shape[0]).rjust(4),
                        str(self.data['Y'].shape[1]).rjust(4))
        to_print = to_print + '  {}  : [{},{}]\n'.format('Var'.ljust(32),
                        str(self.data['Var'].shape[0]).rjust(4),
                        str(self.data['Var'].shape[1]).rjust(4))
        to_print = to_print + '\nPathLoss\n'
        to_print = to_print + '  {}  : {}\n'.format('Name'.ljust(32),self.pathloss.name)
        to_print = to_print + '  {}  : {}\n'.format('Log-likelihood'.ljust(32),
                        str(self.pathloss.nll()))
        to_print = to_print + '  {}  : {}\n'.format('Number of Parameters'.ljust(32),
                        str(np.prod(self.pathloss.params.shape)))
        to_print = to_print + '\nGaussian Process'
        to_print = to_print + self.gp.__str__().replace('\n','\n  ')

        return to_print
