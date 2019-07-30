import cPickle as pickle

def save(var,filepath='var.p'):
    """
    Pickle var.
    If no filepath is given, it is saved at <currentpath>/var.p
    """
    try:
        import cPickle as pickle
    except ImportError:
        import pickle

    with open(filepath, 'wb') as f:
        pickle.dump(var,f,2)
    return True

def load(filepath='last_model.p'):
    """
    Load a previously pickled var from filepath.
    If no filepath is given, <currentpath>/var.p
    """
    try:
        import cPickle as pickle
    except ImportError:
        import pickle

    with open(filepath, 'rb') as f:
        var = pickle.load(f)
    return var

def load(path):
    with open(path, 'rb') as f:
        return pickle.load(f)
