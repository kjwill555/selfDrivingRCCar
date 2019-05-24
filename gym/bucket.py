import os
import sys
import json
import random
import glob

from autopilot import AutoPilot

import numpy as np
import pandas as pd
from PIL import Image


""" A datastore to store sensor data in a key, value format. """
class Bucket(object):

    def __init__(self, path):
        self.path = os.path.expanduser(path)
        self.df = None
        self.current_ix = self.get_last_ix() + 1


    """ Get the highest record index in the folder """
    def get_last_ix(self):
        index = self.get_index()
        if len(index) >= 1:
            return max(index)
        return -1


    """ Update the dataframe with all the data in the folder """
    def update_df(self):
        df = pd.DataFrame([self.get_json_record(i) for i in self.get_index(shuffled=False)])
        self.df = df


    """ Return the underlying dataframe """
    def get_df(self):
        if self.df is None:
            self.update_df()
        return self.df


    """
    Get a list of all the indices in the folder

    Parameters:
    shuffled (bool) : Whether to shuffle the list before returning
    """
    def get_index(self, shuffled=True):
        files = next(os.walk(self.path))[2]
        record_files = [f for f in files if f[:6] == 'record']

        def get_file_ix(file_name):
            try:
                name = file_name.split('.')[0]
                num = int(name.split('_')[1])
            except:
                num = 0
            return num

        nums = [get_file_ix(f) for f in record_files]

        if shuffled:
            random.shuffle(nums)
        else:
            nums = sorted(nums)

        return nums


    """ Returns the number of records in the folder """
    def get_num_records(self):
        import glob
        files = glob.glob(os.path.join(self.path, 'record_*.json'))
        return len(files)


    """
    Converts the file paths in the given dict to absolute paths

    Parameters:
    record_dict (dict) : The dict in which to convert the paths
    """
    def make_record_paths_absolute(self, record_dict):
        d = {}
        for k, v in record_dict.items():
            if type(v) == str:  # Filename
                if '.' in v:
                    v = os.path.join(self.path, v)
            d[k] = v

        return d


    """
    Returns the path to the record corresponding to the given index
    
    Parameters:
    ix (int) : The index of the record
    """
    def get_json_record_path(self, ix):
        return os.path.join(self.path, 'record_' + str(ix) + '.json')


    """
    Loads the specified json record file

    Parameters:
    ix (int) : The index of the record to load
    """
    def get_json_record(self, ix):
        path = self.get_json_record_path(ix)
        try:
            with open(path, 'r') as fp:
                json_data = json.load(fp)
        except UnicodeDecodeError:
            raise Exception('Bad record: %d.' % ix)
        except FileNotFoundError:
            raise
        except:
            print('Unexpected error: {}'.format(sys.exc_info()[0]))
            raise

        record_dict = self.make_record_paths_absolute(json_data)
        return record_dict


    """
    Gets the key/value pairs from the desired record

    Parameters:
    ix (int) : The index of the record to retrieve
    """
    def get_record(self, ix):
        json_data = self.get_json_record(ix)
        data = self.read_record(json_data)
        return data


    """
    Reads the given record and loads the image file

    Parameters:
    record_dict (dict) : The json dict to read
    """
    def read_record(self, record_dict):
        data = {}
        for key, val in record_dict.items():
            # Load image file
            if key == 'image':
                img = Image.open((val))
                val = np.atleast_3d(np.array(img))

            data[key] = val
        return data


    """
    Creates a record generator
    
    Parameters:
    record_transform (function) : Transform function to apply to the records
    shuffle (bool) : Whether to shuffle the order of the records
    df (Dataframe) : Dataframe to create the generator from.

    Returns:
    dict : A single record
    """
    def get_record_gen(self, record_transform=None, shuffle=True, df=None):
        if df is None:
            df = self.get_df()

        while True:
            for _ in self.df.iterrows():
                if shuffle:
                    record_dict = df.sample(n=1).to_dict(orient='record')[0]

                record_dict = self.read_record(record_dict)

                if record_transform:
                    record_dict = record_transform(record_dict)

                yield record_dict


    """
    Returns batches of records.

    Additionally, each record in a batch is split up into a dict with inputs:list of values. By specifying keys as a subset of the inputs, unnecessary data can be filtered out.

    Parameters
    ----------
    keys (list of strings) : List of keys to filter out. If None, all inputs are included.
    batch_size (int) : The number of records in one batch.
    record_transform (function) : Transform to apply to the records
    shuffle (bool) : Whether to shuffle the data
    df (Dataframe) : The dataframe to create the batches from

    Returns
    -------
    dict : A dict with keys mapping to the specified keys, and values lists of size batch_size.
    """
    def get_batch_gen(self, keys=None, batch_size=128, record_transform=None, shuffle=True, df=None):
        record_gen = self.get_record_gen(record_transform=record_transform, shuffle=shuffle, df=df)

        if keys is None:
            keys = list(self.df.columns)

        while True:
            record_list = [next(record_gen) for _ in range(batch_size)]

            batch_arrays = {}
            for i, k in enumerate(keys):
                arr = np.array([r[k] for r in record_list])
                batch_arrays[k] = arr
            yield batch_arrays


    """
    Returns a training/validation set. The records are always shuffled.

    Parameters
    ----------
    X_keys (list of strings) : List of the feature(s) to use.
    Y_keys (list of strings) : List of the label(s) to use.
    batch_size (int) : Size of the batches
    record_transform (function) : Transform to apply to the records
    df (Dataframe) : The dataframe to create the generator from

    Returns
    -------
    A tuple (X, Y), where X is a two dimensional array ( len(X_keys) x batch_size ) and Y is a two dimensional array ( len(Y_keys) x batch_size ).
    """
    def get_train_gen(self, X_keys, Y_keys,
                      batch_size=128,
                      record_transform=None,
                      df = None):
        batch_gen = self.get_batch_gen(X_keys + Y_keys,
                                       batch_size=batch_size,
                                       record_transform=record_transform,
                                       df=df)

        while True:
            batch = next(batch_gen)
            X = [batch[k] for k in X_keys]
            Y = [batch[k] for k in Y_keys]
            yield X, Y


    """
    Create generators for the training and validation sets.

    Parameters
    ----------
    X_keys (list of strings) : List of the feature(s) to use.
    Y_keys (list of strings) : List of the label(s) to use.
    batch_size (int) : Size of the batches
    train_frac (float) : Training/validation set split
    train_record_transform (function) : Transform function for the training set
    val_record_transform (function) : Transform  function for the validation set.

    Returns
    -------
    A tuple (train_gen, val_gen), where where train_gen is the training set generator, and
    val_gen the validation set generator.
    """
    def get_train_val_gen(self, X_keys, Y_keys, batch_size=128, train_frac=.8,
                          train_record_transform=None, val_record_transform=None):
        train_df = self.df.sample(frac=train_frac, random_state=200)
        val_df = self.df.drop(train_df.index)

        train_gen = self.get_train_gen(X_keys=X_keys, Y_keys=Y_keys, batch_size=batch_size,
                                       record_transform=train_record_transform, df=train_df)

        val_gen = self.get_train_gen(X_keys=X_keys, Y_keys=Y_keys, batch_size=batch_size,
                                     record_transform=val_record_transform, df=val_df)

        return train_gen, val_gen


""" Combines multiple buckets into one """
class BucketGroup(Bucket):

    def __init__(self, bucket_paths_arg):
        bucket_paths = self.expand_path_arg(bucket_paths_arg)
        self.buckets = [Bucket(path) for path in bucket_paths]

        record_count = 0
        for t in self.buckets:
            t.update_df()
            record_count += len(t.df)

        self.df = pd.concat([t.df for t in self.buckets], axis=0, join='inner')


    """ Gets the number of buckets this instance contains """
    def get_num_buckets(self):
        return len(self.buckets)


    """ Gets the total number of records """
    def get_num_records(self):
        return len(self.df)


    """
    Expands any wildcards in a given path

    Parameters:
    path (str) : The path to expand

    Returns:
    list of strings : The paths that matched the input
    """
    def expand_path_mask(self, path):
        matches = []
        path = os.path.expanduser(path)
        for file in glob.glob(path):
            if os.path.isdir(file):
                matches.append(os.path.join(os.path.abspath(file)))

        return matches


    """
    Expands all of the input paths

    Parameters:
    path_str (str) : The list of strings to split and expand

    Returns:
    list of strings : The list of expanded paths
    """
    def expand_path_arg(self, path_str):
        path_list = path_str.split(",")
        expanded_paths = []
        for path in path_list:
            paths = self.expand_path_mask(path)
            expanded_paths += paths

        return expanded_paths

