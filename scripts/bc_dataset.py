import numpy as np
import h5py
import os
import argparse
import sys
from robomimic.utils.file_utils import create_hdf5_filter_key

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import utils.demo as demo

OBS_FILE_NAMES = ['gray_obs.hdf5', 'history_obs.hdf5']
ACTION_FILE_NAME = 'action.hdf5'

def batch_data(data_dir, dataset_name="dataset"):

    demo_file_paths = {}
    demo_names = os.listdir(data_dir)
    for demo_name in demo_names:
        data_paths = {
            'action': os.path.join(data_dir, demo_name, ACTION_FILE_NAME),
            'obs': [os.path.join(data_dir, demo_name, file_name) for file_name in OBS_FILE_NAMES],
        }
        
        if not os.path.isfile(data_paths['action']):
            continue
        if not all([os.path.isfile(path) for path in data_paths['obs']]):
            continue
        demo_file_paths[demo_name] = data_paths

    count_demos = 0
    num_demos = len(demo_file_paths.keys())
    out_dict = {}

    dataset_path = os.path.join(path, 'datasets', '{}.hdf5').format(dataset_name)
    out_file = h5py.File(dataset_path, 'w')
    out_file.attrs["total"] = num_demos
    out_file.attrs["env_args"] = ""
    out_data = out_file.create_group("data")
    count_demos = 0

    # Process data with a limited worker
    for ep_id, paths in demo_file_paths.items():
        print ('[{}/{}] processing demo: {}...'.format(count_demos, num_demos, ep_id))

        failed = False
        samples = []
        obs = {}
        # Read raw data
        for path in paths['obs']:
            print('\tworking on {}...'.format(path))
            data_file = h5py.File(path, 'r')
            demo_data = data_file['obs']                
            for key in demo_data.keys():
                if 'rgb' in key:
                    obs[key] = np.array(demo_data[key], dtype='uint8')
                if 'gray' in key:
                    obs[key] = np.array(demo_data[key], dtype='uint8')
                else:
                    obs[key] = np.array(demo_data[key])
                # samples.append((key, demo_data[key].shape))
                samples.append(demo_data[key].shape[0])
            data_file.close()

        print('\tworking on {}...'.format(paths['action']))
        data_file = h5py.File(paths['action'], 'r')
        actions = np.array(data_file['action'])
        samples.append(actions.shape[0])
        data_file.close()

        # Check if sample size is equal
        if not all([sample == samples[0] for sample in samples]):
            print('sample size not equal, skip this demo... {}'.format(samples))
            continue

        dones = np.zeros(samples[0])
        dones[-1] = 1

        # Save data
        ep_grp = out_data.create_group("demo_{}".format(count_demos))
        obs_grp = ep_grp.create_group(f"obs")
        for key, data_value in obs.items():
            if 'rgb' in key or 'gray' in key:
                obs_grp.create_dataset(key, data=data_value, compression="gzip", chunks=True, dtype='uint8')
            else:
                obs_grp.create_dataset(key, data=data_value, compression="gzip", chunks=True, dtype='float32')
        ep_grp.create_dataset("dones", data=dones, dtype='uint8')
        ep_grp.create_dataset("rewards", data=dones, dtype='float32')
        ep_grp.create_dataset("actions", data=actions, dtype='float32')
        ep_grp.attrs["num_samples"] = samples[0]
        ep_grp.attrs["tag"] = ep_id

        count_demos += 1

    out_file.close()


def split_train_val_from_hdf5(hdf5_path, val_ratio=0.1, filter_key=None):
    """
    Splits data into training set and validation set from HDF5 file.
    Args:
        hdf5_path (str): path to the hdf5 file
            to load the transitions from
        val_ratio (float): ratio of validation demonstrations to all demonstrations
        filter_key (str): if provided, split the subset of demonstration keys stored
            under mask/@filter_key instead of the full set of demonstrations
    """

    # retrieve number of demos
    f = h5py.File(hdf5_path, "r")
    if filter_key is not None:
        print("using filter key: {}".format(filter_key))
        demos = sorted([elem.decode("utf-8") for elem in np.array(f["mask/{}".format(filter_key)])])
    else:
        demos = sorted(list(f["data"].keys()))
    num_demos = len(demos)
    f.close()

    # get random split
    num_demos = len(demos)
    num_val = int(val_ratio * num_demos)
    mask = np.zeros(num_demos)
    mask[:num_val] = 1.
    np.random.shuffle(mask)
    mask = mask.astype(int)
    train_inds = (1 - mask).nonzero()[0]
    valid_inds = mask.nonzero()[0]
    train_keys = [demos[i] for i in train_inds]
    valid_keys = [demos[i] for i in valid_inds]
    print("{} validation demonstrations out of {} total demonstrations.".format(num_val, num_demos))

    # pass mask to generate split
    name_1 = "train"
    name_2 = "valid"
    if filter_key is not None:
        name_1 = "{}_{}".format(filter_key, name_1)
        name_2 = "{}_{}".format(filter_key, name_2)

    train_lengths = create_hdf5_filter_key(hdf5_path=hdf5_path, demo_keys=train_keys, key_name=name_1)
    valid_lengths = create_hdf5_filter_key(hdf5_path=hdf5_path, demo_keys=valid_keys, key_name=name_2)

    print("Total number of train samples: {}".format(np.sum(train_lengths)))
    print("Average number of train samples {}".format(np.mean(train_lengths)))

    print("Total number of valid samples: {}".format(np.sum(valid_lengths)))
    print("Average number of valid samples {}".format(np.mean(valid_lengths)))
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--path",
        type=str,
        default='./data/sim',
        help="demo task name",
    )
    parser.add_argument(
        "--label",
        type=str,
        default=None,
        help="dataset label",
    )
    parser.add_argument(
        "--filter_key",
        type=str,
        default=None,
        help="if provided, split the subset of trajectories in the file that correspond to\
            this filter key into a training and validation set of trajectories, instead of\
            splitting the full set of trajectories",
    )
    parser.add_argument(
        "--ratio",
        type=float,
        default=0.1,
        help="validation ratio, in (0, 1)"
    )
    args = parser.parse_args()

    # seed to make sure results are consistent
    np.random.seed(0)

    if not os.path.isdir("./data/datasets"):
        os.makedirs("./data/datasets")
        print("=> created dataset directory")
    else:
        print("=> dataset directory already exists, skipping...")

    dataset_name = "dataset"
    if args.label is not None:
        dataset_name += "_" + args.label
    dataset_path = os.path.join('./data','datasets', '{}.hdf5').format(dataset_name)
    if os.path.isfile(dataset_path):
        print("=> dataset already exists, skipping...")
    else:
        batch_data(data_dir=args.path, dataset_name=dataset_name)
        print("=> dataset created at: {}".format(dataset_path))

    split_train_val_from_hdf5(dataset_path, val_ratio=args.ratio, filter_key=args.filter_key)
    print("=> dataset split into train and validation sets")
