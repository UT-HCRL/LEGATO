import argparse
import json
import numpy as np
import os
import sys

import torch
from torch.utils.data import DataLoader

import robomimic.utils.train_utils as TrainUtils
import robomimic.utils.torch_utils as TorchUtils
import robomimic.utils.obs_utils as ObsUtils
import robomimic.utils.file_utils as FileUtils
from robomimic.utils.log_utils import PrintLogger
from robomimic.envs.env_base import EnvType
from robomimic.utils.dataset import SequenceDataset

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from mimic import WandbLogger, TrainConfig, algo_factory
from mimic import TrainConfig, policy_from_checkpoint


## Load demostration data
def load_data_for_training(config, obs_keys):
    """
    Data loading at the start of an algorithm.
    Args:
        config (BaseConfig instance): config object
        obs_keys (list): list of observation modalities that are required for
            training (this will inform the dataloader on what modalities to load)
    Returns:
        train_dataset (SequenceDataset instance): train dataset object
        valid_dataset (SequenceDataset instance): valid dataset object (only if using validation)
    """

    # config can contain an attribute to filter on
    filter_by_attribute = config.train.hdf5_filter_key

    # load the dataset into memory
    assert not config.train.hdf5_normalize_obs, "no support for observation normalization with validation data yet"
    train_filter_by_attribute = "train"
    valid_filter_by_attribute = "valid"
    if filter_by_attribute is not None:
        train_filter_by_attribute = "{}_{}".format(filter_by_attribute, train_filter_by_attribute)
        valid_filter_by_attribute = "{}_{}".format(filter_by_attribute, valid_filter_by_attribute)

    def get_dataset(filter_by_attribute):
        return SequenceDataset(
            hdf5_path=config.train.data,
            obs_keys=obs_keys,
            dataset_keys=config.train.dataset_keys,
            load_next_obs=False,
            frame_stack=config.train.frame_stack,
            seq_length=config.train.seq_length,
            pad_frame_stack=config.train.pad_frame_stack,
            pad_seq_length=config.train.pad_seq_length,
            get_pad_mask=False,
            goal_mode=config.train.goal_mode,
            hdf5_cache_mode=config.train.hdf5_cache_mode,
            hdf5_use_swmr=config.train.hdf5_use_swmr,
            hdf5_normalize_obs=config.train.hdf5_normalize_obs,
            filter_by_attribute=filter_by_attribute,
        )

    train_dataset = get_dataset(train_filter_by_attribute)
    valid_dataset = get_dataset(valid_filter_by_attribute)

    return train_dataset, valid_dataset


## Train Navigation Controller
def train(config, device, info, ckpt_path=None):
    """
    Train a model using the algorithm.
    """

    # Configuration
    np.random.seed(config.train.seed)
    torch.manual_seed(config.train.seed)

    print("\n============= New Training Run with Config =============")
    print(config)
    print("")
    log_dir, ckpt_dir, video_dir = TrainUtils.get_exp_dir(config)

    json.dump(info['config'], open(os.path.join(log_dir,"config.json"), "w"))

    if config.experiment.logging.terminal_output_to_txt:
        # log stdout and stderr to a text file
        logger = PrintLogger(os.path.join(log_dir, 'log.txt'))
        sys.stdout = logger
        sys.stderr = logger

    # read config to set up metadata for observation modalities (e.g. detecting rgb observations)
    ObsUtils.initialize_obs_utils_with_config(config)

    # make sure the dataset exists
    dataset_path = os.path.expanduser(config.train.data)
    if not os.path.exists(dataset_path):
        raise Exception("Dataset at provided path {} not found!".format(dataset_path))

    # Configure meta data
    env_meta = {
        "env_name": "legato",
        "type": EnvType.GYM_TYPE,
        "env_kwargs": {}
    }
    shape_meta = FileUtils.get_shape_metadata_from_dataset(
        dataset_path=config.train.data,
        all_obs_keys=config.all_obs_keys,
        verbose=True
    )

    if ckpt_path is not None:
        print("Loading checkpoint from {}".format(ckpt_path))
        ckpt_model, ckpt_dict = policy_from_checkpoint(ckpt_path=ckpt_path, device=device)
        model=ckpt_model.policy
        model.set_train()
    else:
        # BC Model
        model = algo_factory(
            algo_name=config.algo_name,
            config=config,
            obs_key_shapes=shape_meta["all_shapes"],
            ac_dim=shape_meta["ac_dim"],
            device=device
        )

    print("\n============= Model Summary =============")
    print(model)

    wandb_logger = WandbLogger(project=info['note']['experiment'],
                               task="legato",
                               path=log_dir, 
                               config=info['config'], 
                               model=model.nets)

    # Load data set
    obs_normalization_stats = None
    if config.train.hdf5_normalize_obs:
        obs_normalization_stats = trainset.get_obs_normalization_stats()

    trainset, validset = load_data_for_training(
        config, obs_keys=shape_meta["all_obs_keys"])
    train_sampler = trainset.get_dataset_sampler()
    print("\n============= Training Dataset =============")
    print(trainset)
    print("")

    train_loader = DataLoader(
        dataset=trainset,
        sampler=train_sampler,       # no custom sampling logic (uniform sampling)
        batch_size=config.train.batch_size,     # batches of size 100
        shuffle=True,
        pin_memory=True,
        num_workers=config.train.num_data_workers,
        drop_last=True# don't provide last batch in dataset pass if it's less than 100 in size
    )

    valid_sampler = validset.get_dataset_sampler()
    valid_loader = DataLoader(
        dataset=validset,
        sampler=valid_sampler,
        batch_size=config.train.batch_size,
        shuffle=(valid_sampler is None),
        num_workers=0,
        drop_last=True
        )

    # Train
    best_epoch_interval = 100
    best_valid_loss = None
    best_training_loss = None
    for epoch in range(1, config.train.num_epochs + 1):
        should_save_ckpt = False
        train_step_log = TrainUtils.run_epoch(model=model, data_loader=train_loader, epoch=epoch, num_steps=config.experiment.epoch_every_n_steps)
        model.on_epoch_end(epoch)
        print("Train Epoch {}: Loss {}".format(epoch, train_step_log["Loss"]))
        wandb_logger.log_train_data({'Train/{}'.format(key): item for key, item in train_step_log.items()}, epoch)

        if best_training_loss is None or train_step_log["Loss"] < best_training_loss:
            should_save_ckpt = True
            epoch_ckpt_name = "model_best_training_at_epoch_{}".format(epoch - (epoch%best_epoch_interval))
            print("Best Model Loss: Loss {}".format(train_step_log["Loss"]))

        with torch.no_grad():
            valid_step_log = TrainUtils.run_epoch(model=model, data_loader=valid_loader, epoch=epoch, validate=True, num_steps=config.experiment.validation_epoch_every_n_steps)
        valid_check = "Loss" in valid_step_log
        if valid_check and (best_valid_loss is None or (valid_step_log["Loss"] <= best_valid_loss)):
            best_valid_loss = valid_step_log["Loss"]
            valid_epoch_ckpt_name = "model_best_validation_at_epoch_{}".format(epoch - (epoch%best_epoch_interval))
            should_save_ckpt = True
            print("Best Validation Loss: Loss {}".format(valid_step_log["Loss"]))

        if should_save_ckpt:
            print("Saving checkpoint")
            TrainUtils.save_model(
                model=model,
                config=config,
                env_meta=env_meta,
                shape_meta=shape_meta,
                ckpt_path=os.path.join(ckpt_dir, epoch_ckpt_name + ".pth"),
                obs_normalization_stats=obs_normalization_stats,
            )

        if (epoch % 20 == 0 and epoch>2000) or (epoch % 100 == 0):
            should_save_ckpt = True
            epoch_ckpt_name = "epoch_{}".format(epoch)
            TrainUtils.save_model(
                model=model,
                config=config,
                env_meta=env_meta,
                shape_meta=shape_meta,
                ckpt_path=os.path.join(ckpt_dir, epoch_ckpt_name + ".pth"),
                obs_normalization_stats=obs_normalization_stats,
            )

        if epoch >= config.experiment.rollout.warmstart:
            pass
        
        wandb_logger.log_train_data({'Eval/{}'.format(key): item for key, item in valid_step_log.items()}, epoch)
    return model

    
def main():
    
    cwd = os.getcwd()
    sys.path.append(cwd)
    
    parser = argparse.ArgumentParser()

    # External config file that overwrites default config
    parser.add_argument(
        "--config",
        type=str,
        default='sim_train',
        help="path to a config json that will be used to override the default settings. \
              For example, --config=CONFIG.json will load a .json config file at ./config/nav/CONFIG.json.\
              If omitted, default settings are used. This is the preferred way to run experiments.")
    parser.add_argument("--exp", type=str, default='default', help="experiment name")
    parser.add_argument("--data_path", type=str, default=None, help="path to dataset file")
    parser.add_argument("--device", type=str, default=None, help="device to use: cuda or cpu")
    parser.add_argument("--ckpt_path", type=str, default=None, help="path to checkpoint if resuming training")
    args = parser.parse_args()

    ckpt_path=args.ckpt_path

    ext_cfg = json.load(open('./configs/{}.json'.format(args.config), 'r'))
    ext_cfg['experiment']['name'] = args.exp
    ext_cfg['train'].update({'data': args.data_path,
                            'output_dir':os.path.join(cwd, 'save')
                            })

    config = TrainConfig()

    info = {}
    info['config'] = ext_cfg
    info['note'] = {'experiment': args.exp, 'dataset': args.data_path}

    # update config with external json - this will throw errors if
    # the external config has keys not present in the base algo config
    with config.values_unlocked():
        config.update(ext_cfg)
        
    if args.device is not None:
        device = args.device
    else:
        device = TorchUtils.get_torch_device(try_to_use_cuda=True)
    print("This experiment is using device: {}...".format(device))

    model = train(config, device, info, ckpt_path=ckpt_path)


if __name__ == "__main__":
    main()
