from collections import OrderedDict

import torch
import torch.nn as nn
import torch.nn.functional as F

import robomimic.models.base_nets as BaseNets
import robomimic.utils.obs_utils as ObsUtils
import robomimic.utils.torch_utils as TorchUtils
from robomimic.algo import PolicyAlgo, RolloutPolicy
from robomimic.algo.bc import *
from robomimic.utils.file_utils import maybe_dict_from_checkpoint, config_from_checkpoint, algo_name_from_checkpoint  # noqa: E402
from .policy_nets import *

import mimic.dhb_loss as dhb_loss
import os, sys

def algo_factory(algo_name, config, obs_key_shapes, ac_dim, device):
    """
    Factory function for creating algorithms based on the algorithm name and config.

    Args:
        algo_name (str): the algorithm name

        config (BaseConfig instance): config object

        obs_key_shapes (OrderedDict): dictionary that maps observation keys to shapes

        ac_dim (int): dimension of action space

        device (torch.Device): where the algo should live (i.e. cpu, gpu)
    """

    # @algo_name is included as an arg to be explicit, but make sure it matches the config
    assert algo_name == config.algo_name and algo_name == "bc"

    algo_config = config.algo
    obs_config = config.observation

    gaussian_enabled = ("gaussian" in algo_config and algo_config.gaussian.enabled)
    gmm_enabled = ("gmm" in algo_config and algo_config.gmm.enabled)
    vae_enabled = ("vae" in algo_config and algo_config.vae.enabled)

    if algo_config.rnn.enabled:
        if gmm_enabled:
            if len(algo_config.gmm.discrete_indices)==0:
                assert len(algo_config.gmm.discrete_indices) == len(algo_config.gmm.discrete_classes)
                algo_cls = BC_RNN_GMM
            else:
                algo_cls = BC_RNN_HYBRID
        else:
            if len(algo_config.gmm.discrete_indices)==0:
                assert len(algo_config.gmm.discrete_indices) == len(algo_config.gmm.discrete_classes)
                algo_cls = BC_RNN
            else:
                raise NotImplementedError
    elif algo_config.transformer.enabled:
        if gmm_enabled:
            if len(algo_config.gmm.discrete_indices)==0:
                assert len(algo_config.gmm.discrete_indices) == len(algo_config.gmm.discrete_classes)
                algo_cls = BC_Transformer_GMM
            else:
                algo_cls = BC_Transformer_HYBRID
        else:
            if len(algo_config.gmm.discrete_indices)==0:
                assert len(algo_config.gmm.discrete_indices) == len(algo_config.gmm.discrete_classes)
                algo_cls = BC_Transformer
            else:
                raise NotImplementedError
    else:
        assert sum([gaussian_enabled, gmm_enabled, vae_enabled]) <= 1
        if gaussian_enabled:
            algo_cls =  BC_Gaussian
        if gmm_enabled:
            algo_cls =  BC_GMM
        if vae_enabled:
            algo_cls = BC_VAE
        else:
            algo_cls = BC

    # create algo instance
    return algo_cls(
        algo_config=algo_config,
        obs_config=obs_config,
        global_config=config,
        obs_key_shapes=obs_key_shapes,
        ac_dim=ac_dim,
        device=device,
    )


def policy_from_checkpoint(device=None, ckpt_path=None, ckpt_dict=None, verbose=False):
    """
    This function restores a trained policy from a checkpoint file or
    loaded model dictionary.

    Args:
        device (torch.device): if provided, put model on this device

        ckpt_path (str): Path to checkpoint file. Only needed if not providing @ckpt_dict.

        ckpt_dict(dict): Loaded model checkpoint dictionary. Only needed if not providing @ckpt_path.

        verbose (bool): if True, include print statements

    Returns:
        model (RolloutPolicy): instance of Algo that has the saved weights from
            the checkpoint file, and also acts as a policy that can easily
            interact with an environment in a training loop

        ckpt_dict (dict): loaded checkpoint dictionary (convenient to avoid
            re-loading checkpoint from disk multiple times)
    """
    ckpt_dict = maybe_dict_from_checkpoint(ckpt_path=ckpt_path, ckpt_dict=ckpt_dict)

    # algo name and config from model dict
    algo_name, _ = algo_name_from_checkpoint(ckpt_dict=ckpt_dict)
    config, _ = config_from_checkpoint(algo_name=algo_name, ckpt_dict=ckpt_dict, verbose=verbose)

    # read config to set up metadata for observation modalities (e.g. detecting rgb observations)
    ObsUtils.initialize_obs_utils_with_config(config)

    # shape meta from model dict to get info needed to create model
    shape_meta = ckpt_dict["shape_metadata"]

    # maybe restore observation normalization stats
    obs_normalization_stats = ckpt_dict.get("obs_normalization_stats", None)
    if obs_normalization_stats is not None:
        assert config.train.hdf5_normalize_obs
        for m in obs_normalization_stats:
            for k in obs_normalization_stats[m]:
                obs_normalization_stats[m][k] = np.array(obs_normalization_stats[m][k])

    if device is None:
        # get torch device
        device = TorchUtils.get_torch_device(try_to_use_cuda=config.train.cuda)

    # create model and load weights
    model = algo_factory(
        algo_name,
        config,
        obs_key_shapes=shape_meta["all_shapes"],
        ac_dim=shape_meta["ac_dim"],
        device=device,
    )
    print(model)
    model.deserialize(ckpt_dict["model"])
    model.set_eval()
    model = RolloutPolicy(model, obs_normalization_stats=obs_normalization_stats)
    if verbose:
        print("============= Loaded Policy =============")
        print(model)
    return model, ckpt_dict


class BC_Transformer_HYBRID(BC_Transformer):
    """
    BC training with an Transformer GMM policy.
    """
    def _create_networks(self):
        """
        Creates networks and places them into @self.nets.
        """
        assert self.algo_config.gmm.enabled
        assert self.algo_config.transformer.enabled
        assert len(self.algo_config.gmm.discrete_indices) != 0

        self.discrete_indices = self.algo_config.gmm.discrete_indices
        self.discrete_classes = self.algo_config.gmm.discrete_classes

        self.trajectory_indices = [idx for idx in range(self.ac_dim)
                                    if idx not in self.discrete_indices]

        print("Discrete class: ", self.discrete_classes)
        print("Discrete indices: ", self.discrete_indices)
        print("Trajectory indices: ", self.trajectory_indices)

        self.discrete_dim = len(self.discrete_indices)
        self.trajctory_dim = len(self.trajectory_indices)
        self.ac_dim = self.discrete_dim + self.trajctory_dim

        self.nets = nn.ModuleDict()
        self.nets["policy"] = TransformerGMMHybridActorNetwork(
            obs_shapes=self.obs_shapes,
            goal_shapes=self.goal_shapes,
            trajectory_indices=self.trajectory_indices,
            discrete_classes=self.discrete_classes,
            discrete_indices=self.discrete_indices,
            ac_dim=self.ac_dim,
            num_modes=self.algo_config.gmm.num_modes,
            min_std=self.algo_config.gmm.min_std,
            std_activation=self.algo_config.gmm.std_activation,
            low_noise_eval=self.algo_config.gmm.low_noise_eval,
            encoder_kwargs=ObsUtils.obs_encoder_kwargs_from_config(self.obs_config.encoder),
            **BaseNets.transformer_args_from_config(self.algo_config.transformer),
        )
        self._set_params_from_config()
        self.nets = self.nets.float().to(self.device)


    # def process_batch_for_training(self, batch):
    #     """
    #     Processes input batch from a data loader to filter out
    #     relevant information and prepare the batch for training.
    #     Args:
    #         batch (dict): dictionary with torch.Tensors sampled
    #             from a data loader
    #     Returns:
    #         input_batch (dict): processed and filtered batch that
    #             will be used for training
    #     """
    #     input_batch = dict()
    #     h = self.context_length
    #     input_batch["obs"] = {k: batch["obs"][k][:, :h, :] for k in batch["obs"]}
    #     input_batch["goal_obs"] = batch.get("goal_obs", None) # goals may not be present

    #     if self.supervise_all_steps:
    #         # supervision on entire sequence (instead of just current timestep)
    #         input_batch["actions"] = batch["actions"][:, :h, :]
    #     else:
    #         # just use current timestep
    #         input_batch["actions"] = batch["actions"][:, h-1, :]

    #     input_batch = TensorUtils.to_device(TensorUtils.to_float(input_batch), self.device)
    #     return input_batch


    def _forward_training(self, batch, epoch=None):
        """
        Modify from super class to support GMM training.
        """
        # ensure that transformer context length is consistent with temporal dimension of observations
        TensorUtils.assert_size_at_dim(
            batch["obs"],
            size=(self.context_length),
            dim=1,
            msg="Error: expect temporal dimension of obs batch to match transformer context length {}".format(self.context_length),
        )

        pred_dists, pred_discrete = self.nets["policy"].forward_train(
            obs_dict=batch["obs"],
            goal_dict=batch["goal_obs"],
        )

        demo_trajectory = batch["actions"][..., self.trajectory_indices]
        demo_discrete = torch.round(batch["actions"][..., self.discrete_indices]).type(torch.long)

        # make sure that this is a batch of multivariate action distributions, so that
        # the log probability computation will be correct
        assert len(pred_dists.batch_shape) == 2 # [B, T]

        if not self.supervise_all_steps:
            # only use final timestep prediction by making a new distribution with only final timestep.
            # This essentially does `dists = dists[:, -1]`
            component_distribution = D.Normal(
                loc=pred_dists.component_distribution.base_dist.loc[:, -1],
                scale=pred_dists.component_distribution.base_dist.scale[:, -1],
            )
            component_distribution = D.Independent(component_distribution, 1)
            mixture_distribution = D.Categorical(logits=pred_dists.mixture_distribution.logits[:, -1])
            pred_dists = D.MixtureSameFamily(
                mixture_distribution=mixture_distribution,
                component_distribution=component_distribution,
            )

        log_probs = pred_dists.log_prob(demo_trajectory)

        if self.supervise_all_steps:
            cross_entropy = nn.CrossEntropyLoss()(pred_discrete.flatten(end_dim=1), demo_discrete.flatten(end_dim=1))
        else:
            # spawn the current timestep to match the batch size
            cross_entropy = nn.CrossEntropyLoss()(pred_discrete.flatten(end_dim=1),
                                               demo_discrete.unsqueeze(1).expand(pred_discrete.size(0), pred_discrete.size(1), demo_discrete.size(1)).flatten(end_dim=1))
        invariant_loss = torch.zeros_like(cross_entropy)

        if "position_diffs" in batch["obs"].keys() and "quaternions" in batch["obs"].keys():

            position_diffs = batch["obs"]["position_diffs"]
            position_diffs = position_diffs.view(*position_diffs.shape[:-1], -1, 3)
            rotation_diffs = batch["obs"]["rotation_diffs"]
            rotation_diffs = rotation_diffs.view(*rotation_diffs.shape[:-1], -1, 4)

            trajectory_sample = pred_dists.sample()
            pred_position_diff = trajectory_sample[..., :3]
            pred_rotation_diff = trajectory_sample[..., 3:]

            demo_position_diff = demo_trajectory[..., :3]
            demo_rotation_diff = demo_trajectory[..., 3:]

            if demo_rotation_diff.shape[-1] == 4:
                mode = "quat"
            else:
                mode = "rpy"

            pred_linear_invar, pred_angular_invar = dhb_loss.compute_DHB_recur(
                pred_position_diff, pred_rotation_diff, position_diffs, rotation_diffs, mode
            )

            demo_linear_invar, demo_angular_invar = dhb_loss.compute_DHB_recur(
                demo_position_diff, demo_rotation_diff, position_diffs, rotation_diffs, mode
            )

            invariant_loss = nn.MSELoss()(pred_linear_invar, demo_linear_invar) +\
                            nn.MSELoss()(pred_angular_invar, demo_angular_invar)
        else:
            invariant_loss = torch.zeros_like(cross_entropy)

        predictions = OrderedDict(
            log_probs=log_probs,
            cross_entropy=cross_entropy,
            invariant_loss=invariant_loss,
        )
        return predictions


    def _compute_losses(self, predictions, batch):
        """
        Internal helper function for BC algo class. Compute losses based on
        network outputs in @predictions dict, using reference labels in @batch.

        Args:
            predictions (dict): dictionary containing network outputs, from @_forward_training
            batch (dict): dictionary with torch.Tensors sampled
                from a data loader and filtered by @process_batch_for_training

        Returns:
            losses (dict): dictionary of losses computed over the batch
        """

        # loss is just negative log-likelihood of action targets
        log_probs = predictions["log_probs"].mean()
        cross_entropy = predictions["cross_entropy"]
        invariant_loss = predictions["invariant_loss"]
        action_loss = - self.algo_config.loss.log_probs_weight*log_probs\
                      + self.algo_config.loss.cross_entropy_weight*cross_entropy\
                      + self.algo_config.loss.invar_loss_weight*invariant_loss
        return OrderedDict(
            cross_etropy=cross_entropy,
            log_probs=log_probs,
            invariant_loss=invariant_loss,
            action_loss=action_loss,
        )


    def log_info(self, info):
        """
        Process info dictionary from @train_on_batch to summarize
        information to pass to tensorboard for logging.

        Args:
            info (dict): dictionary of info

        Returns:
            loss_log (dict): name -> summary statistic
        """
        log = PolicyAlgo.log_info(self, info)
        log["Loss"] = info["losses"]["action_loss"].item()
        log["Invariant Loss"] = info["losses"]["invariant_loss"].item()
        log["Log_Likelihood"] = info["losses"]["log_probs"].item()
        log["Cross_Entropy"] = info["losses"]["cross_etropy"].item()
        if "policy_grad_norms" in info:
            log["Policy_Grad_Norms"] = info["policy_grad_norms"]
        return log



class BC_RNN_HYBRID(BC_RNN):
    """
    BC training with an RNN GMM policy.
    """
    def _create_networks(self):
        """
        Creates networks and places them into @self.nets.
        """
        assert self.algo_config.gmm.enabled
        assert self.algo_config.rnn.enabled
        assert len(self.algo_config.gmm.discrete_indices) != 0

        self.discrete_indices = self.algo_config.gmm.discrete_indices
        self.discrete_classes = self.algo_config.gmm.discrete_classes

        self.trajectory_indices = [idx for idx in range(self.ac_dim)
                                    if idx not in self.discrete_indices]

        print("Discrete class: ", self.discrete_classes)
        print("Discrete indices: ", self.discrete_indices)
        print("Trajectory indices: ", self.trajectory_indices)

        self.discrete_dim = len(self.discrete_indices)
        self.trajctory_dim = len(self.trajectory_indices)
        self.ac_dim = self.discrete_dim + self.trajctory_dim

        self.nets = nn.ModuleDict()
        self.nets["policy"] = RNNHybridActorNetwork(
            obs_shapes=self.obs_shapes,
            goal_shapes=self.goal_shapes,
            trajectory_indices=self.trajectory_indices,
            discrete_classes=self.discrete_classes,
            discrete_indices=self.discrete_indices,
            ac_dim=self.ac_dim,
            mlp_layer_dims=self.algo_config.actor_layer_dims,
            num_modes=self.algo_config.gmm.num_modes,
            min_std=self.algo_config.gmm.min_std,
            std_activation=self.algo_config.gmm.std_activation,
            low_noise_eval=self.algo_config.gmm.low_noise_eval,
            encoder_kwargs=ObsUtils.obs_encoder_kwargs_from_config(self.obs_config.encoder),
            **BaseNets.rnn_args_from_config(self.algo_config.rnn),
        )

        self._rnn_hidden_state = None
        self._rnn_horizon = self.algo_config.rnn.horizon
        self._rnn_counter = 0
        self._rnn_is_open_loop = self.algo_config.rnn.get("open_loop", False)

        self.nets = self.nets.float().to(self.device)

    def _forward_training(self, batch):
        """
        Internal helper function for BC algo class. Compute forward pass
        and return network outputs in @predictions dict.

        Args:
            batch (dict): dictionary with torch.Tensors sampled
                from a data loader and filtered by @process_batch_for_training

        Returns:
            predictions (dict): dictionary containing network outputs
        """
        pred_dists, pred_discrete = self.nets["policy"].forward_train(
            obs_dict=batch["obs"],
            goal_dict=batch["goal_obs"],
        )

        demo_trajectory = batch["actions"][..., self.trajectory_indices]
        demo_discrete = torch.round(batch["actions"][..., self.discrete_indices]).type(torch.long)

        # original_stdout = sys.stdout # Save a reference to the original standard output
        # with open('/home/mingyo/test.txt', 'a') as f:
        #    sys.stdout = f # Change the standard output to the file we created.
        #    print(batch["actions"].shape, demo_trajectory.shape, demo_discrete.shape)
        #    sys.stdout = original_stdout # Reset the standard output to its original value

        # make sure that this is a batch of multivariate action distributions, so that
        # the log probability computation will be correct
        assert len(pred_dists.batch_shape) == 2 # [B, T]
        log_probs = pred_dists.log_prob(demo_trajectory)

        # original_stdout = sys.stdout # Save a reference to the original standard output
        # with open('/home/mingyo/test.txt', 'a') as f:
        #    sys.stdout = f # Change the standard output to the file we created.
        #    print(pred_discrete.shape, demo_discrete.shape)
        #    sys.stdout = original_stdout # Reset the standard output to its original value

        cross_entropy = nn.CrossEntropyLoss()(pred_discrete.flatten(end_dim=1), demo_discrete.flatten(end_dim=1))

        if "delta_positions" in batch["obs"].keys() and "delta_eulers" in batch["obs"].keys():

            delta_positions = batch["obs"]["delta_positions"]
            delta_positions = delta_positions.view(*delta_positions.shape[:-1], -1, 3)
            delta_rotations = batch["obs"]["delta_eulers"]
            delta_rotations = delta_rotations.view(*delta_rotations.shape[:-1], -1, 3)

            trajectory_sample = pred_dists.sample()
            pred_position_diff = trajectory_sample[..., :3]
            pred_rotation_diff = trajectory_sample[..., 3:]

            demo_position_diff = demo_trajectory[..., :3]
            demo_rotation_diff = demo_trajectory[..., 3:]

            if delta_rotations.shape[-1] == 4:
                mode = "quat"
            else:
                mode = "rpy"

            pred_linear_invar, pred_angular_invar = dhb_loss.compute_DHB_recur(
                pred_position_diff, pred_rotation_diff, delta_positions, delta_rotations, mode
            )

            demo_linear_invar, demo_angular_invar = dhb_loss.compute_DHB_recur(
                demo_position_diff, demo_rotation_diff, delta_positions, delta_rotations, mode
            )

            invariant_loss = nn.MSELoss()(pred_linear_invar, demo_linear_invar) +\
                            nn.MSELoss()(pred_angular_invar, demo_angular_invar)
        else:
            invariant_loss = torch.zeros_like(cross_entropy)

        predictions = OrderedDict(
            log_probs=log_probs,
            cross_entropy=cross_entropy,
            invariant_loss=invariant_loss,
        )
        return predictions

    def _compute_losses(self, predictions, batch):
        """
        Internal helper function for BC algo class. Compute losses based on
        network outputs in @predictions dict, using reference labels in @batch.

        Args:
            predictions (dict): dictionary containing network outputs, from @_forward_training
            batch (dict): dictionary with torch.Tensors sampled
                from a data loader and filtered by @process_batch_for_training

        Returns:
            losses (dict): dictionary of losses computed over the batch
        """

        # loss is just negative log-likelihood of action targets
        log_probs = predictions["log_probs"].mean()
        cross_entropy = predictions["cross_entropy"]
        invariant_loss = predictions["invariant_loss"]
        action_loss = - self.algo_config.loss.log_probs_weight*log_probs\
                      + self.algo_config.loss.cross_entropy_weight*cross_entropy\
                      + self.algo_config.loss.invar_loss_weight*invariant_loss
        return OrderedDict(
            cross_etropy=cross_entropy,
            log_probs=log_probs,
            invariant_loss=invariant_loss,
            action_loss=action_loss,
        )

    def log_info(self, info):
        """
        Process info dictionary from @train_on_batch to summarize
        information to pass to tensorboard for logging.

        Args:
            info (dict): dictionary of info

        Returns:
            loss_log (dict): name -> summary statistic
        """
        log = PolicyAlgo.log_info(self, info)
        log["Loss"] = info["losses"]["action_loss"].item()
        log["Invariant Loss"] = info["losses"]["invariant_loss"].item()
        log["Log_Likelihood"] = info["losses"]["log_probs"].item()
        log["Cross_Entropy"] = info["losses"]["cross_etropy"].item()
        if "policy_grad_norms" in info:
            log["Policy_Grad_Norms"] = info["policy_grad_norms"]
        return log
