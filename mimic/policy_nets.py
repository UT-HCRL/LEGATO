import numpy as np
from collections import OrderedDict

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.distributions as D

import robomimic.utils.tensor_utils as TensorUtils
from robomimic.models.obs_nets import MIMO_MLP, RNN_MIMO_MLP, MIMO_Transformer
from robomimic.models.distributions import TanhWrappedDistribution
from robomimic.models.policy_nets import RNNActorNetwork, TransformerActorNetwork

class RNNHybridActorNetwork(RNNActorNetwork):
    """
    An RNN GMM policy network that predicts sequences of action distributions from observation sequences.
    """
    def __init__(
        self,
        obs_shapes,
        trajectory_indices,
        discrete_classes,
        discrete_indices,
        ac_dim,
        mlp_layer_dims,
        rnn_hidden_dim,
        rnn_num_layers,
        rnn_type="LSTM",  # [LSTM, GRU]
        rnn_kwargs=None,
        num_modes=5,
        min_std=0.01,
        std_activation="softplus",
        low_noise_eval=True,
        use_tanh=False,
        goal_shapes=None,
        encoder_kwargs=None,
    ):
        """
        Args:

            rnn_hidden_dim (int): RNN hidden dimension

            rnn_num_layers (int): number of RNN layers

            rnn_type (str): [LSTM, GRU]

            rnn_kwargs (dict): kwargs for the torch.nn.LSTM / GRU

            num_modes (int): number of GMM modes

            min_std (float): minimum std output from network

            std_activation (None or str): type of activation to use for std deviation. Options are:

                `'softplus'`: Softplus activation applied

                `'exp'`: Exp applied; this corresponds to network output being interpreted as log_std instead of std

            low_noise_eval (float): if True, model will sample from GMM with low std, so that
                one of the GMM modes will be sampled (approximately)

            use_tanh (bool): if True, use a tanh-Gaussian distribution

            encoder_kwargs (dict or None): If None, results in default encoder_kwargs being applied. Otherwise, should
                be nested dictionary containing relevant per-modality information for encoder networks.
                Should be of form:

                obs_modality1: dict
                    feature_dimension: int
                    core_class: str
                    core_kwargs: dict
                        ...
                        ...
                    obs_randomizer_class: str
                    obs_randomizer_kwargs: dict
                        ...
                        ...
                obs_modality2: dict
                    ...
        """

        # parameters specific to GMM actor
        self.num_modes = num_modes
        self.min_std = min_std
        self.low_noise_eval = low_noise_eval
        self.use_tanh = use_tanh

        self.discrete_classes = discrete_classes
        self.discrete_indices = discrete_indices
        self.trajectory_indices = trajectory_indices

        self.discrete_dim = len(discrete_indices)
        self.trajectory_dim = len(trajectory_indices)

        # Define activations to use
        self.activations = {
            "softplus": F.softplus,
            "exp": torch.exp,
        }
        assert std_activation in self.activations, \
            "std_activation must be one of: {}; instead got: {}".format(self.activations.keys(), std_activation)
        self.std_activation = std_activation

        super(RNNHybridActorNetwork, self).__init__(
            obs_shapes=obs_shapes,
            ac_dim=ac_dim,
            #trajectory_dim=trajectory_dim,
            #discrete_dim=discrete_dim,
            mlp_layer_dims=mlp_layer_dims,
            rnn_hidden_dim=rnn_hidden_dim,
            rnn_num_layers=rnn_num_layers,
            rnn_type=rnn_type,
            rnn_kwargs=rnn_kwargs,
            goal_shapes=goal_shapes,
            encoder_kwargs=encoder_kwargs,
        )


    def _get_output_shapes(self):
        """
        Tells @MIMO_MLP superclass about the output dictionary that should be generated
        at the last layer. Network outputs parameters of GMM distribution.
        """
        return OrderedDict(
            mean=(self.num_modes, self.trajectory_dim), 
            scale=(self.num_modes, self.trajectory_dim), 
            logits=(self.num_modes,),
            discrete=(max(self.discrete_classes), self.discrete_dim)
        )

    def forward_train(self, obs_dict, goal_dict=None, rnn_init_state=None, return_state=False):
        """
        Return full GMM distribution, which is useful for computing
        quantities necessary at train-time, like log-likelihood, KL 
        divergence, etc.

        Args:
            obs_dict (dict): batch of observations
            goal_dict (dict): if not None, batch of goal observations
            rnn_init_state: rnn hidden state, initialize to zero state if set to None
            return_state (bool): whether to return hidden state

        Returns:
            dists (Distribution): sequence of GMM distributions over the timesteps
            rnn_state: return rnn state at the end if return_state is set to True
        """
        if self._is_goal_conditioned:
            assert goal_dict is not None
            # repeat the goal observation in time to match dimension with obs_dict
            mod = list(obs_dict.keys())[0]
            goal_dict = TensorUtils.unsqueeze_expand_at(goal_dict, size=obs_dict[mod].shape[1], dim=1)

        outputs = RNN_MIMO_MLP.forward(
            self, obs=obs_dict, goal=goal_dict, rnn_init_state=rnn_init_state, return_state=return_state)

        if return_state:
            outputs, state = outputs
        else:
            state = None
        
        means = outputs["mean"]
        scales = outputs["scale"]
        logits = outputs["logits"]
        discrete = outputs["discrete"]

        # apply tanh squashing to mean if not using tanh-GMM to ensure means are in [-1, 1]
        if not self.use_tanh:
            means = torch.tanh(means)

        if self.low_noise_eval and (not self.training):
            # low-noise for all Gaussian dists
            scales = torch.ones_like(means) * 1e-4
        else:
            # post-process the scale accordingly
            scales = self.activations[self.std_activation](scales) + self.min_std

        # mixture components - make sure that `batch_shape` for the distribution is equal
        # to (batch_size, timesteps, num_modes) since MixtureSameFamily expects this shape
        component_distribution = D.Normal(loc=means, scale=scales)
        component_distribution = D.Independent(component_distribution, 1) # shift action dim to event shape

        # unnormalized logits to categorical distribution for mixing the modes
        mixture_distribution = D.Categorical(logits=logits)

        dists = D.MixtureSameFamily(
            mixture_distribution=mixture_distribution,
            component_distribution=component_distribution,
        )

        if self.use_tanh:
            # Wrap distribution with Tanh
            dists = TanhWrappedDistribution(base_dist=dists, scale=1.)

        discrete_logits = torch.softmax(discrete, dim=-2)

        if return_state:
            return (dists, discrete_logits), state
        else:
            return (dists, discrete_logits)

    def forward(self, obs_dict, goal_dict=None, rnn_init_state=None, return_state=False):
        """
        Samples actions from the policy distribution.

        Args:
            obs_dict (dict): batch of observations
            goal_dict (dict): if not None, batch of goal observations

        Returns:
            action (torch.Tensor): batch of actions from policy distribution
        """
        out = self.forward_train(obs_dict=obs_dict, goal_dict=goal_dict, rnn_init_state=rnn_init_state, return_state=return_state)
        if return_state:
            ad, state = out
            dists, discrete_logits = ad
            trajectory_sample = dists.sample()
            discrete_args = torch.argmax(discrete_logits, dim=-2)
            return (trajectory_sample, discrete_args), state
        else:
            dists, discrete_logits = out
            trajectory_sample = dists.sample()
            discrete_args = torch.argmax(discrete_logits, dim=-2)
            return (trajectory_sample, discrete_args)


    def forward_train_step(self, obs_dict, goal_dict=None, rnn_state=None):
        """
        Unroll RNN over single timestep to get action GMM distribution, which 
        is useful for computing quantities necessary at train-time, like 
        log-likelihood, KL divergence, etc.

        Args:
            obs_dict (dict): batch of observations. Should not contain
                time dimension.
            goal_dict (dict): if not None, batch of goal observations
            rnn_state: rnn hidden state, initialize to zero state if set to None

        Returns:
            ad (Distribution): GMM action distributions
            state: updated rnn state
        """
        obs_dict = TensorUtils.to_sequence(obs_dict)
        ad, state = self.forward_train(
            obs_dict, goal_dict, rnn_init_state=rnn_state, return_state=True)

        dists, discrete_logits = ad

        # to squeeze time dimension, make another action distribution
        assert dists.component_distribution.base_dist.loc.shape[1] == 1
        assert dists.component_distribution.base_dist.scale.shape[1] == 1
        assert dists.mixture_distribution.logits.shape[1] == 1
        component_distribution = D.Normal(
            loc=dists.component_distribution.base_dist.loc.squeeze(1),
            scale=dists.component_distribution.base_dist.scale.squeeze(1),
        )
        component_distribution = D.Independent(component_distribution, 1)
        mixture_distribution = D.Categorical(logits=dists.mixture_distribution.logits.squeeze(1))
        dists = D.MixtureSameFamily(
            mixture_distribution=mixture_distribution,
            component_distribution=component_distribution,
        )

        return (dists, discrete_logits), state

    def forward_step(self, obs_dict, goal_dict=None, rnn_state=None):
        """
        Unroll RNN over single timestep to get sampled actions.

        Args:
            obs_dict (dict): batch of observations. Should not contain
                time dimension.
            goal_dict (dict): if not None, batch of goal observations
            rnn_state: rnn hidden state, initialize to zero state if set to None

        Returns:
            acts (torch.Tensor): batch of actions - does not contain time dimension
            state: updated rnn state
        """
        obs_dict = TensorUtils.to_sequence(obs_dict)
        acts, state = self.forward(
            obs_dict, goal_dict, rnn_init_state=rnn_state, return_state=True)
        dist_act, discrete_act = acts
        assert dist_act.shape[1] == 1
        out = torch.zeros((discrete_act.shape[0], self.discrete_dim + self.trajectory_dim), device=discrete_act.device)
        # out[:, self.discrete_indices] = discrete_act[:, 0]
        out[:, self.discrete_indices] = discrete_act[:, 0].type(torch.float32)
        out[:, self.trajectory_indices] = dist_act[:, 0]
        return out, state

    def _to_string(self):
        """Info to pretty print."""
        msg = "trajectory_dim={}, discrete_dim={}, std_activation={}, low_noise_eval={}, num_nodes={}, min_std={}".format(
            self.trajectory_dim, self.discrete_dim, self.std_activation, self.low_noise_eval, self.num_modes, self.min_std)
        return msg


class TransformerGMMHybridActorNetwork(TransformerActorNetwork):
    """
    A Transformer GMM policy network that predicts sequences of action distributions from observation 
    sequences (assumed to be frame stacked from previous observations).
    """
    def __init__(
        self,
        obs_shapes,
        trajectory_indices,
        discrete_classes,
        discrete_indices,
        ac_dim,
        transformer_embed_dim,
        transformer_num_layers,
        transformer_num_heads,
        transformer_context_length,
        transformer_emb_dropout=0.1,
        transformer_attn_dropout=0.1,
        transformer_block_output_dropout=0.1,
        transformer_sinusoidal_embedding=False,
        transformer_activation="gelu",
        transformer_nn_parameter_for_timesteps=False,
        num_modes=5,
        min_std=0.01,
        std_activation="softplus",
        low_noise_eval=True,
        use_tanh=False,
        goal_shapes=None,
        encoder_kwargs=None,
    ):
        """
        Args:

            obs_shapes (OrderedDict): a dictionary that maps modality to
                expected shapes for observations.
            
            ac_dim (int): dimension of action space.

            transformer_embed_dim (int): dimension for embeddings used by transformer

            transformer_num_layers (int): number of transformer blocks to stack

            transformer_num_heads (int): number of attention heads for each
                transformer block - must divide @transformer_embed_dim evenly. Self-attention is 
                computed over this many partitions of the embedding dimension separately.
            
            transformer_context_length (int): expected length of input sequences

            transformer_embedding_dropout (float): dropout probability for embedding inputs in transformer

            transformer_attn_dropout (float): dropout probability for attention outputs for each transformer block

            transformer_block_output_dropout (float): dropout probability for final outputs for each transformer block

            num_modes (int): number of GMM modes

            min_std (float): minimum std output from network

            std_activation (None or str): type of activation to use for std deviation. Options are:

                `'softplus'`: Softplus activation applied

                `'exp'`: Exp applied; this corresponds to network output being interpreted as log_std instead of std

            low_noise_eval (float): if True, model will sample from GMM with low std, so that
                one of the GMM modes will be sampled (approximately)

            use_tanh (bool): if True, use a tanh-Gaussian distribution

            encoder_kwargs (dict or None): If None, results in default encoder_kwargs being applied. Otherwise, should
                be nested dictionary containing relevant per-modality information for encoder networks.
                Should be of form:

                obs_modality1: dict
                    feature_dimension: int
                    core_class: str
                    core_kwargs: dict
                        ...
                        ...
                    obs_randomizer_class: str
                    obs_randomizer_kwargs: dict
                        ...
                        ...
                obs_modality2: dict
                    ...
        """
        
        # parameters specific to GMM actor
        self.num_modes = num_modes
        self.min_std = min_std
        self.low_noise_eval = low_noise_eval
        self.use_tanh = use_tanh

        self.discrete_classes = discrete_classes
        self.discrete_indices = discrete_indices
        self.trajectory_indices = trajectory_indices

        self.discrete_dim = len(discrete_indices)
        self.trajectory_dim = len(trajectory_indices)

        # Define activations to use
        self.activations = {
            "softplus": F.softplus,
            "exp": torch.exp,
        }
        assert std_activation in self.activations, \
            "std_activation must be one of: {}; instead got: {}".format(self.activations.keys(), std_activation)
        self.std_activation = std_activation

        super(TransformerGMMHybridActorNetwork, self).__init__(
            obs_shapes=obs_shapes,
            ac_dim=ac_dim,
            transformer_embed_dim=transformer_embed_dim,
            transformer_num_layers=transformer_num_layers,
            transformer_num_heads=transformer_num_heads,
            transformer_context_length=transformer_context_length,
            transformer_emb_dropout=transformer_emb_dropout,
            transformer_attn_dropout=transformer_attn_dropout,
            transformer_block_output_dropout=transformer_block_output_dropout,
            transformer_sinusoidal_embedding=transformer_sinusoidal_embedding,
            transformer_activation=transformer_activation,
            transformer_nn_parameter_for_timesteps=transformer_nn_parameter_for_timesteps,            
            encoder_kwargs=encoder_kwargs,
            goal_shapes=goal_shapes,
        )

    def _get_output_shapes(self):
        """
        Tells @MIMO_Transformer superclass about the output dictionary that should be generated
        at the last layer. Network outputs parameters of GMM distribution.
        """
        return OrderedDict(
            mean=(self.num_modes, self.trajectory_dim), 
            scale=(self.num_modes, self.trajectory_dim), 
            logits=(self.num_modes,),
            discrete=(max(self.discrete_classes), self.discrete_dim)
        )

    def forward_train(self, obs_dict, actions=None, goal_dict=None, low_noise_eval=None):
        """
        Return full GMM distribution, which is useful for computing
        quantities necessary at train-time, like log-likelihood, KL 
        divergence, etc.
        Args:
            obs_dict (dict): batch of observations
            actions (torch.Tensor): batch of actions
            goal_dict (dict): if not None, batch of goal observations
        Returns:
            dists (Distribution): sequence of GMM distributions over the timesteps
        """
        if self._is_goal_conditioned:
            assert goal_dict is not None
            # repeat the goal observation in time to match dimension with obs_dict
            mod = list(obs_dict.keys())[0]
            goal_dict = TensorUtils.unsqueeze_expand_at(goal_dict, size=obs_dict[mod].shape[1], dim=1)

        forward_kwargs = dict(obs=obs_dict, goal=goal_dict)

        outputs = MIMO_Transformer.forward(self, **forward_kwargs)
        
        means = outputs["mean"]
        scales = outputs["scale"]
        logits = outputs["logits"]
        discrete = outputs["discrete"]

        # apply tanh squashing to mean if not using tanh-GMM to ensure means are in [-1, 1]
        if not self.use_tanh:
            means = torch.tanh(means)

        if low_noise_eval is None:
            low_noise_eval = self.low_noise_eval
        if low_noise_eval and (not self.training):
            # low-noise for all Gaussian dists
            scales = torch.ones_like(means) * 1e-4
        else:
            # post-process the scale accordingly
            scales = self.activations[self.std_activation](scales) + self.min_std


        # mixture components - make sure that `batch_shape` for the distribution is equal
        # to (batch_size, timesteps, num_modes) since MixtureSameFamily expects this shape
        component_distribution = D.Normal(loc=means, scale=scales)
        component_distribution = D.Independent(component_distribution, 1) # shift action dim to event shape

        # unnormalized logits to categorical distribution for mixing the modes
        mixture_distribution = D.Categorical(logits=logits)

        dists = D.MixtureSameFamily(
            mixture_distribution=mixture_distribution,
            component_distribution=component_distribution,
        )

        if self.use_tanh:
            # Wrap distribution with Tanh
            dists = TanhWrappedDistribution(base_dist=dists, scale=1.)

        discrete_logits = torch.softmax(discrete, dim=-2)

        return (dists, discrete_logits)


    def forward(self, obs_dict, actions=None, goal_dict=None):
        """
        Samples actions from the policy distribution.

        Args:
            obs_dict (dict): batch of observations
            goal_dict (dict): if not None, batch of goal observations

        Returns:
            action (torch.Tensor): batch of actions from policy distribution
        """
        out = self.forward_train(obs_dict=obs_dict, goal_dict=goal_dict)
        dists, discrete_logits = out
        trajectory_sample = dists.sample()
        discrete_args = torch.argmax(discrete_logits, dim=-2)
        assert trajectory_sample.shape[1] == 1
        out = torch.zeros((discrete_args.shape[0], self.discrete_dim + self.trajectory_dim), device=discrete_args.device)
        # out[:, self.discrete_indices] = discrete_act[:, 0]
        out[:, self.discrete_indices] = discrete_args[:, 0].type(torch.float32)
        out[:, self.trajectory_indices] = trajectory_sample[:, 0]
        return out


    def _to_string(self):
        """Info to pretty print."""
        msg = "trajectory_dim={}, discrete_dim={}, std_activation={}, low_noise_eval={}, num_nodes={}, min_std={}".format(
            self.trajectory_dim, self.discrete_dim, self.std_activation, self.low_noise_eval, self.num_modes, self.min_std)
        return msg
