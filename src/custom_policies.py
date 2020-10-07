import tensorflow as tf
import numpy as np
from stable_baselines.common.distributions import ProbabilityDistributionType, DiagGaussianProbabilityDistribution

from stable_baselines.common.policies import ActorCriticPolicy, mlp_extractor
from stable_baselines.common.tf_layers import linear

from src.custom_normalizers import TFNormalizer


class NormalMlpPolicy(ActorCriticPolicy):
    """
    Custom policy object that implements input normalization and clipping, using a feed forward neural network.

    :param sess: (TensorFlow session) The current TensorFlow session
    :param ob_space: (Gym Space) The observation space of the environment
    :param ac_space: (Gym Space) The action space of the environment
    :param n_env: (int) The number of environments to run
    :param n_steps: (int) The number of steps to run for each environment
    :param n_batch: (int) The number of batch to run (n_envs * n_steps)
    :param act_norm_init: (bool)
    :param obs_norm_init: (dict)
    :param net_arch: (dict)
    :param reuse: (bool) If the policy is reusable or not
    :param act_fun: (tf.func) the activation function to use in the neural network.
    """

    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch, act_norm_init=None, obs_norm_init=None,
                 net_arch=None, reuse=False, act_fun=tf.tanh):
        super(NormalMlpPolicy, self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse)

        if obs_norm_init is not None:
            self.obs_norm = TFNormalizer(sess, 'obs_norm', ob_space.shape[0], reuse=reuse, **obs_norm_init)
        else:
            self.obs_norm = None

        if act_norm_init is not None:
            self.act_norm = TFNormalizer(sess, 'act_norm', ac_space.shape[0], reuse=reuse,  **act_norm_init)
        else:
            self.act_norm = None

        del self._pdtype
        self._pdtype = ActNormGaussProbDistType(ac_space.shape[0], self.act_norm)

        if net_arch is None:
            net_arch = [dict(vf=[64, 64], pi=[64, 64])]

        with tf.variable_scope("model", reuse=reuse):
            # normalization and clipping
            if self.obs_norm is not None:
                extractor_in = self.obs_norm.clip_normalize(tf.layers.flatten(self.processed_obs))
            else:
                extractor_in = tf.layers.flatten(self.processed_obs)

            pi_latent, vf_latent = mlp_extractor(extractor_in, net_arch, act_fun)

            self._value_fn = linear(vf_latent, 'vf', 1)

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent, init_scale=0.01)

        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp= self.sess.run([self.deterministic_action, self.value_flat, self.neglogp],
                                                   {self.obs_ph: obs})
        else:
            action, value, neglogp, mean_orig = \
                self.sess.run([self.action, self.value_flat, self.neglogp, self._policy], {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp, mean_orig

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})


class ActNormGaussProbDistType(ProbabilityDistributionType):
    def __init__(self, size: int, act_norm: TFNormalizer):
        """
        The probability distribution type for multivariate Gaussian input
        The mean action is normalized

        :param size: (int) the number of dimensions of the multivariate gaussian
        :param act_norm: (TFNormalizer)
        """
        self.size = size
        self.act_norm = act_norm

    def probability_distribution_class(self):
        return DiagGaussianProbabilityDistribution

    def proba_distribution_from_flat(self, flat):
        """
        returns the probability distribution from flat probabilities

        :param flat: ([float]) the flat probabilities
        :return: (ProbabilityDistribution) the instance of the ProbabilityDistribution associated
        """
        return self.probability_distribution_class()(flat)

    def proba_distribution_from_latent(self, pi_latent_vector, vf_latent_vector, init_scale=1.0, init_bias=0.0):
        mean = linear(pi_latent_vector, 'pi', self.size, init_scale=init_scale, init_bias=init_bias)
        if self.act_norm is None:
            norm_mean = mean
        else:
            # norm_mean = self.act_norm.un_normalize(mean)
            norm_mean = self.act_norm.clip_normalize(mean)
        logstd = tf.get_variable(name='pi/logstd', shape=[1, self.size], initializer=tf.zeros_initializer())
        pdparam = tf.concat([norm_mean, mean * 0.0 + logstd], axis=1)
        q_values = linear(vf_latent_vector, 'q', self.size, init_scale=init_scale, init_bias=init_bias)
        return self.proba_distribution_from_flat(pdparam), mean, q_values

    def param_shape(self):
        return [2 * self.size]

    def sample_shape(self):
        return [self.size]

    def sample_dtype(self):
        return tf.float32



