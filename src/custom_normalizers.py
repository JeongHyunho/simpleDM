import warnings

import tensorflow as tf
import numpy as np

from stable_baselines.common import tf_util

UPDATE_STEP = 10000
UPDATE_FRAC = 0.2


class TFNormalizer:
    """
    Tensor normalizer with mean and std
    :param sess: (Tensorflow session)
    :param scope: (str)
    :param size: (int)
    :param reuse: (boolean)
    :param eps: (float)
    :param clip: (float or inf)
    :param init_mean: ([float])
    :param init_std: ([float])
    """

    def __init__(self, sess, scope, size, reuse, eps=0.02, clip=np.inf,
                 init_mean=None, init_std=None):
        self.sess = sess
        self.scope = scope
        self.reuse = reuse
        self.eps = eps
        self.clip = clip
        self.count = 0
        self.mean = np.zeros(size)
        self.sq = np.zeros(size)
        self.std = np.ones(size)

        self.mean_stack = np.zeros([UPDATE_STEP, size])
        self.sq_stack = np.zeros([UPDATE_STEP, size])

        if init_mean is None:
            warnings.warn('No initial mean. You use array of zeros')
            init_mean = np.zeros(size)
        if init_std is None:
            warnings.warn('No initial std. You use array of ones')
            init_std = np.ones(size)
        self._set_mean_std(init_mean, init_std)

        self._build_resource()

        if not self.reuse:
            self._update_resource()

    def _set_mean_std(self, mean, std):
        self.mean = mean
        self.sq = np.square(mean) + np.square(std)
        self.std = std

    def _build_resource(self):
        with tf.variable_scope(self.scope, reuse=tf.AUTO_REUSE):
            self.mean_tf = tf.get_variable(dtype=tf.float32,
                                           name='mean',
                                           initializer=tf.constant(self.mean, dtype=tf.float32),
                                           trainable=False)
            self.std_tf = tf.get_variable(dtype=tf.float32,
                                          name='std',
                                          initializer=tf.constant(self.std, dtype=tf.float32),
                                          trainable=False)
            if not self.reuse:  # if action mode
                self.mean_ph = tf.placeholder(dtype=tf.float32,
                                              name='new_mean',
                                              shape=self.mean.shape)
                self.std_ph = tf.placeholder(dtype=tf.float32,
                                             name='new_std',
                                             shape=self.std.shape)
                self.update_op = tf.group(self.mean_tf.assign(self.mean_ph, name='assign_mean'),
                                          self.std_tf.assign(self.std_ph, name='assign_std'),
                                          name='norm_params_assign')

    def _update_resource(self):
        feed = {self.mean_ph: self.mean,
                self.std_ph: self.std}
        self.sess.run(self.update_op, feed_dict=feed)

    def update(self, new_sample):
        self.mean_stack[self.count % UPDATE_STEP] = new_sample
        self.sq_stack[self.count % UPDATE_STEP] = new_sample ** 2

        if self.count % UPDATE_STEP == UPDATE_STEP - 1:
            w_old = (self.count + 1 - UPDATE_STEP) / (self.count + 1)
            w_new = UPDATE_STEP / (self.count + 1)
            self.mean = w_old * self.mean + w_new * np.mean(self.mean_stack, axis=0)
            self.sq = w_old * self.sq + w_new * np.mean(self.sq_stack, axis=0)
            # self.mean = self.mean + UPDATE_FRAC * (np.mean(self.mean_stack, axis=0) - self.mean)
            # self.sq = self.sq + UPDATE_FRAC * (np.mean(self.sq_stack, axis=0) - self.sq)
            std = np.maximum(np.sqrt(self.sq - np.square(self.mean)), 0)
            std = np.nan_to_num(std)
            self.std = np.maximum(std, self.eps)

            self._update_resource()

        self.count += 1

    def clip_normalize(self, in_tf):
        norm_in = (in_tf - self.mean_tf) / self.std_tf
        clipped_out = tf.clip_by_value(norm_in, -self.clip, self.clip)
        return clipped_out

    def un_normalize(self, in_tf):
        out_tf = self.std_tf * in_tf + self.mean_tf
        return out_tf
