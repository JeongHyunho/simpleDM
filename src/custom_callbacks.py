import os

import cv2
import numpy as np

from stable_baselines.common.callbacks import BaseCallback


class SaveGifCallback(BaseCallback):
    """
    Callback for saving a model's gif every `save_freq` steps

    :param save_freq: (int)
    :param save_path: (str) Path to the folder where the model will be saved.
    :param fps: (int) frame per second
    :param verbose: (boolean)
    """
    def __init__(self, save_freq: int, save_path: str, fps: int, verbose=False):
        super(SaveGifCallback, self).__init__(verbose)
        self.save_freq = save_freq
        self.save_path = save_path
        self.fps = fps
        self.fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        self.writer = None
        self.write_flag = False
        self.c_count = 0

    def _init_callback(self) -> None:
        self.n_sample = 5 * self.fps
        self.img_size = self.model.env.render(mode='rgb_array').shape[1::-1]
        assert (self.save_freq / self.model.n_envs) > self.n_sample, \
            "Try to save video too frequently. Increase save_freq. "
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % int(self.save_freq / self.model.n_envs) == 0:
            path = os.path.join(self.save_path, '{}_steps'.format(self.num_timesteps))
            self.writer = cv2.VideoWriter(path + '.avi',
                                          self.fourcc,
                                          self.fps,
                                          self.img_size)
            self.write_flag = True
            self.c_count = 0
            if self.verbose:
                print("Saving video of model to {}".format(path))

        if self.write_flag:
            rendered = self.model.env.render(mode='rgb_array')
            cv2.putText(rendered,
                        '#step: ' + str(self.num_timesteps),
                        (10, self.img_size[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 0), 2, cv2.LINE_AA)
            self.writer.write(rendered)
            if self.n_sample == self.c_count:
                self.writer.release()
                self.write_flag = False
            else:
                self.c_count += 1
        return True


class DiscRwdTerminate(BaseCallback):
    """
    Terminate training based on the number of discounted rewards over threshold

    :param th_perc: (float)
    :param n_skip: (int) Number of times of skipping
    :param verbose: (bool)
    """
    def __init__(self, th_perc: float, n_skip: int, verbose: bool = False):
        super(DiscRwdTerminate, self).__init__(verbose)
        self.th_perc = th_perc
        self.n_skip = n_skip
        self.n_occur = 0

    def _init_callback(self) -> None:
        self.n_steps = self.model.n_steps
        self.n_envs = self.model.n_envs
        self.disc_rwds = np.zeros((self.n_steps - 1, self.n_envs))
        self.gamma = self.model.gamma
        assert self.gamma < 1, "only gamma smaller than one can be used"
        self.max_disc_reward = 1. / (1. - self.gamma)

    def _on_step(self) -> bool:
        if self.n_skip <= self.n_occur:
            return False
        else:
            return True

    def _on_rollout_end(self) -> None:
        rwds = self.locals['mb_rewards']
        if not rwds:
            # todo: add termination message to logger
            return
        dones = self.locals['mb_dones']

        for step in reversed(range(self.n_steps - 1)):
            next_non_term = 1.0 - dones[step + 1]
            self.disc_rwds[step, :] = rwds[step] + self.gamma * next_non_term * self.disc_rwds[step, :]

        self.n_occur += (np.max(self.disc_rwds) > self.max_disc_reward * self.th_perc)


class SaveRewardPortionsCallback(BaseCallback):
    """
    Callback for saving a model's gif every `save_freq` steps

    :param fullfilename: (str)
    :param verbose: (boolean)
    """
    def __init__(self, fullfilename: str, verbose=False):
        super(SaveRewardPortionsCallback, self).__init__(verbose)
        self.fullfilename = fullfilename
        self.write_flag = False
        self.count = 1

    def _init_callback(self) -> None:
        self.writer = open(self.fullfilename, 'w')
        self.writer.write('step r_orn r_avel r_end r_com\n')

    def _on_rollout_end(self) -> None:
        step = [self.count * self.locals['self'].n_steps]
        portions = self.locals['infos'][0]['portions']
        self.writer.write('%d %.2E %.2E %.2E %.2E\n' % tuple(step + portions))
        self.count += 1

    def _on_training_end(self) -> None:
        self.writer.close()
