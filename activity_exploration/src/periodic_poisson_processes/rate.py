#!/usr/bin/env python

from scipy.stats import gamma


class Lambda(object):

    def __init__(self, interval=1):
        self.reset()
        self.interval = interval

    def reset(self):
        self.scale = 1.0
        self.shape = 1.1
        self._gamma_map = self._gamma_mode(self.shape, self.scale)
        self._gamma_mean = gamma.mean(self.shape, scale=1/float(self.scale))

    def update_lambda(self, data):
        self.shape += sum(data)
        self.scale += len(data) * self.interval
        self._gamma_map = self._gamma_mode(self.shape, self.scale)
        self._gamma_mean = gamma.mean(self.shape, scale=1/float(self.scale))
        print(
            "Updated scale: %.2f, shape: %.2f, rate:%.2f" % (
                self.scale, self.shape, self.get_rate()
            )
        )

    def _gamma_mode(self, shape, scale):
        if shape >= 1:
            return (shape - 1) / float(scale)
        else:
            return -1.0

    def get_rate(self):
        return self._gamma_map

    def set_rate(self, value):
        self._gamma_map = value
