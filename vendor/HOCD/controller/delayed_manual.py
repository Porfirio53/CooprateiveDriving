"""Human-only lateral control, fed exclusively by the received TiRex chain."""
import math

from controller.g29 import G29
from utils.data import LatDebugInfo


class DelayedManualControl:
    def __init__(self, config):
        self.driver_control = G29(config.controller.lateral.G29)

    def control(self, path_list, state, err, min_index, **kwargs):
        if 'human_steer' not in kwargs:
            raise ValueError('DelayedManualControl requires the bridge human_steer')
        steer = float(kwargs['human_steer'])
        if not math.isfinite(steer):
            raise ValueError('Nonfinite bridge steering')
        return steer, LatDebugInfo(authority=0., steer_driver=steer,
            steer_vehicle=0., g29_connected=self.driver_control.connected, intention='')
