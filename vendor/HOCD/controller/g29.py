"""T248P axis acquisition; the historic class name is retained for HOCD."""
import math
import pygame


class G29:
    def __init__(self, config=None):
        config = config or {}
        self.connected = False
        self._joystick = None
        self._steer_idx = int(config.get('steering_axis', 0))
        self._steer_sign = float(config.get('steering_sign', 1))
        if self._steer_sign not in (-1., 1.):
            raise ValueError('steering_sign must be 1 or -1')
        if config.get('diagnostic_input', False):
            return
        pygame.joystick.init()
        index = int(config.get('device_index', 0))
        if not 0 <= index < pygame.joystick.get_count():
            raise RuntimeError('T248P not found. Connect it and run scripts/probe_wheel.sh first.')
        self._joystick = pygame.joystick.Joystick(index)
        self._joystick.init()
        if config.get('expected_name') and self._joystick.get_name() != config['expected_name']:
            raise RuntimeError('Joystick identity changed; probe the T248P again')
        if config.get('expected_guid') and self._joystick.get_guid() != config['expected_guid']:
            raise RuntimeError('Joystick GUID changed; probe the T248P again')
        if not 0 <= self._steer_idx < self._joystick.get_numaxes():
            raise ValueError('Configured steering axis is unavailable')
        self.connected = True

    def sample(self):
        if self._joystick is None:
            raise RuntimeError('No live wheel in diagnostic mode')
        if not self._joystick.get_init() or not self.connected:
            raise RuntimeError('T248P disconnected')
        value = self._steer_sign * float(self._joystick.get_axis(self._steer_idx)) * 3.14
        if not math.isfinite(value):
            raise RuntimeError('Nonfinite steering input')
        return value

    def control(self, *args, **kwargs):
        raise RuntimeError('Sample T248P once in the main loop; SharedControl must use delayed human_steer')

    def close(self):
        if self._joystick is not None:
            self._joystick.quit()
        self.connected = False
