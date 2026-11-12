"""Camera display with nonblocking 30 Hz refresh; events are pumped at 100 Hz."""
import time
import math
import numpy as np
import pygame


class PygameDisplay:
    def __init__(self, config):
        pygame.display.init()
        pygame.font.init()
        self.width, self.height = 960, 540
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('HOCD · T248P shared driving')
        self.font = pygame.font.Font(None, 28)
        self.next_refresh = 0.
        self.image_count = 0
        self.scene = None

    def set_scene(self, scene):
        self.scene = scene
        self.reference = np.array(scene['waypoints'])

    def pump(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT or event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE:
                raise KeyboardInterrupt
            if event.type == pygame.JOYDEVICEREMOVED:
                raise RuntimeError('wheel_disconnected')

    def run_step(self, env, ctrl, status=''):
        now = time.monotonic()
        if now < self.next_refresh:
            return
        self.next_refresh = now + 1. / 30
        if env.image is not None:
            surface = pygame.surfarray.make_surface(env.image.swapaxes(0, 1))
            self.screen.blit(pygame.transform.scale(surface, (self.width, self.height)), (0, 0))
            self.image_count += 1
        else:
            self.screen.fill((25, 25, 25))
        if self.scene:
            self._draw_route(env)
        text = self.font.render(status, True, (255, 255, 255), (0, 0, 0))
        self.screen.blit(text, (12, 12))
        pygame.display.flip()

    def _draw_route(self, env):
        # A fixed reference on the camera and an overview guide every condition.
        ego = env.ego
        delta = self.reference[:, :2] - [ego.x, ego.y]
        forward = delta[:, 0] * math.cos(ego.yaw) + delta[:, 1] * math.sin(ego.yaw) - .6
        right = -delta[:, 0] * math.sin(ego.yaw) + delta[:, 1] * math.cos(ego.yaw)
        focal = self.width / (2 * math.tan(math.radians(55)))
        line = []
        for i in range(len(forward)):
            if 5 < forward[i] < 55:
                x = self.width / 2 + right[i] * focal / forward[i]
                y = self.height / 2 + (ego.z + 1.3 - self.reference[i, 2]) * focal / forward[i]
                if 0 <= x < self.width and 0 <= y < self.height:
                    line.append((int(x), int(y)))
        if len(line) > 1:
            pygame.draw.lines(self.screen, (70, 230, 130), False, line, 3)
        panel = pygame.Rect(self.width - 214, 48, 200, 200)
        pygame.draw.rect(self.screen, (20, 25, 30), panel)
        low = self.reference[:, :2].min(axis=0)
        span = np.maximum(self.reference[:, :2].max(axis=0) - low, 1.)
        scale = 180 / max(span)
        def project(x, y):
            return (int(panel.x + 10 + (x - low[0]) * scale), int(panel.y + 10 + (y - low[1]) * scale))
        pygame.draw.lines(self.screen, (150, 160, 170), False, [project(*p[:2]) for p in self.reference], 2)
        for obstacle in self.scene['obstacles']:
            pygame.draw.circle(self.screen, (240, 110, 45), project(obstacle['x'], obstacle['y']), 4)
        pygame.draw.circle(self.screen, (70, 230, 130), project(ego.x, ego.y), 5)

    def close(self):
        pygame.display.quit()
