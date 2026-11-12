"""Inspect pygame axes or save a measured T248P mapping after connection."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import statistics
import time
from zoneinfo import ZoneInfo

import pygame

ROOT = Path(__file__).resolve().parents[1]


def select_wheel_index(wheels, requested_index=None):
    if requested_index is not None:
        if not 0 <= requested_index < len(wheels):
            raise ValueError('无效设备序号')
        return requested_index
    matches = []
    for index, wheel in enumerate(wheels):
        # Linux may report the T248P as "Thrustmaster Racing Wheel FFB".
        # SDL's USB GUID stores vendor/product at bytes 4/8, little endian.
        try:
            guid = bytes.fromhex(wheel.get_guid())
        except ValueError:
            guid = b''
        usb_match = (len(guid) == 16
            and int.from_bytes(guid[0:2], 'little') == 0x0003
            and int.from_bytes(guid[4:6], 'little') == 0x044f
            and int.from_bytes(guid[8:10], 'little') == 0xb696)
        if 't248' in wheel.get_name().lower() or usb_match:
            matches.append(index)
    if len(matches) != 1:
        raise ValueError('无法唯一确定 T248P，请按上面的设备列表指定序号，例如：\n'
                         'bash scripts/probe_wheel.sh --calibrate --device-index 0')
    return matches[0]


def capture(wheel, seconds=1.):
    rows = []
    finish = time.monotonic() + seconds
    while time.monotonic() < finish:
        pygame.event.pump()
        rows.append([float(wheel.get_axis(i)) for i in range(wheel.get_numaxes())])
        time.sleep(.01)
    return [statistics.median(row[i] for row in rows) for i in range(wheel.get_numaxes())]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibrate', action='store_true')
    parser.add_argument('--device-index', type=int)
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    pygame.display.init()
    pygame.joystick.init()
    pygame.display.set_mode((620, 120))
    pygame.display.set_caption('T248P axes · Ctrl+C to finish')
    wheels = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    try:
        for index, wheel in enumerate(wheels):
            wheel.init()
            print(index, wheel.get_name(), 'axes=', wheel.get_numaxes(), 'guid=', wheel.get_guid(), flush=True)
        if not wheels:
            print('未检测到方向盘。接好电源和踏板，接入 USB、等待自校准，再将 MODE → PS/PC 设为 PC。', flush=True)
            return 2
        if args.list:
            return 0
        if args.calibrate:
            try:
                index = select_wheel_index(wheels, args.device_index)
            except ValueError as exc:
                print(str(exc), flush=True)
                return 2
            wheel = wheels[index]
            print(f'已选择 {index} 号方向盘：{wheel.get_name()}', flush=True)
            input('将 ROT° 设为 360°，方向盘回正、踏板保持不动，然后按 Enter：')
            center = capture(wheel)
            input('向左转到设定范围的端点并保持，再按 Enter：')
            left = capture(wheel)
            input('向右转到设定范围的端点并保持，再按 Enter：')
            right = capture(wheel)
            spans = [abs(b - a) for a, b in zip(left, right)]
            axis = max(range(len(spans)), key=spans.__getitem__)
            if spans[axis] < 1.5 or abs(center[axis]) > .15 or left[axis] * right[axis] >= 0:
                raise RuntimeError(f'方向轴范围/回中异常，未保存配置。center={center}, left={left}, right={right}')
            sign = 1 if left[axis] < right[axis] else -1
            input('方向盘回正并保持，然后按 Enter：')
            returned = capture(wheel)
            if abs(returned[axis]) > .15:
                raise RuntimeError('回中偏差过大，未保存配置')
            config = {'device_index': index, 'steering_axis': axis, 'steering_sign': sign,
                'calibrated': True, 'device_name': wheel.get_name(), 'guid': wheel.get_guid(),
                'rotation_degrees': 360, 'center': center[axis], 'left': left[axis], 'right': right[axis],
                'measured_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
                'note': 'Measured pygame mapping; confirm vehicle left/right during the direct practice run.'}
            (ROOT / 'configs/wheel.json').write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
            print('已保存 configs/wheel.json:', config, flush=True)
            return 0
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE:
                    return 0
            for i, wheel in enumerate(wheels):
                print(i, [round(wheel.get_axis(j), 3) for j in range(wheel.get_numaxes())], flush=True)
            time.sleep(.2)
    except KeyboardInterrupt:
        return 0
    finally:
        for wheel in wheels:
            wheel.quit()
        pygame.quit()


if __name__ == '__main__':
    raise SystemExit(main())
