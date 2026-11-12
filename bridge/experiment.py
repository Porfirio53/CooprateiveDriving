"""The single-driver, three-map experiment; no simulator or model side effects."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAPS = ('town03', 'town04', 'town05')
FORMAL_CONDITIONS = ('hold', 'tirex', 'manual_tirex')


def condition_settings(condition):
    if condition not in ('direct', *FORMAL_CONDITIONS):
        raise ValueError(f'Unknown condition: {condition}')
    return {
        'bridge_condition': 'tirex' if condition == 'manual_tirex' else condition,
        'shared_control': condition != 'manual_tirex',
        'delay_us': 0 if condition == 'direct' else 300_000,
        'prediction': condition in ('tirex', 'manual_tirex'),
    }


def load_scene(map_name):
    if map_name not in MAPS:
        raise ValueError(f'Choose one of {MAPS}')
    path = ROOT / 'configs/scenarios' / (map_name + '.json')
    scene = json.loads(path.read_text())
    if scene['town'].removesuffix('_Opt').lower() != map_name or not scene['obstacles']:
        raise ValueError('Scene town/obstacles are not configured')
    if scene['target_speed_kmh'] != 30 or not 220 < scene['length_m'] < 340:
        raise ValueError('Expected a short 30 km/h formal route')
    return path, scene


def terminal_state(*, collisions, lateral_error, progress, route_length,
                   distance_to_end, elapsed, max_seconds, diagnostic=False,
                   off_route_limit=5., finish_distance=5.):
    """Classify observed physical outcomes before requesting another plan."""
    if collisions:
        return {'completed': False, 'exit_reason': 'collision'}
    if abs(lateral_error) > off_route_limit:
        return {'completed': False, 'exit_reason': 'left_route'}
    if elapsed is not None:
        if distance_to_end < finish_distance and progress > route_length - 12:
            return {'completed': True, 'exit_reason': 'route_complete'}
        if elapsed >= max_seconds:
            return {'completed': False, 'exit_reason': 'diagnostic_duration' if diagnostic else 'time_limit'}
    return None
