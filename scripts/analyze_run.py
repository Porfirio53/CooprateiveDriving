"""Score all driving ticks, including forecast fallbacks and unfinished runs."""
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def analyze(directory, plot=False):
    directory = Path(directory)
    cfg = json.loads((directory / 'config.json').read_text())
    ticks_path = directory / 'ticks.csv'
    ticks = pd.read_csv(ticks_path) if ticks_path.exists() else pd.DataFrame(columns=['phase'])
    scored = ticks[ticks.phase == 'score'].copy()
    result = {k: cfg[k] for k in ('trial_id', 'participant', 'condition', 'diagnostic_input')}
    result.update(map=cfg.get('map', 'legacy_town04'), repeat=cfg.get('repeat', 1),
                  formal=cfg.get('formal', False))
    summary = json.loads((directory / 'summary.json').read_text())
    result['status'] = summary['status']
    result.update(completed=summary['completed'], exit_reason=summary['exit_reason'],
                  collision_count=len(summary.get('collisions', [])), scored_ticks=len(scored))
    if len(scored):
        raw = pd.read_json(directory / 'bridge-raw.jsonl', lines=True)
        raw = raw.sort_values('native_source_time_us')
        source = raw.native_source_time_us.to_numpy(dtype=np.int64)
        native = raw.steer_h.to_numpy(dtype=float)
        use = scored.actual_tick_start_us.to_numpy(dtype=np.int64)
        indices = np.searchsorted(source, use, side='right') - 1
        if (indices < 0).any():
            raise ValueError('Missing causal raw ground truth at application')
        truth = native[indices]
        err = scored.human_steer_est.to_numpy() - truth
        scored['truth_steer_at_actual_tick'] = truth
        scored['steer_estimation_error'] = err
        observed = scored.last_native_source_time_us.notna()
        native_age = (scored.loc[observed, 'actual_tick_start_us'] - scored.loc[observed, 'last_native_source_time_us']) / 1000
        result.update(lateral_rmse_m=float(np.sqrt(np.mean(scored.lateral_error_m ** 2))),
            steer_mae=float(np.mean(np.abs(err))), steer_rmse=float(np.sqrt(np.mean(err ** 2))),
            mean_grid_aoi_at_planned_use_ms=float(scored.aoi_us.mean() / 1000),
            mean_native_aoi_at_actual_tick_ms=float(native_age.mean()),
            mean_target_lateness_ms=float(scored.target_lateness_us.mean() / 1000),
            p95_target_lateness_ms=float(scored.target_lateness_us.quantile(.95) / 1000),
            trajectory_fraction=float((scored.selection_mode == 'trajectory').mean()),
            fallback_fraction=float((scored.selection_mode != 'trajectory').mean()) if cfg.get('prediction', cfg['condition'] == 'tirex') else 0.,
            mean_speed_kmh=float(scored.speed_mps.mean() * 3.6),
            mean_automatic_authority=float(scored.authority.mean()),
            scoring_duration_s=summary.get('scoring_duration_s'),
            mean_tick_interval_ms=float(scored.tick_interval_us.mean() / 1000),
            p95_tick_interval_ms=float(scored.tick_interval_us.quantile(.95) / 1000),
            frame_mismatches=int((scored.target_frame != scored.actual_apply_frame).sum()),
            sim_wall_ratio=float((scored.sim_time_s.iloc[-1] - scored.sim_time_s.iloc[0]) /
                max(1e-6, (scored.wall_time_us.iloc[-1] - scored.wall_time_us.iloc[0]) / 1e6)))
        scored.to_csv(directory / 'scored_ticks.csv', index=False)
    inference_path = directory / 'bridge-inferences.jsonl'
    if inference_path.exists() and inference_path.stat().st_size:
        requests = pd.read_json(inference_path, lines=True)
        result.update(mean_inference_ms=float(requests.compute_us.mean() / 1000),
            p95_inference_ms=float(requests.compute_us.quantile(.95) / 1000),
            inference_failures=int((requests.status != 'ok').sum()),
            control_margin_covered=bool((requests.planned_steps * 25_000 >=
                requests.aoi_us + requests.queue_us + requests.compute_estimate_us + requests.reuse_us + 10_000).all()))
    else:
        result['mean_inference_ms'] = None
    trajectory_path = directory / 'trajectory.csv'
    if trajectory_path.exists():
        trajectory = pd.read_csv(trajectory_path)
        result['trajectory_rows'] = len(trajectory)
        result['trajectory_frame_gaps'] = int((trajectory.frame.diff().dropna() != 1).sum())
    if plot:
        from bridge.run_artifacts import plot_trajectory
        plot_trajectory(directory)
    (directory / 'metrics.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--plot', action='store_true', help='Optionally generate one trajectory figure per run')
    args = parser.parse_args()
    results = [analyze(path, plot=args.plot) for path in args.runs]
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
