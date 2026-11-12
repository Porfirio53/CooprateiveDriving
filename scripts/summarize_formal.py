"""Keep all attempts of the current nine groups, excluding diagnostics/old runs."""
import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    design = json.loads((ROOT / 'configs/formal_experiment.json').read_text())
    records = []
    for path in sorted((ROOT / 'runs').glob('*/config.json')):
        config = json.loads(path.read_text())
        if (config.get('experiment_id') != design['id'] or not config.get('formal')
                or config.get('diagnostic_input')):
            continue
        summary_path = path.with_name('summary.json')
        metrics_path = path.with_name('metrics.json')
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else {}
        records.append({'trial_id': config['trial_id'], 'participant': config['participant'],
            'map': config['map'], 'condition': config['condition'], 'repeat': config['repeat'],
            'status': summary.get('status', config['status']), 'completed': summary.get('completed', False),
            'exit_reason': summary.get('exit_reason'), 'run_directory': str(path.parent),
            **{key: metrics.get(key) for key in ('collision_count', 'scored_ticks', 'lateral_rmse_m',
                'steer_mae', 'steer_rmse', 'mean_native_aoi_at_actual_tick_ms', 'mean_inference_ms',
                'fallback_fraction', 'scoring_duration_s', 'mean_speed_kmh', 'mean_automatic_authority',
                'trajectory_rows', 'trajectory_frame_gaps')}})
    with (ROOT / 'configs/participant_schedule.csv').open() as stream:
        schedule = list(csv.DictReader(stream))
    groups = []
    for group in schedule:
        attempts = [r for r in records if all(str(r[k]) == group[k] for k in ('participant', 'map', 'condition', 'repeat'))]
        groups.append({**group, 'status': 'recorded' if attempts else 'pending',
            'attempts': len(attempts), 'completed_attempts': sum(r['completed'] for r in attempts),
            'trial_ids': [r['trial_id'] for r in attempts]})
    report = {'experiment_id': design['id'], 'planned_groups': 9,
        'groups_with_records': sum(g['attempts'] > 0 for g in groups), 'groups': groups, 'all_attempts': records}
    output = ROOT / 'runs/formal_results.json'
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    if records:
        with (ROOT / 'runs/formal_results.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, list(records[0]))
            writer.writeheader()
            writer.writerows(records)
    print(json.dumps({k: report[k] for k in ('experiment_id', 'planned_groups', 'groups_with_records')}, indent=2))
    print(output)


if __name__ == '__main__':
    main()
