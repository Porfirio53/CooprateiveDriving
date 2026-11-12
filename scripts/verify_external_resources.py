"""Audit external resources against the pre-deployment size/mtime/mode manifest."""
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    baseline = json.loads((ROOT / 'logs/external-resources-before.json').read_text())
    report = {}
    for base, entry in baseline.items():
        current = {}
        for directory, dirs, names in os.walk(base):
            dirs[:] = [d for d in dirs if d != '.git']
            for name in names:
                path = Path(directory) / name
                stat = path.lstat()
                current[str(path.relative_to(base))] = [stat.st_size, stat.st_mtime_ns, stat.st_mode]
        old = entry['files']
        status = subprocess.check_output(['git', '--no-optional-locks', '-C', base, 'status', '--porcelain'], text=True)
        report[base] = {'baseline_files': len(old), 'current_files': len(current),
            'added': sorted(current.keys() - old.keys()), 'removed': sorted(old.keys() - current.keys()),
            'changed': sorted(k for k in current.keys() & old.keys() if current[k] != old[k]),
            'git_status_unchanged': status == entry['git_status']}
    passed = all(not e['added'] and not e['removed'] and not e['changed'] and e['git_status_unchanged'] for e in report.values())
    result = {'passed': passed, 'method': 'all files outside .git: size, mtime_ns, mode; plus git status', 'resources': report}
    (ROOT / 'logs/external-resources-audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
