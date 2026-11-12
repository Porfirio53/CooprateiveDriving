"""Read the existing current TiRex; keep every runtime write in COOP_ROOT.

Only construction/loading/metadata differ from the external TiRexModel.
Context preparation, trend processing and forecasting are inherited unchanged.
"""
from dataclasses import asdict, replace
import importlib
import os
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

RESOURCE_ROOT = Path('/home/patton/MyTirex')
RUN_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RESOURCE_ROOT / 'src'))

from mytirex.common import assert_local_imports, read_json, source_hash, verify_files
from mytirex.inference.gpu import cache_snapshot, configure_runtime
from mytirex.inference.model import TiRexModel, read_tuning


class ExternalTiRexModel(TiRexModel):
    def __init__(self, run_root=RUN_ROOT):
        self.root = Path(run_root).resolve()
        if self.root != RUN_ROOT or self.root == RESOURCE_ROOT:
            raise ValueError('Runtime root must be the cooperative_driving workspace')
        self.resource_root = RESOURCE_ROOT
        self.settings = read_tuning(self.root, 'configs/tirex_tuning.json')
        self.runtime = {**read_json(self.root / 'configs/experiment.json')['runtime'], 'batch_size': 1}
        if self.runtime['device'] != 'cuda':
            raise ValueError('Current TiRex requires CUDA')
        self.source = 'current'
        self.context_length = self.settings['context_length']
        self.batch_size = 1
        self.source_directory = self.resource_root / 'vendor/tirex'
        self.source_sha256 = source_hash(self.source_directory)
        self.model_lock = read_json(self.resource_root / 'configs/model_manifest.json')['TiRex-2']
        self.model_path = self.resource_root / 'models/tirex2'
        verify_files(self.model_path, self.model_lock['sha256'])
        self.model = None
        self.batch_timings = []
        self.differencing_counts = {'target_rows_seen': 0, 'differenced_rows': 0}
        self.first_forecast = None
        self.current_unit_seed = None
        self.seed_policy = 'Existing current TiRex seed policy; batch_size=1; no training'

    def load(self):
        if self.model is not None:
            return
        self.cache_before_load = cache_snapshot(self.root)
        started = perf_counter()
        configure_runtime(self.root)
        for key in ('HF_HUB_OFFLINE', 'TRANSFORMERS_OFFLINE', 'HF_HUB_DISABLE_TELEMETRY'):
            os.environ[key] = '1'
        source = self.source_directory / 'src'
        imported = sys.modules.get('tirex2')
        if imported is not None and Path(imported.__file__).resolve().parent != source / 'tirex2':
            raise RuntimeError('Unexpected tirex2 source already loaded')
        sys.path.insert(0, str(source))
        import torch
        self.torch = torch
        torch.set_num_threads(int(self.runtime['num_threads']))
        torch.manual_seed(int(self.runtime['seed']))
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA unavailable')
        package = importlib.import_module('tirex2')
        if Path(package.__file__).resolve().parent != source / 'tirex2':
            raise RuntimeError(f'Unexpected tirex2: {package.__file__}')
        self.timeseries_type = package.TimeseriesType
        self.model = package.load_model(self.model_path, device='cuda', compile=False,
                                       use_flex_attention=False, hf_kwargs={'local_files_only': True})
        self.model.eval()
        if self.context_length > int(self.model.context_len):
            raise ValueError('Unsupported context length')
        devices = {str(p.device) for p in self.model.parameters()}
        if not devices or any(not d.startswith('cuda:') for d in devices):
            raise RuntimeError(f'Unexpected parameter devices: {devices}')
        self.parameter_devices = sorted(devices)
        processor = self.model.model.postprocessor
        processor.cfg = replace(processor.cfg,
            trend_window_points=self.settings['trend_window_points'],
            trend_threshold=self.settings['trend_threshold'],
            trend_min_r2=self.settings['trend_min_r2'])
        self.postprocessor_settings = asdict(processor.cfg)
        transform_input = processor.transform_input

        def audited(*args, **kwargs):
            result = transform_input(*args, **kwargs)
            for mask in result[2]['diff_masks']:
                self.differencing_counts['target_rows_seen'] += len(mask)
                self.differencing_counts['differenced_rows'] += sum(int(v) for v in mask)
            return result

        processor.transform_input = audited
        self.quantiles = self.model.quantiles.detach().cpu().numpy().copy()
        if np.flatnonzero(np.isclose(self.quantiles, .5)).size != 1:
            raise ValueError('A unique median quantile is required')
        torch.cuda.synchronize()
        self.load_time_s = perf_counter() - started
        self.cache_after_load = cache_snapshot(self.root)
        self.package_files = assert_local_imports(self.resource_root)

    def metadata(self):
        return {
            'source': self.source, 'source_directory': str(self.source_directory),
            'source_sha256': self.source_sha256, 'resource_root': str(self.resource_root),
            'runtime_root': str(self.root), 'model_directory': str(self.model_path),
            'model_lock': self.model_lock, 'settings': self.settings, 'runtime': self.runtime,
            'package_files': assert_local_imports(self.resource_root),
            'parameter_devices': self.parameter_devices, 'load_time_s': self.load_time_s,
            'first_forecast': self.first_forecast, 'cache_before_load': self.cache_before_load,
            'cache_after_load': self.cache_after_load, 'gpu_name': self.torch.cuda.get_device_name(),
            'torch_version': self.torch.__version__, 'torch_cuda_version': self.torch.version.cuda,
            'forecast_options': self.forecast_options(),
            'load_options': {'compile': False, 'use_flex_attention': False, 'local_files_only': True},
            'postprocessor_settings': self.postprocessor_settings,
            'differencing_counts': self.differencing_counts,
            'batch_count': len(self.batch_timings), 'training': 'none',
        }


def warmup_model(model, requests=4):
    """Kernel preparation only; these diagnostics are not experiment samples."""
    context = .08 * np.sin(np.arange(256, dtype=float) * .035)
    timings = []
    for _ in range(requests):
        result = model.predict_signals({'steer_h': context}, 32, units={'steer_h': 'hocd_steer'})
        if result.status != 'ok' or result.fallback:
            raise RuntimeError(f'TiRex warmup failed: {result.failure_reason}')
        timings.append(result.batch_elapsed_s)
    return timings


if __name__ == '__main__':
    import json
    model = ExternalTiRexModel()
    model.load()
    timings = warmup_model(model)
    report = {**model.metadata(), 'warmup_s': timings, 'status': 'passed'}
    (RUN_ROOT / 'logs/tirex-warmup.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': 'passed', 'warmup_s': timings, 'gpu': report['gpu_name']}), flush=True)
