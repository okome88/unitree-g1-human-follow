"""Check the pinned runtime without importing or starting Isaac Sim."""
import importlib.util
import importlib.metadata
import json
import subprocess
import sys


def check_runtime():
    errors = []
    versions = {}
    if sys.version_info[:2] != (3,11):
        errors.append('Isaac Sim 5.1 requires Python 3.11; use the separate Isaac environment.')
    for package, expected in [('isaaclab','2.3.0'), ('isaacsim','5.1.0')]:
        try:
            found = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            found = None
        versions[package] = found
        if found is None or found.split('+')[0] != expected:
            errors.append(f'{package}=={expected} required; found {found}')
    for module in ['torch','onnxruntime','yaml','numpy']:
        if importlib.util.find_spec(module) is None:
            errors.append(f'Missing dependency: {module}')
    gpu = None
    try:
        gpu = subprocess.check_output(['nvidia-smi','--query-gpu=name,memory.total',
            '--format=csv,noheader,nounits'], text=True, timeout=10).strip()
    except (OSError, subprocess.SubprocessError):
        errors.append('NVIDIA GPU/driver could not be queried.')
    return {'ready':not errors, 'errors':errors, 'python':sys.version.split()[0],
            'versions':versions, 'gpus':gpu,
            'hardware_note':'Official minimum: 16 GB VRAM and 32 GB RAM. A passed software check does not validate hardware or physics.'}


if __name__ == '__main__':
    result = check_runtime()
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['ready'] else 2)
