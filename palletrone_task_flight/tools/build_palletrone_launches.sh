#!/usr/bin/env bash
# User-operated build only. This script never launches a ROS node or a driver.
set -eo pipefail
REPO=/home/palletrone/ros2_ws/src/PX4-Modifying-Lattepanda
ENV_FILE=/home/palletrone/palletrone_env.bash
source /opt/ros/humble/setup.bash
source /home/palletrone/ros2_ws/install/setup.bash

if [[ -L "$ENV_FILE" ]]; then
    printf 'Refusing to replace an environment-file symlink: %s
' "$ENV_FILE" >&2
    exit 1
fi
if [[ -e "$ENV_FILE" ]] && ! head -n 1 "$ENV_FILE" | grep -Fxq '# PALLETRONE_LAUNCH_ENV_V1'; then
    printf 'Existing unmanaged file preserved: %s
' "$ENV_FILE" >&2
    exit 1
fi

RUNTIME_DIR=$(mktemp -d "$HOME/palletrone_task_runtime.XXXXXX")
printf 'New isolated build/install directory: %s
' "$RUNTIME_DIR"
cd /home/palletrone/ros2_ws
PYTHONDONTWRITEBYTECODE=1 colcon --log-base "$RUNTIME_DIR/log" build \
    --base-paths "$REPO/palletrone_task_flight" \
    --packages-select palletrone_task_flight \
    --build-base "$RUNTIME_DIR/build" \
    --install-base "$RUNTIME_DIR/install" \
    --symlink-install --allow-overriding palletrone_task_flight \
    --cmake-args -DBUILD_TESTING=OFF

source "$RUNTIME_DIR/install/local_setup.bash"
PKG_PREFIX=$(ros2 pkg prefix palletrone_task_flight)
if [[ "$PKG_PREFIX" != "$RUNTIME_DIR/install/palletrone_task_flight" ]]; then
    printf 'Unexpected selected package: %s
' "$PKG_PREFIX" >&2
    exit 1
fi

python3 -B - "$REPO/palletrone_task_flight" "$PKG_PREFIX" "$RUNTIME_DIR" "$ENV_FILE" <<'PY'
import ast
import os
from pathlib import Path
import shutil
import sys
import tempfile
import yaml
import palletrone_task_flight.configuration as configuration
from palletrone_task_flight.trajectory import Plan

source, prefix, runtime, env_path = map(Path, sys.argv[1:])
share = prefix / 'share/palletrone_task_flight'
for name in ('configuration.py', 'trajectory.py', 'state_machine.py', 'messages.py', 'output.py'):
    module = __import__('palletrone_task_flight.'+name[:-3], fromlist=['unused'])
    if Path(module.__file__).read_bytes() != (source/'palletrone_task_flight'/name).read_bytes():
        raise SystemExit('Selected module differs from current source: '+name)
entry = prefix/'lib/palletrone_task_flight/task_flight.py'
if entry.read_bytes() != (source/'scripts/task_flight.py').read_bytes():
    raise SystemExit('Installed task entry point differs from current source')
for name in ('task_short.launch.py', 'j1_live.launch.py'):
    path = share/'launch'/name
    ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
p = configuration.load_yaml(share/'config/task_short_live.yaml')
plan = Plan((0.0, 0.0, 0.0), 0.0, p)
approaches = [s.name for s in plan.segments if s.name.startswith('APPROACH_')]
if p.dry_run or approaches != ['APPROACH_P1_0', 'APPROACH_P2_0']:
    raise SystemExit('Unexpected live task profile')
j1 = yaml.safe_load((share/'config/j1_live.yaml').read_text())['dynamixel_fix_with_j1']['ros__parameters']
if (j1['dry_run'], j1['zero_tick'], j1['direction'], j1['expected_j1_model_number']) != (False, 802, -1, 1120):
    raise SystemExit('Unexpected J1 operational profile')

marker = '# PALLETRONE_LAUNCH_ENV_V1'
if env_path.is_symlink():
    raise SystemExit('Environment path became a symlink; preserved')
if env_path.exists():
    if env_path.read_text(encoding='utf-8').splitlines()[0] != marker:
        raise SystemExit('Existing unmanaged environment file preserved')
    backup = Path(tempfile.mkdtemp(prefix='palletrone_env_backup_', dir=env_path.parent))
    shutil.copy2(env_path, backup/env_path.name)
    print('Previous environment file backup:', backup/env_path.name)
text = (marker+'\n'
        '# Source this file in each terminal; it does not start any process.\n'
        'source /opt/ros/humble/setup.bash\n'
        'source /home/palletrone/ros2_ws/install/setup.bash\n'
        'source "'+str(runtime/'install/local_setup.bash')+'"\n')
fd, tmp_name = tempfile.mkstemp(prefix='.palletrone_env_', dir=env_path.parent)
with os.fdopen(fd, 'w', encoding='utf-8') as stream:
    stream.write(text)
os.replace(tmp_name, env_path)
print('Selected package:', prefix)
print('Selected Python configuration:', configuration.__file__)
print('Short-task nominal duration:', round(plan.nominal_duration, 1), 's')
print('Environment file:', env_path)
print('No nodes or hardware were started.')
PY
