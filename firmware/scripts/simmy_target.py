"""Optional host-only target: pio run -d firmware -e native -t simmy.

Registration does not require simmy or matplotlib. Neither is needed for
ordinary native tests or the board build. SIMMY_PYTHON selects a host
Python with matplotlib installed (default: python3 on PATH).
"""
import os
from pathlib import Path
import subprocess

Import('env')  # noqa: F821 -- PlatformIO/SCons injects this


def plot_simmy(source, target, env):
    root = Path(env.subst('$PROJECT_DIR')).parent
    subprocess.run([os.environ.get('SIMMY_PYTHON', 'python3'),
                    str(root / 'tools/simmy_plot.py')], check=True)


env.AddCustomTarget(  # noqa: F821
    name='simmy', dependencies=None, actions=plot_simmy,
    title='Plot simmy simulation',
    description='Build Control.c + external simmy on host and render simulated telemetry',
)
