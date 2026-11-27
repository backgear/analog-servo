"""Set the CLI's `version` reply from `git describe --tags --always --dirty`.

Writes FW_VERSION into $BUILD_DIR/generated/FwVersion.h, which only
Protocol.c includes, so a new version recompiles that one file instead of
every file a changed -D flag would touch. The header is rewritten only when
the text changes. Outside a git checkout, or without git, the version is
"unknown". --dirty ignores untracked files, so a build that uses a new file
not yet `git add`ed doesn't say -dirty; `get build.id` still tells it apart.

Runs when `pio run` starts, so a commit made between builds is picked up
by the next build, not by a build already in progress.
"""
from pathlib import Path
import subprocess

Import('env')  # noqa: F821 -- PlatformIO/SCons injects this


def describe(project_dir):
    try:
        return subprocess.run(['git', 'describe', '--tags', '--always', '--dirty'],
                              cwd=project_dir, capture_output=True, text=True,
                              check=True).stdout.strip() or 'unknown'
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


header = Path(env.subst('$BUILD_DIR')) / 'generated' / 'FwVersion.h'  # noqa: F821
version = describe(env.subst('$PROJECT_DIR')).replace('\\', '\\\\').replace('"', '\\"')  # noqa: F821
text = '#define FW_VERSION "%s"\n' % version
if not header.exists() or header.read_text() != text:
    header.parent.mkdir(parents=True, exist_ok=True)
    header.write_text(text)
env.Append(CPPPATH=[str(header.parent)])  # noqa: F821
