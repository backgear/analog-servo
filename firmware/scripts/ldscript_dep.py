"""Relink when the env's linker script (ld/, board_build.ldscript) changes.

SCons doesn't scan -T scripts, so without this an edit to ld/*.ld leaves
the old firmware.elf in place until something else forces a link.
"""
import os

Import('env')  # noqa: F821 -- PlatformIO/SCons injects this

script = os.path.join(env.subst('$PROJECT_DIR'), env.GetActualLDScript())  # noqa: F821
env.Depends('$BUILD_DIR/${PROGNAME}.elf', script)  # noqa: F821
