#!/usr/bin/env python3
"""Export servo run JSON or binary monitor recordings to mixed-signal srzip.

Missing MCU ticks become zero placeholders with Sample_present=0; no time
compression or interpolation. Original integer data stays in the input file.
"""
import argparse
import csv
import gzip
import json
import subprocess
import tempfile
from pathlib import Path

COLUMNS = ['Tracking_error',
           'Measured_velocity_counts_per_tick', 'Reference_velocity_counts_per_tick',
           'Torque_command_V', 'Enabled', 'Fault_present', 'Hardware_enable',
           'Synthetic', 'Cycle_start', 'Sample_present', 'Error_valid']


def read_frames(path):
    if path.suffix == '.json' or path.name.endswith('.json.gz'):
        text = gzip.decompress(path.read_bytes()).decode() if path.suffix == '.gz' else path.read_text()
        document = json.loads(text)
        return document['frames'], {k:v for k,v in document.items() if k != 'frames'}
    from monitor_frames import Frames
    frames = []
    decoder = Frames()
    with path.open('rb') as source:
        while data := source.read(65536):
            for seq, row in decoder.feed(data):
                logic = row['logic']
                frames.append(dict(seq=seq, position=row['position'],
                    cmd_position=row['reference'], velocity=row['velocity'],
                    cmd_velocity=row['reference_velocity'], torque=row['torque'],
                    enabled=bool(logic & 1), fault_latched=bool(logic & 2),
                    hw_enable=bool(logic & 4), synthetic=bool(logic & 8),
                    cycle_start=bool(logic & 16)))
    return frames, {}


def write_csv(frames, destination, error_kind="position"):
    if error_kind not in ("position", "velocity"):
        raise ValueError("Choose position or velocity error")
    if not frames:
        raise ValueError('No decoded telemetry frames')
    origin = frames[0]['position']
    previous = None
    missing = 0
    with destination.open('w', newline='') as stream:
        writer = csv.writer(stream)
        columns = list(COLUMNS)
        columns[0] = "Following_error_counts" if error_kind == "position" else "Velocity_error_counts_per_tick"
        writer.writerow(columns)
        for frame in frames:
            seq = frame['seq']
            if previous is not None:
                delta = (seq - previous) & 0xffffffff
                if not 1 <= delta <= 10001:
                    raise ValueError('Duplicate/reset/large tick gap; split recording into sessions')
                for _ in range(delta-1):
                    writer.writerow([0]*len(COLUMNS))
                missing += delta-1
            previous = seq
            error = frame['cmd_position']-frame['position'] if error_kind == 'position' else frame['cmd_velocity']-frame['velocity']
            valid = frame['enabled'] and not frame['fault_latched'] and not frame['synthetic']
            writer.writerow([error,
                frame['velocity'], frame['cmd_velocity'], frame['torque'] * 3.3 / 32767,
                int(frame['enabled']), int(frame['fault_latched']),
                int(frame['hw_enable']), int(frame['synthetic']),
                int(frame['cycle_start']), 1, int(valid)])
    return dict(position_origin=origin, start_tick=frames[0]['seq'],
                end_tick=frames[-1]['seq'], received_samples=len(frames),
                missing_samples=missing, total_samples=len(frames)+missing,
                samplerate=1000, error_kind=error_kind,
                error_definition="commanded minus measured " + error_kind,
                torque_scale_volts_per_unit=3.3/32767,
                torque_full_scale_volts=3.3,
                torque_definition="Signed nominal differential command voltage, before trim/DAC quantization; not a voltage measurement")


def export(source, output, error_kind="auto"):
    if output.suffix != ".sr":
        raise ValueError("Output must have .sr extension")
    # Do not replace previous recordings or their provenance.
    if output.exists() or output.with_suffix('.json').exists():
        raise FileExistsError(output)
    frames, context = read_frames(source)
    if error_kind == "auto":
        settings = context.get('run_settings', context.get('initial', {}))
        feedback = settings.get('feedback.source')
        if feedback not in ('encoder', 'tach_emulated'):
            raise ValueError('Feedback source unknown; specify --error position or --error velocity')
        error_kind = 'velocity' if feedback == 'tach_emulated' else 'position'
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.servo-srzip-', dir=output.parent) as directory:
        csv_path = Path(directory)/'samples.csv'
        metadata = write_csv(frames, csv_path, error_kind)
        candidate = Path(directory)/'capture.sr'
        subprocess.run(['sigrok-cli', '-I',
            'csv:header=true:column_formats=4a,7l:samplerate=1000',
            '-i', str(csv_path), '-O', 'srzip', '-o', str(candidate)], check=True)
        if not candidate.is_file():
            raise RuntimeError("sigrok-cli produced no archive; check its backend initialization errors")
        # Verify libsigrok can reopen the output before publishing it.
        subprocess.run(['sigrok-cli', '-i', str(candidate), '--show'], check=True,
                       stdout=subprocess.DEVNULL)
        candidate.replace(output)
    metadata.update(source=str(source.resolve()), context=context,
        notes='Controller quantities, not measured volts/current. Torque_command_V is a nominal signed command voltage: raw torque * 3.3 / 32767, independent of the configured torque clamp. Missing ticks are zero placeholders with Sample_present=0. Tracking error is centered on zero. Error_valid is low for disabled, faulted, synthetic or missing samples; unknown branches/config transitions are not fully represented by telemetry. In tach mode velocity error uses the real encoder, not the decimated/filtered PI measurement. Position error requires encoder feedback and must not be used for tach control. Original integers remain in the source recording.')
    output.with_suffix('.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='run JSON with frames, or raw monitor binary')
    parser.add_argument('output', type=Path, help='new .sr archive; provenance saved beside it')
    parser.add_argument('--error', choices=['auto', 'position', 'velocity'], default='auto', help='auto reads feedback.source from run JSON; required explicitly for raw binary')
    args = parser.parse_args()
    if args.source.resolve() == args.output.with_suffix('.json').resolve():
        parser.error('Output provenance must not overwrite the source')
    print(json.dumps(export(args.source, args.output, args.error), indent=2))


if __name__ == '__main__':
    main()
