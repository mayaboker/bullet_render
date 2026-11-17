"""Stop stale processes from this Bullet workspace before a new flight launch."""
import argparse
import os
from pathlib import Path
import signal
import socket
import time


WORKSPACE = Path(__file__).resolve().parents[2]
GRACE_S = 3.
TCP_PORTS = (5555, 5556, 5557, 5558, 5559, 5761)
UDP_PORTS = (9002, 9003, 9004)


def process_kind(arguments, cwd, workspace=WORKSPACE):
    values = [os.fsdecode(value) for value in arguments]
    joined = '\0'.join(values)
    workspace = str(workspace.resolve())
    if not values or workspace not in joined:
        return None
    if 'tools/run_flight.py' in joined and Path(cwd).resolve() == Path(workspace) / 'bullet_render':
        return 'launcher'
    physics_configs = (f'{workspace}/config/', f'{workspace}/bullet_drone/configs/')
    relative_workspace_config = ('../config/' in joined
                                 and Path(cwd).resolve() == Path(workspace) / 'bullet_drone')
    if ('-m\0examples.external_rc.main' in joined
            and (relative_workspace_config or any(path in joined for path in physics_configs))):
        return 'physics'
    if '-m\0mcap_bagger.cli' in joined and f'{workspace}/bullet_render/configs/recorder.yaml' in joined:
        return 'recorder'
    if f'{workspace}/.venv/bin/bullet-app' in joined and f'{workspace}/bullet_render/configs/transport.json' in joined:
        return 'app'
    if f'{workspace}/bullet_render/godot' in values:
        return 'godot'
    if f'{workspace}/bullet_drone/.sitl/betaflight-source/obj/main/betaflight_SITL.elf' in values:
        return 'sitl'
    return None


def processes():
    own_pid = os.getpid()
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            arguments = (entry / 'cmdline').read_bytes().split(b'\0')[:-1]
            cwd = (entry / 'cwd').resolve()
            kind = process_kind(arguments, cwd)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if kind is not None:
            yield int(entry.name), kind, ' '.join(os.fsdecode(value) for value in arguments)


def send(pid, sig):
    try:
        target = -pid if os.getpgid(pid) == pid else pid
        os.kill(target, sig)
    except (ProcessLookupError, PermissionError):
        pass


def wait_for_exit(pids, timeout_s):
    deadline = time.monotonic() + timeout_s
    remaining = set(pids)
    while remaining and time.monotonic() < deadline:
        remaining = {pid for pid in remaining if Path(f'/proc/{pid}').exists()}
        if remaining:
            time.sleep(.05)
    return remaining


def check_ports():
    busy = []
    for kind, ports in ((socket.SOCK_STREAM, TCP_PORTS), (socket.SOCK_DGRAM, UDP_PORTS)):
        for port in ports:
            with socket.socket(socket.AF_INET, kind) as probe:
                if kind == socket.SOCK_STREAM:
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    probe.bind(('127.0.0.1', port))
                except OSError:
                    busy.append(port)
    if busy:
        raise RuntimeError(f'Flight ports still in use after cleanup: {", ".join(map(str, busy))}')


def cleanup(*, dry_run=False):
    initial = list(processes())
    if dry_run:
        for pid, kind, command in initial:
            print(f'{pid} {kind}: {command}')
        return len(initial)
    launchers = [pid for pid, kind, _ in initial if kind == 'launcher']
    for pid in launchers:
        send(pid, signal.SIGTERM)
    wait_for_exit(launchers, GRACE_S)
    found = list(processes())
    workers = [pid for pid, kind, _ in found if kind != 'recorder']
    recorders = [pid for pid, kind, _ in found if kind == 'recorder']
    for pid in workers:
        send(pid, signal.SIGTERM)
    remaining = wait_for_exit(workers, GRACE_S)
    for pid in remaining:
        send(pid, signal.SIGKILL)
    for pid in recorders:
        send(pid, signal.SIGTERM)
    remaining = wait_for_exit(recorders, GRACE_S)
    for pid in remaining:
        send(pid, signal.SIGKILL)
    wait_for_exit(remaining, 1.)
    check_ports()
    print(f'Cleared {len(initial)} stale Bullet flight process(es).')
    return len(initial)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    try:
        cleanup(dry_run=args.dry_run)
    except RuntimeError as error:
        parser.error(str(error))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
