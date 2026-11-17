from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import socket


spec = spec_from_file_location('cleanup_flight', Path(__file__).resolve().parents[1] / 'tools/cleanup_flight.py')
module = module_from_spec(spec)
spec.loader.exec_module(module)


def classify(*arguments, cwd='/home/user/workspaces/bullet/bullet_render'):
    return module.process_kind([value.encode() for value in arguments], cwd)


def test_only_processes_from_this_workspace_are_selected():
    python = '/home/user/workspaces/bullet/.venv/bin/python3'
    assert classify(python, '-m', 'examples.external_rc.main', '--config',
                    '/home/user/workspaces/bullet/bullet_drone/configs/basic.yaml') == 'physics'
    assert classify(python, '-m', 'examples.external_rc.main', '--config',
                    '/home/user/workspaces/bullet/config/basic.yaml') == 'physics'
    assert classify(python, '-m', 'examples.external_rc.main', '--config', '../config/basic.yaml',
                    cwd='/home/user/workspaces/bullet/bullet_drone') == 'physics'
    assert classify(python, '-m', 'mcap_bagger.cli', '--config',
                    '/home/user/workspaces/bullet/bullet_render/configs/recorder.yaml') == 'recorder'
    assert classify('/home/user/workspaces/bullet/.venv/bin/bullet-app', '--transport-config',
                    '/home/user/workspaces/bullet/bullet_render/configs/transport.json') == 'app'
    assert classify('/usr/bin/godot', '--path', '/home/user/workspaces/bullet/bullet_render/godot') == 'godot'
    assert classify('/home/user/workspaces/bullet/bullet_drone/.sitl/betaflight-source/obj/main/betaflight_SITL.elf',
                    '--ip', '127.0.0.1') == 'sitl'
    assert classify(python, 'tools/run_flight.py') == 'launcher'
    assert classify('/home/user/projects/bullet/.venv/bin/python3', '-m', 'examples.external_rc.main',
                    '--config', '/home/user/projects/bullet/bullet_drone/configs/basic.yaml') is None
    assert classify('/usr/bin/python3', '-m', 'mcap_bagger.cli', '--config', '/tmp/recorder.yaml') is None


def test_port_check_ignores_tcp_time_wait():
    listener = socket.socket()
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(('127.0.0.1', 0))
    listener.listen()
    port = listener.getsockname()[1]
    client = socket.create_connection(('127.0.0.1', port))
    server, _ = listener.accept()
    server.close()
    client.close()
    listener.close()

    original = module.TCP_PORTS
    module.TCP_PORTS = (port,)
    try:
        module.check_ports()
    finally:
        module.TCP_PORTS = original
