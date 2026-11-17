import os
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import zmq

spec = spec_from_file_location('sitl_supervisor', Path(__file__).resolve().parents[1] / 'tools/sitl_supervisor.py')
module = module_from_spec(spec)
spec.loader.exec_module(module)
SitlSupervisor = module.SitlSupervisor


FAKE_SITL = '''#!/usr/bin/env python3
import socket, sys, time
if '--config' in sys.argv:
    raise SystemExit(0)
server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('127.0.0.1', 5761))
server.listen()
while True:
    client, _ = server.accept()
    client.close()
'''


def request(endpoint):
    with zmq.Context() as context, context.socket(zmq.REQ) as socket:
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVTIMEO, 2000)
        socket.connect(endpoint)
        socket.send_json({'action': 'prepare'})
        return socket.recv_json()


def test_supervisor_provisions_and_restarts_one_firmware_process(tmp_path):
    binary = tmp_path / 'fake-sitl'
    binary.write_text(FAKE_SITL)
    os.chmod(binary, 0o755)
    profile = tmp_path / 'profile.config'
    profile.write_text('save\n')
    supervisor = SitlSupervisor(binary, profile, tmp_path / 'session')
    try:
        supervisor.start()
        assert request(supervisor.endpoint) == {'ok': True, 'generation': 1}
        first_pid = supervisor.process.pid
        assert request(supervisor.endpoint) == {'ok': True, 'generation': 2}
        assert supervisor.process.pid != first_pid
    finally:
        supervisor.close()
    assert supervisor.process is None
