"""Launcher-owned firmware process and local episode preparation interface."""
from pathlib import Path
import socket
import subprocess
from threading import Event, Thread
import time

import zmq

READY_TIMEOUT_S = 10
STOP_TIMEOUT_S = 3


class SitlSupervisor:
    def __init__(self, binary, profile, session):
        self.binary, self.profile, self.session = Path(binary).resolve(), Path(profile).resolve(), Path(session).resolve()
        self.endpoint = 'ipc://' + str(self.session / 'sitl-control.sock')
        self.stop, self.ready = Event(), Event()
        self.process = self.thread = self.log = None
        self.error = None
        self.generation = 0

    def start(self):
        if not self.binary.is_file() or not self.profile.is_file():
            raise ValueError('Pinned SITL binary/profile missing. Run bullet_drone/tools/setup_betaflight_sitl.sh first.')
        self.session.mkdir(parents=True, exist_ok=True)
        self.work = self.session / 'sitl'
        self.work.mkdir()
        self.log = (self.session / 'sitl.log').open('ab', buffering=0)
        self.thread = Thread(target=self._run, name='sitl-supervisor', daemon=True)
        self.thread.start()
        if not self.ready.wait(READY_TIMEOUT_S) or self.error:
            self.close()
            raise RuntimeError(self.error or 'SITL supervisor did not start')

    def _stop_process(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(STOP_TIMEOUT_S)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
            self.process = None

    def _prepare(self):
        self._stop_process()
        if self.generation == 0:
            for kind, port in ((socket.SOCK_DGRAM, 9003), (socket.SOCK_STREAM, 5761)):
                with socket.socket(socket.AF_INET, kind) as probe:
                    if kind == socket.SOCK_STREAM:
                        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    probe.bind(('127.0.0.1', port))
            subprocess.run([str(self.binary), '--config', str(self.profile)], cwd=self.work,
                           stdout=self.log, stderr=subprocess.STDOUT, timeout=READY_TIMEOUT_S, check=True)
        self.process = subprocess.Popen([str(self.binary), '--ip', '127.0.0.1'], cwd=self.work,
                                        stdout=self.log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + READY_TIMEOUT_S
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError('SITL exited during startup; inspect sitl.log')
            try:
                with socket.create_connection(('127.0.0.1', 5761), timeout=.05):
                    self.generation += 1
                    return self.generation
            except OSError:
                time.sleep(.02)
        raise RuntimeError('SITL MSP port did not become ready; inspect sitl.log')

    def _run(self):
        try:
            with zmq.Context() as context, context.socket(zmq.REP) as server:
                server.setsockopt(zmq.LINGER, 0)
                server.setsockopt(zmq.MAXMSGSIZE, 1024)
                server.bind(self.endpoint)
                self.ready.set()
                while not self.stop.is_set():
                    if not server.poll(50):
                        if self.process is not None and self.process.poll() is not None:
                            raise RuntimeError('SITL exited unexpectedly; inspect sitl.log')
                        continue
                    try:
                        request = server.recv_json()
                        if request != {'action': 'prepare'}:
                            raise ValueError('Unsupported SITL supervisor request')
                        generation = self._prepare()
                        server.send_json(dict(ok=True, generation=generation))
                    except Exception as error:
                        server.send_json(dict(ok=False, error=str(error)))
        except Exception as error:
            self.error = str(error)
            self.ready.set()
        finally:
            self._stop_process()

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(READY_TIMEOUT_S + STOP_TIMEOUT_S)
        if self.log is not None:
            self.log.close()
