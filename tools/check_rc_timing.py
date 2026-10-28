"""Passively sample default local RC/mission telemetry; never sends commands."""
import json
import time

import zmq


def main():
    topics = (b'flight/drone_1/rc', b'app/drone_1/mission', b'simulation/state')
    latest, commands = {}, []
    with zmq.Context() as context, context.socket(zmq.SUB) as subscriber:
        subscriber.setsockopt(zmq.LINGER, 0)
        for topic in topics:
            subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        for port in (5555, 5558, 5559):
            subscriber.connect(f'tcp://127.0.0.1:{port}')
        end = time.monotonic() + 8
        while time.monotonic() < end:
            if not subscriber.poll(20):
                continue
            parts = subscriber.recv_multipart()
            if len(parts) != 2 or parts[0] not in topics:
                continue
            value = json.loads(parts[1])
            latest[parts[0].decode()] = value
            if parts[0] == topics[0]:
                commands.append((time.monotonic_ns() // 1000, value))
    gaps = [(b[0] - a[0]) / 1000 for a, b in zip(commands, commands[1:])]
    margins = [(value['valid_until_us'] - stamp) / 1000 for stamp, value in commands]
    print(json.dumps(dict(latest=latest, rc_count=len(commands),
                          largest_rc_gaps_ms=sorted(gaps)[-12:],
                          minimum_rc_remaining_ms=min(margins) if margins else None), indent=2))


if __name__ == '__main__':
    main()
