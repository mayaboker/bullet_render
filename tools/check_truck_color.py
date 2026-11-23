"""Capture one frame from the running Isaac renderer and report the truck's HSV range.

Run with the system Python (not python.sh) while the Isaac renderer is streaming:
    uv run --project ../bullet_visual python tools/check_truck_color.py

Reads one frame from SHM via ZMQ, converts to BGR then HSV, and prints the
HSV statistics of pixels that look red-ish. Helps diagnose whether the Isaac
RTX output falls inside the detector's (H 0-10 | 170-180, S 100+, V 80+) range.
"""
import sys
import time

import cv2
import numpy as np
import zmq
from bullet_image_transport.shm import FrameReader, MAX_MESSAGE_BYTES, topic_prefix

ENDPOINT = 'tcp://127.0.0.1:5556'
TOPIC = 'camera/front/image'
TIMEOUT_S = 10


def main():
    reader = FrameReader(TOPIC)
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.MAXMSGSIZE, MAX_MESSAGE_BYTES)
    sock.setsockopt(zmq.SUBSCRIBE, topic_prefix(TOPIC))
    sock.connect(ENDPOINT)

    print(f'Waiting for a frame on {ENDPOINT} (topic={TOPIC})...')
    deadline = time.monotonic() + TIMEOUT_S
    frame = None
    while time.monotonic() < deadline:
        try:
            message = sock.recv(zmq.NOBLOCK)
        except zmq.Again:
            time.sleep(0.05)
            continue
        result = reader.read(message)
        if result is not None:
            frame = result
            break

    reader.close()
    sock.close(linger=0)
    ctx.term()

    if frame is None:
        print('No frame received within timeout.', file=sys.stderr)
        return 1

    metadata, pixels = frame
    print(f'Frame: {metadata["width"]}x{metadata["height"]} encoding={metadata["encoding"]} '
          f'version={metadata["version"]}')

    rows = np.frombuffer(pixels, dtype=np.uint8).reshape(metadata['height'], metadata['step'])
    image = rows[:, :metadata['width'] * 3].reshape(metadata['height'], metadata['width'], 3)
    if metadata['encoding'] == 'rgb8':
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    else:
        bgr = image.copy()

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Find pixels matching the detector's red thresholds
    low_red = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))
    high_red = cv2.inRange(hsv, (170, 100, 80), (180, 255, 255))
    mask = low_red | high_red
    detected_count = np.count_nonzero(mask)

    # Also check a wider red range to see what we're missing
    wide_low = cv2.inRange(hsv, (0, 50, 40), (20, 255, 255))
    wide_high = cv2.inRange(hsv, (160, 50, 40), (180, 255, 255))
    wide_mask = wide_low | wide_high
    wide_count = np.count_nonzero(wide_mask)

    # Report the most saturated reddish pixels
    reddish = hsv[wide_mask > 0]
    total_pixels = metadata['width'] * metadata['height']

    print(f'\nDetector red pixels (H 0-10|170-180, S>=100, V>=80): {detected_count} / {total_pixels}')
    print(f'Wide red pixels    (H 0-20|160-180, S>=50,  V>=40): {wide_count} / {total_pixels}')

    if wide_count > 0:
        print(f'\nWide-red HSV statistics:')
        print(f'  H: min={reddish[:, 0].min()} max={reddish[:, 0].max()} '
              f'mean={reddish[:, 0].mean():.1f} median={np.median(reddish[:, 0]):.0f}')
        print(f'  S: min={reddish[:, 1].min()} max={reddish[:, 1].max()} '
              f'mean={reddish[:, 1].mean():.1f} median={np.median(reddish[:, 1]):.0f}')
        print(f'  V: min={reddish[:, 2].min()} max={reddish[:, 2].max()} '
              f'mean={reddish[:, 2].mean():.1f} median={np.median(reddish[:, 2]):.0f}')
    else:
        # Show overall image stats to understand what colors are present
        print('\nNo reddish pixels found. Full image HSV stats:')
        print(f'  H: min={hsv[:,:,0].min()} max={hsv[:,:,0].max()} mean={hsv[:,:,0].mean():.1f}')
        print(f'  S: min={hsv[:,:,1].min()} max={hsv[:,:,1].max()} mean={hsv[:,:,1].mean():.1f}')
        print(f'  V: min={hsv[:,:,2].min()} max={hsv[:,:,2].max()} mean={hsv[:,:,2].mean():.1f}')

        # Find the brightest non-green region
        non_green = cv2.inRange(hsv, (0, 30, 30), (60, 255, 255))
        non_green_inv = cv2.bitwise_not(non_green)
        bright = cv2.inRange(hsv, (0, 0, 100), (180, 255, 255))
        candidates = cv2.bitwise_and(non_green_inv, bright)
        cand_count = np.count_nonzero(candidates)
        if cand_count > 0:
            cand_hsv = hsv[candidates > 0]
            print(f'\nBright non-green pixels ({cand_count}):')
            print(f'  H: min={cand_hsv[:, 0].min()} max={cand_hsv[:, 0].max()} '
                  f'mean={cand_hsv[:, 0].mean():.1f}')
            print(f'  S: min={cand_hsv[:, 1].min()} max={cand_hsv[:, 1].max()} '
                  f'mean={cand_hsv[:, 1].mean():.1f}')
            print(f'  V: min={cand_hsv[:, 2].min()} max={cand_hsv[:, 2].max()} '
                  f'mean={cand_hsv[:, 2].mean():.1f}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
