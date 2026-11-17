# ZeroMQ IPC versus Zenoh SHM benchmark

The ZeroMQ pipeline works, but these runs do **not establish a meaningful FPS improvement**. Median delivery was 28.96 FPS versus 28.68 FPS (+0.99%); ZeroMQ run-to-run variation was larger than that difference. Publication calls and renderer CPU usage were lower.

## Conditions

Ubuntu 24.04, Godot 4.7.2 OpenGL compatibility, Debug native adapters, NVIDIA RTX 5070 Laptop GPU, driver 580.178.04. Both versions used NVIDIA PRIME offload (`__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`). libzmq 4.3.5 / PyZMQ 27.2.0 versus Zenoh 1.10.1.

Three 60-second recordings per transport, alternating Zenoh then ZeroMQ. Both used the same landscape, 640×480 RGB8, 90° horizontal FOV, 30 Hz simulation-time capture requests, 240 Hz physics, MCAP schemas, uncompressed recording, and no PlotJuggler process. Builds and test suites were stopped during measurement. GPU clocks and other host activity were not controlled.

The timer starts after the first recorded image. The square mission restarts if it completes during the measurement; restart and configuration reacquisition gaps remain in FPS. Recorded counts include startup and the final drain, while FPS uses the timed interval. Each temporary MCAP was independently read back and then deleted. All 5,165 Zenoh images were verified as SHM-backed. ZeroMQ carried all 5,188 image payloads over IPC sockets.

## Results

| Transport | Run | Delivered FPS | Images read back | Poses read back | Peak writer queue (bytes) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Zenoh UDP + SHM | 1 | 28.564 | 1717 | 1915 | 922050 |
| ZeroMQ IPC | 1 | 28.038 | 1686 | 1850 | 922050 |
| Zenoh UDP + SHM | 2 | 28.680 | 1724 | 1854 | 922051 |
| ZeroMQ IPC | 2 | 28.963 | 1740 | 1813 | 922049 |
| Zenoh UDP + SHM | 3 | 28.687 | 1724 | 1824 | 922047 |
| ZeroMQ IPC | 3 | 29.303 | 1762 | 1822 | 922049 |

| Median metric | Zenoh | ZeroMQ IPC |
| --- | ---: | ---: |
| Delivered FPS | 28.68 | 28.96 |
| Capture/readback time (ms) | 11.81 | 15.72 |
| Publication call time (ms) | 0.736 | 0.143 |
| Last observed sequence gaps | 11 | 23 |
| Renderer CPU (% of one core) | 49.4 | 42.1 |
| Physics CPU (% of one core) | 28.1 | 27.2 |
| Recorder + harness CPU (% of one core) | 15.0 | 14.1 |

The publication call averaged about 0.14 ms with ZeroMQ versus 0.74 ms with Zenoh. This is enqueue/allocation/copy time on the Godot thread, **not end-to-end transport latency**. Both transports send asynchronously. Capture timing includes waiting for frame draw, GPU readback, RGB conversion and obtaining pixel bytes; its variation still dominates the publication cost.

Native counters and timing means come from the last periodic snapshot, which may precede shutdown. Sequence gaps omit requests missed before the first capture of a stream and are not a complete loss measurement. PUB/SUB may silently drop messages before recorder acceptance; successful MCAP readback proves the recorded messages are valid, not lossless delivery of all scheduled captures. CPU estimates sample process ticks during the measurement; 100% means one CPU core.

This compares full configurations: Zenoh UDP telemetry plus SHM images against ZeroMQ IPC for both. It is not a pure library-only comparison with identical underlying transports. The 30 Hz source cap and mission restart gaps also limit visible FPS gains.

## Reproduce and inspect

The retained [raw results](benchmark-results.json) contain individual reports, native snapshots, CPU values, versions and the baseline commits. Full logs and camera previews for this session remain under `/tmp/bullet-zmq-bench/{zenoh,zmq}-{1,2,3}/`. Isolated Zenoh sources and the preserved native library remain under `/tmp/bullet-zmq-baseline/`; no baseline branch was edited.

| Baseline repository | Commit |
| --- | --- |
| bullet_render | `6ea1756730b6037cf62f3a6c09602dd204c9ca7d` |
| bullet_drone | `a267ee0aebb56e39050cb31b2f537c45fbff1b9f` |
| mcap_bagger | `3c38d6e138579180f8f2892fc7f85ab6aaee0146` |

The baseline copy has the same temporary Godot timing and harness CPU instrumentation as the experiment. To repeat a run from either prepared renderer directory:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  uv run --with-editable ../mcap_bagger --with pillow \
  python tools/check_pipeline.py --seconds 60 --report-dir /tmp/unique-report-directory
```

Use isolated sibling checkouts/environments for the corresponding physics and recorder commits. On a fresh machine, build the baseline extension from its pinned sources before running. Raw baseline commits already measure FPS and verify SHM; the identical timing and CPU instrumentation used here must be applied if comparing those additional metrics. Do not run the pipelines simultaneously.
