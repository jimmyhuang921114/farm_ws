# Livox continuity and GLIM session boundaries

`livox_cloud_guard` observes raw `/livox/lidar`, transformed
`/livox/imu_base`, monotonic receive time, and both sensor header stamps. Its
output `/livox/lidar_valid` is gated by this state machine:

```text
HEALTHY -> DEGRADED -> DISCONNECTED -> RECOVERING -> HEALTHY
```

The defaults are derived from the observed 10 Hz LiDAR and 200 Hz IMU:

- `lidar_expected_period_sec=0.1`
- `imu_expected_period_sec=0.005`
- `lidar_warning_gap_sec=0.3` (never less than three expected periods)
- `imu_warning_gap_sec=0.05` (never less than three expected periods)
- `recoverable_gap_sec=2.0`
- `restart_required_gap_sec=30.0`
- `max_lidar_imu_delta_sec=0.02`
- `recovery_lidar_frames=20`
- `recovery_imu_duration_sec=1.0`

Non-increasing stamps and a large sensor-time jump without a corresponding wall
gap require a new GLIM session immediately. A receive/sensor outage longer than
30 seconds also sets `restart_required`. The value 5690 seconds is not embedded
in the implementation.

During recovery, the guard publishes no clouds until all recovery conditions
pass. Diagnostics are available at `/livox/continuity/diagnostics`; the latched
state and restart flag are `/livox/continuity/state` and
`/livox/continuity/restart_required`.

A warning-sized gap enters `DEGRADED` and is counted, but the next healthy
frame returns directly to `HEALTHY`. The N-frame/IMU/synchronization gate is
reserved for recovery from `DISCONNECTED`; this avoids discarding 20 healthy
frames for ordinary scheduling jitter while retaining the warning evidence.

The one-click launcher sets `CYCLONEDDS_URI` to
`config/cyclonedds_livox.xml`. Loopback has higher priority than the removable
`enp5s0` interface, so guard-to-supervisor control remains available when the
MID-360 Ethernet carrier disappears. Livox sensor UDP is still received
directly by `livox_ros_driver2`.

`glim_session_supervisor.py` owns only the GLIM process group. On a required
boundary it stops GLIM, leaves sensors/UI/manager running, waits for `HEALTHY`,
backs off exponentially, and creates a new directory under:

```text
/tmp/farm_ws_runtime/<run>/glim_sessions/session_<number>_<UTC time>/
```

`session_events.jsonl` distinguishes a sensor discontinuity from an unexpected
GLIM exit and records the last good LiDAR/IMU stamps. Sessions are never merged
or overwritten. The current collection manager is UI-only and does not create
MCAP. If a separate recorder is added, keep the raw bag continuous and use this
event file to split GLIM post-processing; never imply the two trajectories are
continuous.

The supervisor allows three automatic restarts by default. A fourth failure
disables further starts to prevent a crash loop.

## Controlled test

Start the system and wait at least ten minutes:

```bash
/workspace/farm_ws/scripts/run_livox_glim_ui.sh --headless
```

For a short test, disconnect only the MID-360 Ethernet cable for 5–10 seconds,
then reconnect it. Expect `DISCONNECTED`, guarded recovery, and return to the
same session because the default restart threshold is 30 seconds.

For a long test, disconnect it for 60 seconds. Expect the old GLIM session to
end after 30 seconds and a new session to start only after continuity returns to
`HEALTHY`. Preserve the whole runtime directory after each test.
