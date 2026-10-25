# IMU quality / replay contract

Ingress converts accelerometer values by common.gnorm, then ImuQualityGate performs the single decision. Gyro thresholds are rad/s; accelerometer thresholds are m/s^2 after conversion. Zero limits mean unknown, with that axis saturation detector disabled. Supply limits from the actual sensor configuration; no MID360 limits are invented.

Each sample retains original validity/saturation/held masks (gyro bits 0..2, accel bits 3..5), reason flags, processed finite input, per-axis process variance scales and accumulated degraded seconds. Both consumers and history receive the same sample. Replay never reruns quality detection using a later reference value.

Default hold_seconds=0: an invalid measurement or detected saturation stops both estimator chains and latches the fault until whole-node restart. Unknown-time packets and minor reorder/duplicates are dropped without moving the watermark. Clock rollback >1 s, raw interval >50 ms, unrecoverable history or queues fault both layers. A short input interruption or LiDAR timeout never synthesizes new timestamps or repeats old TF.

Optional experimental hold: use only the failed axis's last healthy input, within a finite <=50 ms episode; raise that axis's process uncertainty using D Q D. Q in snapshots remains the baseline. Healthy samples must satisfy recovery_samples, and recovery hysteresis also consumes the episode budget. No healthy reference or budget expiration latches a fault. This bounds uncertainty treatment; it does not reconstruct true saturated motion. Default remains OFF pending sensor-specific validation.

Interpolation takes the conservative union of endpoint masks and maximum per-axis noise scales, including a right endpoint at alpha=1. Both layers use identical subdivisions and right-end convention. Initialization skips degraded IMUs. Angular velocity output variance includes current gyro quality scale.

Tests cover single-axis clipping, NaN, reorder, reset, >50 ms raw interval crossing a short LiDAR boundary, default stop, bounded hold and recovery, baseline Q restoration, and a delayed posterior inside a degraded interval replaying to identical state/covariance (1e-12 tolerance).
