# Point and state contract

All point timestamps, relative times, packet spans, IMU times and frame boundaries are seconds on the sensor clock. Livox offset_time and Ouster t are divided by 1e9 at conversion. alpha_time is sampling metadata, never a registration interpolation weight.

| Stage | Coordinates / ownership |
|---|---|
| cloud_convert | raw_point = sensor LiDAR sample, metres; timestamp remains sample time |
| CropAuxPoints | correct aux time with configured offset, crop half-open main interval |
| ProcessMeasurements aux transform | raw_point becomes main LiDAR frame **at the same sampling time** using T_main_aux |
| Undistort / DeskewToEndImu | raw_point and timestamp unchanged; end_point = T_WI(end)^-1 T_WI(sample) T_IL raw_point, IMU coordinates at frame end |
| buildFrame / search / residual | point = R_end end_point + p_end; residual takes end_point directly, no inverse-world reconstruction or alpha interpolation |
| committed registration | refresh world points with the committed posterior; frozen prediction trajectory is not a smoothed trajectory |

IMU coverage failure removes the affected point and logs its count; no default end pose or extrapolation is used. Insufficient remaining support rejects the frame. The two first frames no longer overwrite alpha_time.

R1 removes the CT, iterative and no-compensation runtime paths, unused analytical factors/parameterization and dead alternate factor construction. Old `icpmodel`, `motion_compensation`, `point_to_plane_with_distortion`, `beta_*` settings are rejected with a migration error. Use `point_time_model: IMU_DESKEW_END`.

Verification: `bash test/run_registration_tests.sh`, actual AutoDiff PointToPlaneFunctor plus EigenQuaternionManifold. Right physical rotation perturbations are R Exp(delta); Ceres uses a left half-angle tangent, converted with delta_ceres=R delta/2. World reconstruction maximum error 9.93014e-16 m; local Jacobian difference 2.52344e-10. These are synthetic correctness results, not a real-motion A/B performance conclusion.

R2 state ownership: Ceres only mutates frame-local candidate storage. Successful ESKF correction commits p/R/v/bg/ba to current_state and frame cache, then refreshes all world points before map/ScanContext/cloud products. Rejected frames commit the propagated prior, skip all products/map maintenance, and preserve last_accepted. A map seed uses the valid propagated filter without ObserveSE3; it is logged seed and only supplies the existing freshness anchor. The value snapshot is sent before map work. Candidate/posterior delta is diagnostic; no repeated observation is applied to force agreement.

R6 cleanup removes the no-longer-read all_state_frame history and redundant end-of-frame copies. current_state remains a derived posterior cache, not a third filter.
