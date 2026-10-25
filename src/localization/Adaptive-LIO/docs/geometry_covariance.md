# Final-match geometry and observation covariance

The final ICP iteration retains exactly its LiDAR correspondences, evaluates them at the solved end pose, and discards previous iterations. Information excludes priors, regularizers and damping. The physical right-tangent Jacobian is [n^T, -n^T R [q]_x]. The squared factor weight and Huber(delta=0.5) nonnegative IRLS weight match the actual Ceres residual. This is approximate geometric information, not calibrated Fisher information.

D=diag(l_ref I,I) defines normalized coordinates. Spectrum, effective rank (max(1e-9,1e-6 lambda_max)), absolute information and trace per point are computed in these coordinates. Eigenvectors are available as weak subspaces and are not independently smoothed. A well distributed three-plane scene must have actual rank 6; normals alone do not establish rank.

The only ObserveSE3 interface accepts a full SPD 6x6 covariance in [world position; Log(R_prior^-1 R_candidate)] coordinates, with m^2, rad^2 and m*rad entries. Both NIS gate and Joseph/reset update validate the same matrix, use factorizations, and preserve cross terms. Invalid input leaves the filter unchanged. The fixed source is 0.01 I (variance, not squared again).

Adaptive covariance uses clamped eigen-direction variances, with zero information assigned the ceiling, and R_x=D R_z D^T. Candidate right-tangent covariance is transported using the exact inverse SO(3) right Jacobian of Log(Exp(phi) Exp(delta)). Hard same-time translation/rotation innovation limits apply independently of R and NIS.

The adaptive source is an uncalibrated engineering option: deskew, map and candidate depend on the prior. Floors, ceilings, residual noise and correlation inflation need independent replay calibration. No absolute accuracy or probabilistic confidence claim is made. Default remains fixed.

R3 verification: r3-core.log and r3-build.log are this run's outputs. Core checks include fixed-update closed-form equivalence, non-diagonal SPD update/gate, atomic rejection, non-small-angle transport (4.12995e-10 numerical error), ranks 3/5/6, Huber scaling and weak-direction update attenuation. Build finished successfully with existing dependency/source warnings.
