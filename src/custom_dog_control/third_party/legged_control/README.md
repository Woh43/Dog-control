# legged_control algorithm sources

The `legged_interface` directory is vendored from
`qiayuanl/legged_control` revision
`a7f381c0367e98e31c01336e678eef47e304d40d`.

It is compiled directly into `custom_dog_control_legged_interface`. The ROS 1
controller, messages, hardware abstraction, launch files and visualization are
not vendored. ROS 2 integration remains in `custom_dog_control`; the optimal
control problem, switched reference manager, swing trajectory planner,
constraints, costs and initializer are based on those pinned algorithm sources.

Local `legged_interface` adaptations are recorded in
`config/legged_control_upstream.sha256`. The reference manager accepts a configured
horizontal **foot-frame center** plane, and swing normal-velocity constraints
optionally restore absolute vertical position. They are loaded from the local
task-file section below (defaults shown):

```text
swing_reference
{
  flatGroundFootCenterHeight  0.0  ; world z in m, configured plane, not a terrain measurement
  positionErrorGain          0.0  ; swing-only 1/s; zero keeps velocity-only tracking
}
```

For this robot, spherical feet of radius 0.026 m on a z=0 plane use a foot-center
plane of 0.026 m, consistent with the flat-ground kinematic estimator. A gain of
20 1/s adds 0.2 m/s upward recovery for a 1 cm vertical tracking deficit:
`vz - vz_ref + gain * (z - z_ref) = 0`. The swing-only gain does not change the
stance `ZeroVelocityConstraint`; keep the separate upstream
`model_settings.positionErrorGain` at its existing zero value for velocity-only
stance. Do not use the upstream shared gain to enable only swing recovery.

The existing planner adds `swingHeight` above this absolute plane at mid-swing.
Its upstream short-swing scaling remains unchanged: 0.10 m is the planned
clearance only when swing duration is at least `swingTimeScale` (currently 0.15 s).
Observed pose re-anchoring does not move the configured plane or the planned
apex. Neither this planned trajectory nor the scheduled contact flag is a
measured foot contact or measured terrain height. Model feasibility, actuator
delay, torque limits and actual clearance still require simulation/robot checks.

All adaptations are compiled from this checked-in overlay. Do not edit the
generated `external/ocs2_ws` tree or the pinned OCS2 `ModelSettings` to configure
these local extensions; upstream commit IDs remain unchanged.

The separately vendored `third_party/legged_wbc` sources come from the same
upstream revision. Solver-success and residual reporting are local safety
instrumentation; the WBC decision variables, constraints and weighted tasks
remain those of upstream `WeightedWbc`.
