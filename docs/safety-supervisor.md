# Safety supervisor and command ownership

`ros2 run lunabot_control safety_supervisor` is the sole production publisher to
`/cmd_vel`. Navigation requests arrive on `/cmd_vel_nav`; fresh teleoperation
requests on `/cmd_vel_teleop` take priority. Commands are finite-value checked
and clamped.

The supervisor fails closed for stale localization, traversability/sensor data,
path, or command; an obstacle stop; excessive roll/pitch; invalid commands; and
a latched emergency stop. Decisions are published as typed `SafetyStatus` on
`/lunabot/control/safety_status`.

Latch or clear emergency stop with:

```bash
ros2 service call /lunabot/control/emergency_stop std_srvs/srv/SetBool '{data: true}'
ros2 service call /lunabot/control/emergency_stop std_srvs/srv/SetBool '{data: false}'
```

Timeouts, slope limits, obstacle input integration, and operator-priority policy
must be tuned and fault-injected in the integrated simulator before deployment.
