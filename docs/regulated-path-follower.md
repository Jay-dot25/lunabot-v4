# Regulated path follower

`ros2 run lunabot_control regulated_path_follower` consumes the production
D*-Lite path, localization pose, and traversability map. It publishes navigation
requests only on `/cmd_vel_nav`; Phase 14's safety supervisor will be the sole
owner of `/cmd_vel`.

The pure-pursuit core reports cross-track error, heading error, curvature, and
target index. Linear speed is reduced by curvature, local terrain cost
(roughness/uncertainty proxy), and clearance input, then constrained by
acceleration/deceleration limits. Empty, expired, or missing paths and stale
localization produce a zero request. Tracking metrics are JSON on
`/lunabot/control/tracking_metrics`.

Final lookahead, speed, acceleration, terrain scaling, and tracking-error
acceptance thresholds require representative simulator and rover tuning.
