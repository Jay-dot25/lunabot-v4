# PHASE A VISUAL CHECK

Run `~/launch-a`, open Gazebo, and check each item. **You are the
gatekeeper**: if any item fails, Phase A is not approved.

```
[ ] Large lunar terrain (not a small flat test plane)
[ ] Grey / monochrome Moon appearance
[ ] Dense crater field (many small + medium craters)
[ ] Large visible crater formations (rims, some with central peaks)
[ ] Uneven / elevated terrain (rolling relief, not flat)
[ ] Rough lunar surface (fine rocky texture)
[ ] Lunar habitat building and equipment module are visible
[ ] Colored forward obstacle and side rock landmark are visible
[ ] Forward obstacle has collision geometry and is a real world object
[ ] Rover visibly located on the terrain (spawn pad, near origin)
[ ] Rover looks small relative to the large terrain
[ ] Gazebo camera view clearly shows the terrain
[ ] Does NOT look like a flat Gazebo plane / generic grey floor
[ ] LiDAR gizmo + camera gizmo visible on the sensor mast (optional)
```

RViz check:

```
[ ] TF tree: odom -> chassis -> sensor_head -> {rgb_camera, depth_camera, lidar}
[ ] LaserScan: 360-degree fan of the crater field (red dots)
[ ] Camera panel: live habitat/terrain view
[ ] Depth Camera panel: live depth image
[ ] Driving with WASD moves the rover and the scan/TF follow
```

Compare the Gazebo window against the uploaded reference screenshot:
same *character* (cratered grey lunar surface, large scale, small rover)
— pixel identity is not the acceptance criterion.
