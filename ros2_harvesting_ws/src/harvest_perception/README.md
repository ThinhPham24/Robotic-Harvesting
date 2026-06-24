# harvest_perception

Perception pipeline for:

- synchronized camera input
- fruit/stem/leaf segmentation
- 3D pose estimation
- target tracking
- confidence and freshness filtering
- obstacle point-cloud publication
- planning-scene updates

Bridge the existing `Scan_app` through files or explicit ROS messages rather
than coupling ROS nodes to the PyQt GUI.
