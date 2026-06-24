# fanuc_hardware_interface

Adapter between the selected FANUC communication stack and standard ROS 2
control interfaces.

Responsibilities:

- joint state and robot-status feedback
- trajectory execution and cancellation
- controller fault mapping
- digital/analog I/O bridge
- heartbeat and reconnect behavior

Do not implement harvesting policy or perception here.

Driver selection is intentionally unresolved until the exact FANUC robot,
controller, installed options, and support requirements are known.
