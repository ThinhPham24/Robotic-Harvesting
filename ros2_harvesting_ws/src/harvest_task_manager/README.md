# harvest_task_manager

Top-level deterministic harvesting coordinator.

Recommended implementation: BehaviorTree.CPP or a lifecycle-aware state
machine with ROS 2 actions.

Owns target selection, workflow state, retries, recovery, cancellation, and
result recording.
