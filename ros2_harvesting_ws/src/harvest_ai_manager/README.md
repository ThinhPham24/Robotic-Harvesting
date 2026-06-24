# harvest_ai_manager

Optional VLA/LLM integration.

Allowed outputs are structured proposals such as:

- select target ID
- request a new observation
- choose one approved recovery behavior
- change a bounded task preference

The package must not publish joint, velocity, torque, controller, or tool I/O
commands.
