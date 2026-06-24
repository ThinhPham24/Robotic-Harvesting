# Safety Rules

- This gateway is not safety rated.
- RC7M safety circuits, E-stop, teach pendant, limits, and native controller
  logic remain authoritative.
- Initial deployments are telemetry-only.
- UDP telemetry is never used as proof that the robot is safe to move.
- Loss of telemetry makes twin data stale; it does not command a stop unless a
  separate validated controller/PLC safety design implements that behavior.
- Unity, VLA, and external simulation cannot directly write robot motion.
- Future requests must select approved native skills rather than arbitrary code.
- Every command channel must be physically and logically disabled by default.
