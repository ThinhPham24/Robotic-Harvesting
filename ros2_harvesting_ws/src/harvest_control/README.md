# harvest_control

Deterministic control configuration and tool adapters:

- arm trajectory controller
- gripper/cutter controller
- tool I/O
- force/compliance integration if supported
- execution monitoring

Low-level loops should be C++/controller-side where timing matters.
