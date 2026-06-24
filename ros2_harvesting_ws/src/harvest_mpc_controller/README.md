# harvest_mpc_controller

Shadow-mode finite-horizon joint reference generator using a double-integrator
model and acceleration/velocity projection.

This Python node is for simulation, replay, and architecture validation. A
production FANUC deployment should use a measured model, deterministic solver,
C++ or generated C implementation, deadline monitoring, and the safety gate.
