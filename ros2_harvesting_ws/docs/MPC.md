# Model Predictive Control Plan

## Initial model

For each joint, use a discrete double-integrator model:

```text
q[k+1]    = q[k] + dt*q_dot[k] + 0.5*dt^2*u[k]
q_dot[k+1]= q_dot[k] + dt*u[k]
```

where `u` is a bounded acceleration reference. This model is appropriate for
testing the ROS integration and constraints, not for claiming accurate FANUC
dynamics.

## Objective

```text
min Σ ||q - q_ref||Q² + ||q_dot - q_dot_ref||V²
      + ||u||R² + ||Δu||S²
```

Subject to:

- joint position bounds
- velocity bounds
- acceleration bounds
- optional Cartesian and collision constraints

## Deployment stages

1. Offline replay against rosbag data.
2. Digital twin only.
3. Shadow mode beside the standard controller; publish recommendations only.
4. Fake hardware/reference adapter.
5. Reduced-speed supervised robot test.
6. Production controller plugin after timing and safety validation.

## Solver options

- Start with NumPy/CasADi for model validation.
- Use acados-generated C or another deterministic QP/NMPC solver for the
  production loop.
- Record solve status, iterations, solve time, prediction error, saturation,
  and fallback use on every cycle.

## Fallback

If state is stale, optimization fails, solve time exceeds its deadline, or any
constraint is invalid:

1. invalidate the MPC output;
2. stop publishing new references;
3. request cancellation/hold through the safety and controller layers;
4. require explicit recovery.
