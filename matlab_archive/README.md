# MATLAB Archive

Historical MATLAB/Simulink modeling work, kept for provenance. Not part of the active
`flex_simulink` ROS2 codegen pipeline (see `../flex_simulink/`).

- `LQR.m` — 4-state linearized cart-pole model + offline `lqr()` gain synthesis (the source of the
  gains that are hardcoded to zero in `firmware/src/main.cpp`'s `K1lqr..K4lqr`).
- `2D_Simulator/` — variant of the same model (different pendulum mass) with a 2D animation script;
  not structured as a deployable control-law block diagram, so not chosen as the codegen target.
- `Multi-Body_Simulator/` — Simscape Multibody model plus the source SolidWorks part
  (`FLEXmatlab-Corps volumiques.SLDPRT`, 19MB, no STL/STEP export yet) used for physical validation,
  not for controller deployment.
- `call_plot.m`, `essaie.m` — scratch/plotting scripts.
- `slprj/`, `*.mat` — Simulink build caches and recorded test data (`couple.mat`, `force.mat`,
  `output.mat`, `untitled.mat`, `xtheta.mat`). New build caches are excluded going forward via the
  repo's `.gitignore`; these existing ones are retained here for history.

See `docs/architecture/control_architecture.md` for why `PendulumCartSim.slx` (in
`flex_simulink/`) was chosen as the master controller candidate instead of these.
