# Meshes — intentionally empty

The current URDF uses box/cylinder primitives (see `urdf/flex_base.xacro`, `flex_wheels.xacro`,
`flex_legs.xacro`). No STL/STEP mesh files exist yet because no SolidWorks export has been done —
see `docs/architecture/hardware_architecture.md` ("CAD-to-Sim Pipeline") for the exact path to
generate real meshes from `matlab_archive/Multi-Body_Simulator/FLEXmatlab-Corps volumiques.SLDPRT`
once SolidWorks access is available. When that happens, drop `.stl`/`.dae` files here and update
the relevant xacro `<geometry>` tags from `<box>`/`<cylinder>` to `<mesh filename="...">`
(collision geometry should stay primitive).
