# Changelog

- Service (PlanMotion) and Action (PlanAndExecute)
- Joint, pose (IK), and Cartesian planning
- OMPL planners with RRTConnect as default
- Time parameterization (IPTP/TOTG)
- Deterministic mode via seeded RNG
- RViz demo + example clients

Fixes and notes:
- Stable initialization (no bad_weak_ptr); safe current state handling
- Updated to non-deprecated MoveIt error codes
- Cleaned legacy nodes and tightened CMake
