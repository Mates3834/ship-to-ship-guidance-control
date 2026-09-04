# Generic Autonomous Vehicle GNC Examples

This repository contains **generic, sanitized MATLAB examples** for educational
use in autonomous-vehicle guidance, control, planning, and state estimation.

Included modules:

- Grid-based A* path planning
- Minimum-jerk / smooth polynomial trajectory interpolation
- Line-of-sight waypoint guidance
- LQR state-feedback design
- LQI tracking-control design
- Simple MRAC example
- Luenberger observer design
- Discrete Kalman filter
- Small non-weapon autonomous-vehicle demo

> Note: These examples are intentionally generic. They do not include
> system-specific targeting, interception, weapon, or operational parameters.

## Structure

```text
src/
├── planning/
├── trajectory/
├── guidance/
├── control/
└── estimation/
examples/
```
