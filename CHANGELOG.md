# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [0.3.1] — 2026-05-21

### Fixed

- **`robotic_arm` module**: Fixed the `Facing` enum.
  - Corrected orientations: `Right` / `Left` / `RightFlipped` / `LeftFlipped`
  - Orientation mapping: right-right-left-left

---

## [0.3.0] — 2026-05-18

### Added

- **`piston` module**: Added the `Piston` struct for 2D mechanical animation.
  - Closed cylinder chamber with a constrained piston
  - Guided rod (rigid connection)
  - Configurable piston dimensions and position
  - Foundation for future internal combustion engine / crankshaft visuals
  - Example: `Piston::new(64, 10, 20, 40)` with `set_pos()` for vertical movement

- **`robotic_arm` module**: Expanded documentation and API.
  - Full assembly: industrial base, shoulder pivot, two segments, end effector, and pneumatic gripper
  - `Facing` enum controls orientation and opening direction (`Right` / `Left`)
  - Clarified angle convention: angles are measured from the upward vertical
  - Customization methods: `with_wall()` (base) and `with_gripper()` (gripper)
  - Detailed constructor, customization, and animation loop examples
  - Added `Facing` (note: `Left` points the gripper right, while `Right` points it left), `with_wall()`, and `with_gripper()`

### Changed

- **`pendulum` documentation**: Significantly improved clarity and completeness.
  - Detailed geometry explanation: angle measured from the downward vertical (0 = resting position)
  - Clarified rendering: mount (rectangle with diagonal hatching), rod, pivot (r=1), and mass (r=3)
  - Expanded examples:
    - Default constructor with positive and negative angles
    - Custom mount using `with_wall()`
    - Simple animation loop using `erase()` and `draw()`
  - Corrected the description in the "Available objects" table

---

## [0.1.0] — Initial release

- `RoboticArm`: Two-segment robotic arm (shoulder + elbow)
- `Pendulum`: Simple pendulum (fixed pivot + rod + mass)
- `SpringMass`: Vertical spring-mass system
- `Compass`: Compass rose with cardinal points
- `Gear`, `GearPair`, `GearTrain`: Synchronized gear animations
- `draw_utils`: Shared primitives (line, arc, disk, rectangle)
