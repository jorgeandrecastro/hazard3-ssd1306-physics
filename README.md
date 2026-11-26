# hazard3-ssd1306-physics

[![docs](https://docs.rs/hazard3-ssd1306-physics/badge.svg)](https://docs.rs/hazard3-ssd1306-physics/) 

[![physics-objects](https://img.shields.io/badge/physics--objects-hazard3--ssd1306--physics-blue.svg)](https://crates.io/crates/hazard3-ssd1306-physics)

[![License: MIT OR Apache-2.0](https://img.shields.io/badge/License-MIT%20or%20Apache--2.0-blue.svg)](https://opensource.org/licenses/MIT)

`no_std` 2D drawings of physical objects for 128×64 SSD1306 OLED displays.

This crate provides reusable drawings for common physics and mechanical components in the [Hazard3 project](https://github.com/jorgeandrecastro/Hazard3). It builds on `hazard3-ssd1306-graphics`.

See the [CHANGELOG.md](CHANGELOG.md) for release notes (version 0.4.0).

---

## Available objects

| Module        | Struct(s)                           | Description                                           |
|---------------|-------------------------------------|-------------------------------------------------------|
| `robotic_arm` | `RoboticArm`, `Facing`              | 2D industrial arm (base and pneumatic gripper)        |
| `pendulum`    | `Pendulum`                          | Simple pendulum (mount, rod, and mass)                |
| `piston`      | `Piston`                            | 2D mechanical piston (chamber and guided rod)         |
| `spring_mass` | `SpringMass`                        | Vertical spring-mass system (zigzag spring and block) |
| `compass`     | `Compass`                           | Compass (dial, cardinal points, and needle)            |
| `gear`        | `Gear`, `GearPair`, `GearTrain`     | Individual gears, pairs, and gear trains              |
| `draw_utils`  | *(internal)*                        | Shared drawing primitives (line, arc, disk, etc.)     |

---

## Add the crate to your project

```toml
[dependencies]
hazard3-ssd1306-physics = "0.6"
embedded-trig-f32 = "0.3"
embedded-hal-async = "1.0"
```

For async application setup and display initialization, see the [Hazard3 project](https://github.com/jorgeandrecastro/Hazard3).

---

## Injected sin/cos functions

Each `draw()` method accepts `cos_fn: fn(f32) -> f32` and `sin_fn: fn(f32) -> f32`.
Pass in your preferred `no_std` trigonometry functions: no coupling and no overhead.

```rust
use embedded_trig_f32 as trig;

arm.draw(&mut gfx, 0.785, -0.524, true, trig::cos, trig::sin);
```

---

## Coordinate system and angles

### Screen coordinates

```text
(0,0) ──── X+
  │
  Y+   (Y increases downward, following the standard screen convention)
```

### Angle conventions by object

#### `RoboticArm`: angles measured from the upward vertical

The angle is measured **from the upward vertical** (the −Y axis):

```text
       0°
       ↑
−90° ←   → +90°
```

- `angle_shoulder`: absolute orientation of segment 1
  - `0.0` → segment points straight up
  - positive → tilts toward the `Facing` side
- `angle_elbow`: rotation of segment 2 relative to segment 1
  - `0.0` → segments are aligned (arm fully extended)
  - positive → elbow bends toward the `Facing` side

#### `Pendulum`: angles measured from the downward vertical

The angle is measured **from the downward vertical** (the +Y axis):

```text
  −90° ← 0° → +90°
         ↓
```

- `0.0` → pendulum at rest (rod points down)
- positive → leans to the right
- negative → leans to the left

#### `Piston`: linear position

The piston moves **vertically** (along the Y axis) inside its chamber:

```text
pos = 0     ← top (chamber closed)
  │
  │ chamber
  │
pos = max   ← bottom (piston fully extended)
```

- `pos ∈ [0, h − piston_h]`: constrained position
- `normalized() ∈ [0.0, 1.0]`: normalized position (0 = top, 1 = bottom)

---

## Module structure

```
src/
├── lib.rs           ← declarations and re-exports
├── draw_utils.rs    ← shared line, arc, disk, and rectangle helpers
├── robotic_arm.rs   ← RoboticArm
├── pendulum.rs      ← Pendulum
├── piston.rs        ← Piston
├── spring_mass.rs   ← SpringMass
├── compass.rs       ← Compass
└── gear.rs          ← Gear · GearPair · GearTrain
```

### Adding a new object

1. Create `src/my_object.rs`.
2. Declare `pub mod my_object;` in `src/lib.rs`.
3. Optionally add `pub use my_object::MyObject;` to re-export it.

No other files need to be changed.

---

## Examples

### Robotic arm

A two-segment 2D industrial arm with a pneumatic gripper.
**Angles are measured from the upward vertical** (0 = straight up, positive = tilts toward `Facing`).

**Design:** industrial base (rectangle and hatching), shoulder pivot, segment 1, segment 2, end effector, and jaw gripper.

**Facing:** the `Right` / `Left` enum controls the arm orientation and the direction in which the gripper opens. In the current implementation, `Facing::Right` mirrors the arm horizontally, while `Facing::Left` keeps its local orientation.

#### RoboticArm features

- `Facing` makes it easy to reverse the arm's orientation.
- `with_wall()` customizes the industrial base.
- `with_gripper()` adjusts the gripper size.
- Angles are measured from the upward vertical.

#### Default constructor

```rust
use hazard3_ssd1306_physics::RoboticArm;
use hazard3_ssd1306_physics::Facing;
use embedded_trig_f32 as trig;

// Base at (64, 63), segment 1: 20 px, segment 2: 18 px
let arm = RoboticArm::new(64, 63, 20, 18);

// Shoulder at 45°, elbow at -30° relative to the shoulder, gripper closed (0.0)
arm.draw(&mut gfx, 0.785, -0.524, 0.0, Facing::Right, true, trig::cos, trig::sin);

// The same arm facing left
arm.draw(&mut gfx, 0.785, -0.524, 0.0, Facing::Left, true, trig::cos, trig::sin);
```

#### Custom base and gripper

```rust
use hazard3_ssd1306_physics::{Facing, RoboticArm};
use embedded_trig_f32 as trig;

let arm = RoboticArm::new(32, 60, 18, 16)
    .with_wall(20, 6)           // larger base: 20×6
    .with_gripper(10, 2);       // gripper: 10 px long, 2 px high

arm.draw(&mut gfx, 0.3, 0.4, 0.0, Facing::Right, true, trig::cos, trig::sin);
```

#### Simple animation

```rust
use hazard3_ssd1306_physics::{Facing, RoboticArm};
use embedded_trig_f32 as trig;

let arm = RoboticArm::new(64, 63, 20, 18);
let mut angle_shoulder: f32 = 0.0;
let mut angle_elbow: f32 = 0.0;

loop {
    arm.erase(&mut gfx, angle_shoulder, angle_elbow, 0.0, Facing::Right, trig::cos, trig::sin);

    // Swing the shoulder and elbow
    angle_shoulder += 0.05;
    angle_elbow -= 0.03;

    arm.draw(&mut gfx, angle_shoulder, angle_elbow, 0.0, Facing::Right, true, trig::cos, trig::sin);
    oled.flush().await.unwrap();
}
```

### Pendulum

A simple pendulum consisting of a fixed pivot, rod, and circular mass.
**The angle is measured from the downward vertical** (0 = resting position, positive = leans right).

**Rendering:** wall mount (rectangle and hatching), rod, pivot (r=1), and mass (r=3).

#### Default constructor

```rust
use hazard3_ssd1306_physics::Pendulum;
use embedded_trig_f32 as trig;

// Pivot at (64, 6), 28 px rod, default 16×5 mount
let pend = Pendulum::new(64, 6, 28);
pend.draw(&mut gfx, 0.4, true, trig::cos, trig::sin);   // ~23° to the right
pend.draw(&mut gfx, -0.4, true, trig::cos, trig::sin);  // ~23° to the left
```

#### Custom mount

```rust
use hazard3_ssd1306_physics::Pendulum;
use embedded_trig_f32 as trig;

// Larger mount: 20×8 instead of 16×5
let pend = Pendulum::with_wall(32, 10, 30, 20, 8);
pend.draw(&mut gfx, 0.2, true, trig::cos, trig::sin);
```

#### Simple animation

```rust
let mut angle: f32 = 0.0;
loop {
    pend.erase(&mut gfx, angle, trig::cos, trig::sin);
    angle += 0.05;  // gradually increment the angle
    pend.draw(&mut gfx, angle, true, trig::cos, trig::sin);
    oled.flush().await.unwrap();
}
```

### Spring-mass system

```rust
use hazard3_ssd1306_physics::SpringMass;

let sm = SpringMass::new(64, 4, 22, 12);
// Extend the spring by 6 px
sm.draw(&mut gfx, 6, true);
// Compress the spring by 4 px
sm.draw(&mut gfx, -4, true);

// Customize: 8 half-coils, amplitude ±6 px
let sm2 = SpringMass::new(64, 4, 22, 12).with_coils(8, 6);
sm2.draw(&mut gfx, 0, true);
```

### Compass

```rust
use hazard3_ssd1306_physics::Compass;
use embedded_trig_f32 as trig;

let compass = Compass::new(112, 32, 14);
compass.draw(&mut gfx, 0.785, true, trig::cos, trig::sin); // NE
```

### Piston

```rust
use hazard3_ssd1306_physics::Piston;

let mut piston = Piston::new(64, 10, 20, 40);

// At the top (closed)
piston.set_pos(0);
piston.draw(&mut gfx, 0.0, true, |_| 0.0, |_| 0.0);

// In the middle
piston.set_pos(20);
piston.draw(&mut gfx, 0.0, true, |_| 0.0, |_| 0.0);

// Simple back-and-forth animation
let max_pos = piston.h - piston.piston_h;
let mut pos: i32 = 0;
loop {
    piston.erase(&mut gfx, 0.0, |_| 0.0, |_| 0.0);
    pos = (pos + 1) % (max_pos * 2);
    let actual_pos = if pos > max_pos { max_pos * 2 - pos } else { pos };
    piston.set_pos(actual_pos);
    piston.draw(&mut gfx, 0.0, true, |_| 0.0, |_| 0.0);
    oled.flush().await.unwrap();
}
```

### Single gear

```rust
use hazard3_ssd1306_physics::Gear;
use embedded_trig_f32 as trig;

//                      cx  cy  pitch_r  teeth  tooth_h  tooth_w%  hub_r
let g = Gear::new(      40, 32,    14,     10,      4,      70,      4);
g.draw(&mut gfx, 0.0, true, trig::cos, trig::sin);
```

### Gear pair

```rust
use hazard3_ssd1306_physics::gear::{Gear, GearPair};
use embedded_trig_f32 as trig;

let pair = GearPair::new(
    Gear::new(30, 32, 12, 8, 4, 70, 3),   // driver (actual cx/cy)
    Gear::new(0, 0, 18, 12, 4, 70, 5),    // driven (cx/cy are calculated)
    0.0,                                    // g2 is to the right of g1
    trig::cos, trig::sin,
);

let mut angle: f32 = 0.0;
loop {
    pair.erase(&mut gfx, angle, trig::cos, trig::sin);
    angle += 0.05;
    pair.draw(&mut gfx, angle, true, trig::cos, trig::sin);
    oled.flush().await.unwrap();
}
```

### Three-gear train

```rust
use hazard3_ssd1306_physics::gear::{Gear, GearTrain};
use embedded_trig_f32 as trig;

let train = GearTrain::new(
    Gear::new(18, 32, 10, 8, 3, 70, 2),  // G1: driver
    Gear::new(0, 0,  14, 12, 3, 70, 3),  // G2: intermediate
    Gear::new(0, 0,   8,  6, 3, 70, 2),  // G3: output
    0.0,                                   // horizontal alignment
    trig::cos, trig::sin,
);
train.draw(&mut gfx, 0.2, true, trig::cos, trig::sin);
```

---

## Gear geometry

```
         ┌─┐ ┌─┐ ┌─┐   ← top (pitch_r + tooth_h)
       ──┘ └─┘ └─┘ └──
      /                \
     │    ┌──────┐      │  ← pitch circle (pitch_r)
     │    │  ◎   │      │  ← hub (hub_r)
     │    └──────┘      │
      \                /
       ──┐ ┌─┐ ┌─┐ ┌──
         └─┘ └─┘ └─┘
```

### GearPair / GearTrain transmission ratio

```
ω2 = -ω1 × (teeth1 / teeth2)
ω3 =  ω1 × (teeth1 / teeth2) × (teeth2 / teeth3)   [GearTrain]
```

Phasing is automatic: the teeth mesh cleanly at the point of contact.

---

## Architecture

```
┌─────────────────────────────────────────────────┐
│                Your application                 │
│  RoboticArm / Pendulum / Gear / GearPair …      │
└──────────────────────┬──────────────────────────┘
                       │ &mut Graphics
┌──────────────────────▼──────────────────────────┐
│        hazard3-ssd1306-physics                  │
│   robotic_arm · pendulum · spring_mass          │
│   compass · gear (Gear / GearPair / GearTrain)  │
│                  ↓ draw_utils                   │
│   line · arc · ring · filled disk · rectangle   │
└──────────────────────┬──────────────────────────┘
                       │ line() / circle() / fill_circle()
┌──────────────────────▼──────────────────────────┐
│          hazard3-ssd1306-graphics               │
│   clipping · pixel()                            │
└──────────────────────┬──────────────────────────┘
                       │ draw_pixel()
┌──────────────────────▼──────────────────────────┐
│              SSD1306 driver                      │
│   framebuffer · I2C · flush()                   │
└─────────────────────────────────────────────────┘
```

---

## Suggested layout for 128×64

```
┌────────────────────────────────────────────────────────────────┐
│ RoboticArm (x 0–55) │ GearPair (x 56–95) │ Pendulum │ Compass │
│ base=(20,50)         │ g1=(66,32) r=10    │ (104,6) │ (120,32)│
│ seg1=28 seg2=18      │ g2=(84,32) r=8     │  l=20   │  r=10   │
└────────────────────────────────────────────────────────────────┘
```

---

## Angle conventions

| Object        | 0° reference                 | Positive direction       | Type      |
|---------------|------------------------------|--------------------------|-----------|
| `RoboticArm`  | Upward vertical (Y−)         | Toward `Facing`          | Angular   |
| `Pendulum`    | Downward vertical (Y+)       | Right                    | Angular   |
| `Piston`      | Chamber top (pos=0)          | Downward (Y+)             | Linear    |
| `Compass`     | North (Y−)                   | Clockwise (East)          | Angular   |
| `Gear`        | Horizontal X+                | Clockwise                 | Angular   |
| `SpringMass`  | —                            | Extension toward Y+       | Linear    |

---

## 🛡️ License

Licensed under either of:

    MIT license (http://opensource.org/licenses/MIT)

    Apache License, Version 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

at your option.

## 🦅 About

Developed by Jorge Andre Castro.
