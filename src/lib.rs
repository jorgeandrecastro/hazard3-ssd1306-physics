// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

//! # hazard3-ssd1306-physics
//!
//! `no_std` 2D drawings of physical objects for 128×64 SSD1306 OLED displays,
//! built on top of an SSD1306 graphics layer.
//!
//! This crate is designed for async embedded applications, including the
//! [Hazard3 project](https://github.com/jorgeandrecastro/Hazard3).
//!
//! ## Available objects
//!
//! | Module          | Struct         | Description                                      |
//! |-----------------|----------------|--------------------------------------------------|
//! | [`robotic_arm`] | [`RoboticArm`] | 2D industrial arm (base + pneumatic gripper)     |
//! | [`pendulum`]    | [`Pendulum`]   | Simple pendulum (mount + rod + mass)             |
//! | [`spring_mass`] | [`SpringMass`] | Vertical spring-mass system                      |
//! | [`compass`]     | [`Compass`]    | Compass (dial + cardinal points + needle)        |
//! | [`gear`]        | [`Gear`]       | Gear (teeth + hollow hub)                        |
//! | [`piston`]      | [`Piston`]     | 2D mechanical piston (chamber + guided rod)      |
//!
//! ## Injected sin/cos functions
//!
//! Each `draw()` method accepts `cos_fn` and `sin_fn` of type `fn(f32) -> f32`.
//! Pass in your preferred `no_std` trigonometry functions:
//!
//! ```rust,no_run
//! use embedded_trig_f32 as trig;
//! arm.draw(&mut gfx, 0.785, -0.524, true, trig::cos, trig::sin);
//! ```
//!
//! ## Adding a new object
//!
//! 1. Create `src/my_object.rs` with the struct implementation, `draw()`, and `erase()`.
//! 2. Declare `pub mod my_object;` in `src/lib.rs`.
//! 3. Optionally add `pub use my_object::MyObject;` to re-export the type.
//!
//! No other files need to be changed.

pub mod draw_utils;
pub mod robotic_arm;
pub mod pendulum;
pub mod spring_mass;
pub mod compass;
pub mod gear;
pub mod piston;


pub use robotic_arm::RoboticArm;
pub use pendulum::Pendulum;
pub use spring_mass::SpringMass;
pub use compass::Compass;
pub use gear::Gear;
pub use gear::GearPair;
pub use gear::GearTrain;
pub use piston::Piston;
pub use robotic_arm::Facing;