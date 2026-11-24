// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later
//! # compass
//!
//! A compass with a dial, cardinal points, and a two-tone needle.
//!
//! ## Geometry
//!
//! The angle (`heading`) is measured clockwise from **North** (the top of the
//! screen, where Y decreases).
//!
//! ```text
//!       N  (inner pixel)
//!    ┌──┼──┐
//!  W ┤  ↑  ├ E    ← heading = 0 (needle points North)
//!    └──┼──┘
//!       S
//! ```
//!
//! ## Rendering
//!
//! | Element          | Rendering                                      |
//! |------------------|------------------------------------------------|
//! | Dial             | Unfilled circle (radius = `radius`)            |
//! | Cardinal points  | 1 pixel at r-2 on each axis                    |
//! | North needle     | Thickened segment toward the tip               |
//! | South needle     | Simple segment (tail)                          |
//! | Center           | Filled disk, r=1                               |
//!
//! ## Example
//!
//! ```rust,no_run
//! use hazard3_ssd1306_physics::Compass;
//! use embedded_trig_f32 as trig;
//!
//! let compass = Compass::new(112, 32, 14);
//! compass.draw(&mut gfx, 0.785, true, trig::cos, trig::sin); // NE
//! ```

use hazard3_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;
use crate::draw_utils::{segment, filled_disk, ring};

/// Compass with a dial, cardinal points, and a needle.
#[derive(Clone, Copy, Debug)]
pub struct Compass {
    /// X coordinate of the center.
    pub cx: i32,
    /// Y coordinate of the center.
    pub cy: i32,
    /// Dial radius in pixels.
    pub radius: i32,
}

impl Compass {
    /// Creates a compass.
    #[inline]
    pub fn new(cx: i32, cy: i32, radius: i32) -> Self {
        Self { cx, cy, radius }
    }

    /// Draws the compass.
    ///
    /// `heading`: heading in radians clockwise from North.
    /// `on`: `true` draws, `false` erases.
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        heading: f32,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        let r = self.radius;

        // Dial
        ring(gfx, self.cx, self.cy, r, on);

        // Cardinal points (pixels at r-2)
        let inner = r - 2;
        gfx.pixel(self.cx,         self.cy - inner, on); // N
        gfx.pixel(self.cx + inner, self.cy,         on); // E
        gfx.pixel(self.cx,         self.cy + inner, on); // S
        gfx.pixel(self.cx - inner, self.cy,         on); // W

        let sh = sin_fn(heading);
        let ch = cos_fn(heading);

        // North needle (thickened, up to r-3)
        let needle = r - 3;
        let nx = self.cx + ((needle as f32) * sh + 0.5) as i32;
        let ny = self.cy - ((needle as f32) * ch + 0.5) as i32;

        segment(gfx, self.cx, self.cy, nx, ny, on);
        // Perpendicular thickening
        let dx = (nx - self.cx).abs();
        let dy = (ny - self.cy).abs();
        if dx >= dy {
            segment(gfx, self.cx, self.cy + 1, nx, ny + 1, on);
        } else {
            segment(gfx, self.cx + 1, self.cy, nx + 1, ny, on);
        }

        // South tail (simple, up to r/2-1)
        let tail = (r / 2) - 1;
        let tx = self.cx - ((tail as f32) * sh + 0.5) as i32;
        let ty = self.cy + ((tail as f32) * ch + 0.5) as i32;
        segment(gfx, self.cx, self.cy, tx, ty, on);

        // Center pivot
        filled_disk(gfx, self.cx, self.cy, 1, on);
    }

    /// Erases the compass.
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        heading: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(gfx, heading, false, cos_fn, sin_fn);
    }
}