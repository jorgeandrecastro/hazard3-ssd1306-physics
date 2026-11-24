//! # pendulum
//!
//! A simple pendulum: fixed pivot, rod, and circular mass.
//!
//! ## Geometry
//!
//! The angle is measured **from the downward vertical** (0 = resting position).
//! Positive angles lean to the right.
//!
//! ```text
//!  ▓▓▓▓▓▓▓▓▓  ← mount (filled w×h rectangle with diagonal hatching)
//!  ─────────  ← wall/pivot separator
//!      │       ← rod (angle = 0)
//!     \│/      ← deflected rod (angle > 0)
//!      ●       ← mass (filled disk, r=3)
//! ```
//!
//! ## Rendering
//!
//! | Element       | Rendering                                     |
//! |---------------|-----------------------------------------------|
//! | Mount         | Filled `wall_w × wall_h` rectangle + hatching |
//! | Base line     | Horizontal line below the rectangle           |
//! | Pivot         | Small filled disk, r=1 (rotation axis)        |
//! | Rod           | Simple line                                   |
//! | Mass          | Filled disk, r=3                              |
//!
//! ## Example
//!
//! ```rust,no_run
//! use hazard3_ssd1306_physics::Pendulum;
//! use embedded_trig_f32 as trig;
//!
//! let pend = Pendulum::new(64, 6, 28);
//! pend.draw(&mut gfx, 0.4, true, trig::cos, trig::sin);
//! ```

use hazard3_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;
use crate::draw_utils::{segment, filled_disk, filled_rect};

/// A simple pendulum with a wall mount, pivot, rod, and mass.
#[derive(Clone, Copy, Debug)]
pub struct Pendulum {
    /// X coordinate of the pivot (center of rotation and horizontal center of the mount).
    pub pivot_x: i32,
    /// Y coordinate of the pivot (bottom edge of the mount and rod attachment point).
    pub pivot_y: i32,
    /// Rod length in pixels.
    pub length: i32,
    /// Mount width in pixels (centered on `pivot_x`).
    pub wall_w: i32,
    /// Mount height in pixels.
    pub wall_h: i32,
}

impl Pendulum {
    /// Creates a pendulum with the default mount (`wall_w=16`, `wall_h=5`).
    #[inline]
    pub fn new(pivot_x: i32, pivot_y: i32, length: i32) -> Self {
        Self { pivot_x, pivot_y, length, wall_w: 16, wall_h: 5 }
    }

    /// Creates a pendulum with a custom mount.
    #[inline]
    pub fn with_wall(pivot_x: i32, pivot_y: i32, length: i32, wall_w: i32, wall_h: i32) -> Self {
        Self { pivot_x, pivot_y, length, wall_w, wall_h }
    }

    /// Draws the pendulum.
    ///
    /// - `angle`: angle from the vertical in radians (0 = resting position)
    /// - `on`: `true` draws, `false` erases
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle: f32,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        let hw = self.wall_w / 2;
        let wall_x = self.pivot_x - hw;
        let wall_y = self.pivot_y - self.wall_h;

        // Mount: filled rectangle
        filled_rect(gfx, wall_x, wall_y, self.wall_w, self.wall_h, on);

        // Diagonal hatching ╲ (concrete/wall texture), spaced 4 px apart.
        // Turn off pixels along the diagonals to create the texture.
        let mut d = 0i32;
        while d < self.wall_w + self.wall_h {
            for dx in 0..self.wall_w {
                let dy = d - dx;
                if dy >= 0 && dy < self.wall_h {
                    gfx.pixel(wall_x + dx, wall_y + dy, !on);
                }
            }
            d += 4;
        }

        // Horizontal separator below the mount
        segment(
            gfx,
            self.pivot_x - hw - 1, self.pivot_y,
            self.pivot_x + hw + 1, self.pivot_y,
            on,
        );

        // Rod (sin → X, cos → Y: the pendulum hangs downward)
        let mx = self.pivot_x + ((self.length as f32) * sin_fn(angle) + 0.5) as i32;
        let my = self.pivot_y + ((self.length as f32) * cos_fn(angle) + 0.5) as i32;
        segment(gfx, self.pivot_x, self.pivot_y, mx, my, on);

        // Pivot: small disk, r=1 (visible rotation axis)
        filled_disk(gfx, self.pivot_x, self.pivot_y, 1, on);

        // Mass
        filled_disk(gfx, mx, my, 3, on);
    }

    /// Erases the pendulum.
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(gfx, angle, false, cos_fn, sin_fn);
    }
}