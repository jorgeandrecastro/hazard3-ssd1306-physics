// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later
//! # spring_mass
//!
//! A vertical spring-mass system.
//!
//! The spring is anchored at the top (`anchor_x`, `anchor_y`) and extends
//! downward. The mass is a filled rectangle drawn below the spring.
//!
//! The spring is rendered as an integer-only zigzag (alternating ±amplitude).
//!
//! ## Rendering
//!
//! ```text
//!  ─── anchor (5 px horizontal line)
//!   |   ← 2 px upper straight section
//!  /\/\/\/  ← zigzag (N half-coils, amplitude ±4 px)
//!   |   ← 2 px lower straight section
//!  ┌───────┐ ← mass block (fixed height: 6 px)
//!  └───────┘
//! ```
//!
//! ## Parameters
//!
//! | Field       | Description                                         |
//! |-------------|-----------------------------------------------------|
//! | `anchor_x`  | X coordinate of the anchor center                  |
//! | `anchor_y`  | Y coordinate of the anchor point                   |
//! | `rest_len`  | Spring rest length in pixels                       |
//! | `block_w`   | Mass block width in pixels (centered)              |
//! | `coils`     | Number of half-coils (default: 6)                   |
//! | `amplitude` | Zigzag amplitude in pixels (default: ±4)            |
//!
//! ## Example
//!
//! ```rust,no_run
//! use hazard3_ssd1306_physics::SpringMass;
//!
//! let sm = SpringMass::new(64, 4, 22, 12);
//! sm.draw(&mut gfx, 6, true);   // extend the spring by 6 px
//! sm.draw(&mut gfx, -4, true);  // compress the spring by 4 px
//! ```

use hazard3_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;
use crate::draw_utils::segment;

/// A vertical spring-mass system.
#[derive(Clone, Copy, Debug)]
pub struct SpringMass {
    /// X coordinate of the anchor point (center).
    pub anchor_x: i32,
    /// Y coordinate of the anchor point (top of the spring).
    pub anchor_y: i32,
    /// Rest length in pixels.
    pub rest_len: i32,
    /// Mass block width in pixels.
    pub block_w: i32,
    /// Number of half-coils in the zigzag.
    pub coils: i32,
    /// Zigzag amplitude in pixels.
    pub amplitude: i32,
}

impl SpringMass {
    /// Creates a spring-mass system with 6 coils and an amplitude of 4.
    #[inline]
    pub fn new(anchor_x: i32, anchor_y: i32, rest_len: i32, block_w: i32) -> Self {
        Self { anchor_x, anchor_y, rest_len, block_w, coils: 6, amplitude: 4 }
    }

    /// Creates a system with a custom number of coils and amplitude.
    #[inline]
    pub fn with_coils(mut self, coils: i32, amplitude: i32) -> Self {
        self.coils = coils;
        self.amplitude = amplitude;
        self
    }

    /// Draws the system.
    ///
    /// - `extension`: signed extension in pixels (positive = stretch, negative = compress)
    /// - `on`: `true` draws, `false` erases
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        extension: i32,
        on: bool,
    ) {
        let total_len = (self.rest_len + extension).max(self.coils + 4);

        // Horizontal anchor line
        for dx in -(self.block_w / 2)..=(self.block_w / 2) {
            gfx.pixel(self.anchor_x + dx, self.anchor_y, on);
        }

        // Upper straight section
        let spring_top = self.anchor_y + 2;
        segment(gfx, self.anchor_x, self.anchor_y, self.anchor_x, spring_top, on);

        // Zigzag
        let spring_bot = self.anchor_y + total_len - 2;
        if spring_bot > spring_top {
            let h = spring_bot - spring_top;
            let mut prev_x = self.anchor_x;
            let mut prev_y = spring_top;

            for i in 1..=self.coils {
                let ny = spring_top + h * i / self.coils;
                let nx = if i % 2 == 1 {
                    self.anchor_x + self.amplitude
                } else {
                    self.anchor_x - self.amplitude
                };
                segment(gfx, prev_x, prev_y, nx, ny, on);
                prev_x = nx;
                prev_y = ny;
            }
            // Return to the center
            segment(gfx, prev_x, prev_y, self.anchor_x, spring_bot, on);
        }

        // Lower straight section
        let mass_top = self.anchor_y + total_len;
        segment(gfx, self.anchor_x, spring_bot, self.anchor_x, mass_top, on);

        // Mass block (filled rectangle, 6 px high)
        let block_h = 6i32;
        let half_w = self.block_w / 2;
        for dy in 0..block_h {
            for dx in -half_w..=half_w {
                gfx.pixel(self.anchor_x + dx, mass_top + dy, on);
            }
        }
    }

    /// Erases the system.
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        extension: i32,
    ) {
        self.draw(gfx, extension, false);
    }
}