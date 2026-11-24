// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later

//! # piston
//!
//! A clean 2D mechanical piston model (without gas).
//!
//! Simplified physical model:
//! - closed cylinder (chamber)
//! - piston constrained inside the chamber
//! - guided rod (rigid connection)
//!
//! Intended as a foundation for future internal combustion engine / crankshaft visuals.

use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;

use crate::draw_utils::{segment, filled_rect};

/// A 2D piston constrained to move vertically inside a chamber.
#[derive(Clone, Copy, Debug)]
pub struct Piston {
    /// X coordinate of the cylinder center.
    pub cx: i32,
    /// Top of the cylinder.
    pub cy: i32,

    /// Cylinder width.
    pub w: i32,
    /// Total cylinder height.
    pub h: i32,

    /// Piston thickness.
    pub piston_h: i32,

    /// Piston position in the range `[0..h - piston_h]`.
    pub pos: i32,
}

impl Piston {
    /// Creates a piston.
    ///
    /// `cx, cy`: center of the top of the chamber.
    /// `w`: cylinder width.
    /// `h`: chamber height.
    pub fn new(cx: i32, cy: i32, w: i32, h: i32) -> Self {
        Self {
            cx,
            cy,
            w,
            h,
            piston_h: 6,
            pos: 0,
        }
    }

    /// Sets the piston position inside the chamber.
    #[inline]
    pub fn set_pos(&mut self, pos: i32) {
        let max = self.h - self.piston_h;
        self.pos = pos.clamp(0, max);
    }

    /// Returns the normalized physical position (0.0 = top, 1.0 = bottom).
    #[inline]
    pub fn normalized(&self) -> f32 {
        let max = (self.h - self.piston_h).max(1);
        self.pos as f32 / max as f32
    }

    /// Draws the piston and chamber.
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        _t: f32,
        on: bool,
        _cos_fn: fn(f32) -> f32,
        _sin_fn: fn(f32) -> f32,
    ) {
        let left = self.cx - self.w / 2;
        let right = self.cx + self.w / 2;

        let top = self.cy;
        let bottom = self.cy + self.h;

        // ─────────────────────────────
        // CHAMBER (closed rectangle)
        // ─────────────────────────────
        segment(gfx, left, top, right, top, on);
        segment(gfx, left, bottom, right, bottom, on);
        segment(gfx, left, top, left, bottom, on);
        segment(gfx, right, top, right, bottom, on);

        // ─────────────────────────────
        // PISTON (constrained)
        // ─────────────────────────────
        let max = self.h - self.piston_h;
        let y = self.cy + self.pos.clamp(0, max);

        filled_rect(
            gfx,
            left + 2,
            y,
            self.w - 4,
            self.piston_h,
            on,
        );

        // ─────────────────────────────
        // GUIDED ROD (no visual overhang)
        // ─────────────────────────────
        let rod_x = self.cx;
        let rod_top = y + self.piston_h / 2;
        let rod_bottom = bottom;

        segment(gfx, rod_x, rod_top, rod_x, rod_bottom, on);

        // ─────────────────────────────
        // BOTTOM CAP (visual stability)
        // ─────────────────────────────
        segment(gfx, left, bottom, right, bottom, on);
    }

    /// Erases the piston and chamber.
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        t: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(gfx, t, false, cos_fn, sin_fn);
    }
}