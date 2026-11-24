// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later
//! # draw_utils
//!
//! Shared drawing primitives used by all physics object modules.
//!
//! These functions adapt the SSD1306 graphics primitives and avoid repeating
//! imports in each object module.

use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;

/// Draws a line between two points (using Bresenham's algorithm via graphics).
#[inline]
pub fn segment<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    x0: i32, y0: i32,
    x1: i32, y1: i32,
    on: bool,
) {
    embassy_ssd1306_graphics::line(gfx, x0, y0, x1, y1, on);
}

/// Draws a filled disk (joint, mass, or hub).
#[inline]
pub fn filled_disk<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    cx: i32, cy: i32, r: i32,
    on: bool,
) {
    embassy_ssd1306_graphics::fill_circle(gfx, cx, cy, r, on);
}

/// Draws an unfilled circle (dial or rim).
#[inline]
pub fn ring<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    cx: i32, cy: i32, r: i32,
    on: bool,
) {
    embassy_ssd1306_graphics::circle(gfx, cx, cy, r, on);
}

/// Draws a filled rectangle (mass block or gear tooth).
#[inline]
pub fn filled_rect<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    x: i32, y: i32, w: i32, h: i32,
    on: bool,
) {
    // Simple scanline: the driver's draw_filled_rect uses u8, so use pixel()
    // to retain i32 coordinates and clipping.
    for dy in 0..h {
        for dx in 0..w {
            gfx.pixel(x + dx, y + dy, on);
        }
    }
}

/// Calculates the endpoint of a polar vector.
///
/// Starting at `(ox, oy)`, with length `len` and direction `(cos_a, sin_a)`.
#[inline]
pub fn polar_end(ox: i32, oy: i32, len: i32, cos_a: f32, sin_a: f32) -> (i32, i32) {
    let x = ox + ((len as f32) * cos_a + 0.5) as i32;
    let y = oy + ((len as f32) * sin_a + 0.5) as i32;
    (x, y)
}

/// Draws a thickened segment (a line plus a one-pixel offset).
///
/// The offset axis is perpendicular to the dominant direction.
#[inline]
pub fn thick_segment<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    x0: i32, y0: i32,
    x1: i32, y1: i32,
    on: bool,
) {
    segment(gfx, x0, y0, x1, y1, on);
    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    if dx >= dy {
        segment(gfx, x0, y0 + 1, x1, y1 + 1, on);
    } else {
        segment(gfx, x0 + 1, y0, x1 + 1, y1, on);
    }
}