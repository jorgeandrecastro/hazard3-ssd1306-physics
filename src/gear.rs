// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later
//! # gear
//!
//! Complete gear (teeth, body, hub, and optional keyway).
//!
//! ## Gear geometry
//!
//! ```text
//!           ┌─┐ ┌─┐ ┌─┐   ← teeth (tooth_h pixels high)
//!         ──┘ └─┘ └─┘ └──
//!        /                \
//!       │   ┌───┐  body    │  ← pitch circle (radius = pitch_r)
//!       │   │ ◎ │  hub     │  ← inner circle (radius = hub_r)
//!       │   └───┘          │
//!        \                /
//!         ──┐ ┌─┐ ┌─┐ ┌──
//!           └─┘ └─┘ └─┘
//! ```
//!
//! ### Main parameters
//!
//! | Field       | Description                                              |
//! |-------------|----------------------------------------------------------|
//! | `cx`, `cy`  | Gear center                                              |
//! | `pitch_r`   | Pitch radius (tooth root), in pixels                     |
//! | `teeth`     | Number of teeth                                          |
//! | `tooth_h`   | Tooth height (addendum), in pixels                       |
//! | `tooth_w`   | Angular tooth width as a percentage of pitch (0–100)     |
//! | `hub_r`     | Inner hub radius (0 = no hollow hub)                     |
//!
//! ### Tooth algorithm
//!
//! For each tooth `i ∈ [0, teeth[`:
//! 1. Calculate the tooth center angle: `θ = angle + i * (TAU / teeth)`.
//! 2. Calculate the four corners of the rectangular tooth in polar coordinates
//!    (addendum = pitch_r + tooth_h, dedendum ≈ pitch_r).
//! 3. Draw the four sides of the tooth rectangle.
//! 4. Draw the body arc between consecutive teeth using a masked `circle()`,
//!    approximated by circle pixels within the angular interval.
//!
//! Coordinates use integer-only fixed-point arithmetic (×1024).
//!
//! ## Meshed gears
//!
//! [`GearPair`] automatically positions the second gear so that it meshes with
//! the first (center distance = `pitch_r1 + pitch_r2`).
//!
//! The second gear rotates in the opposite direction, scaled by the
//! `teeth1 / teeth2` ratio.
//!
//! ## Example
//!
//! ```rust,no_run
//! use hazard3_ssd1306_physics::Gear;
//! use embedded_trig_f32 as trig;
//!
//! // Single gear
//! let g = Gear::new(40, 32, 14, 10, 4, 70, 4);
//! g.draw(&mut gfx, 0.0, true, trig::cos, trig::sin);
//!
//! // Meshed pair
//! use hazard3_ssd1306_physics::gear::GearPair;
//! let pair = GearPair::new(
//!     Gear::new(30, 32, 12, 8, 4, 70, 3),  // driver
//!     Gear::new(0, 0, 18, 12, 4, 70, 5),   // driven (cx/cy are ignored)
//!     0,                                     // contact axis: 0 = right
//!     trig::cos, trig::sin,
//! );
//! pair.draw(&mut gfx, 0.3, true, trig::cos, trig::sin);
//! ```

use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;
use crate::draw_utils::{segment, ring, filled_disk};

// ─────────────────────────────────────────────────────────────────────────────
// Fixed-point constants
// ─────────────────────────────────────────────────────────────────────────────

/// TAU × 1024 (fixed-point scaling avoids floating-point constants in loops).
const TAU_FP: i32 = 6434; // 2π × 1024 ≈ 6433.98 → 6434

// ─────────────────────────────────────────────────────────────────────────────
// Gear
// ─────────────────────────────────────────────────────────────────────────────

/// A complete gear: circular body, rectangular teeth, and hub.
#[derive(Clone, Copy, Debug)]
pub struct Gear {
    /// X coordinate of the center.
    pub cx: i32,
    /// Y coordinate of the center.
    pub cy: i32,
    /// Pitch radius (tooth base) in pixels.
    pub pitch_r: i32,
    /// Number of teeth.
    pub teeth: i32,
    /// Tooth height (addendum) in pixels.
    pub tooth_h: i32,
    /// Tooth width as a percentage of the angular pitch (1–99); 70 means a wide tooth.
    pub tooth_w_pct: i32,
    /// Inner hub radius. 0 = solid, >0 = hollow hub.
    pub hub_r: i32,
}

impl Gear {
    /// Creates a gear.
    ///
    /// # Parameters
    ///
    /// - `cx`, `cy`: center
    /// - `pitch_r`: pitch radius in pixels
    /// - `teeth`: number of teeth (≥ 3)
    /// - `tooth_h`: tooth height in pixels (addendum)
    /// - `tooth_w_pct`: tooth width as a percentage of pitch (30–80 recommended)
    /// - `hub_r`: hub radius (0 = solid disk, 2+ = ring)
    #[inline]
    pub fn new(
        cx: i32, cy: i32,
        pitch_r: i32,
        teeth: i32,
        tooth_h: i32,
        tooth_w_pct: i32,
        hub_r: i32,
    ) -> Self {
        Self { cx, cy, pitch_r, teeth, tooth_h, tooth_w_pct: tooth_w_pct.max(1).min(99), hub_r }
    }

    /// Draws the gear at `angle` (radians).
    ///
    /// `angle`: gear rotation in radians.
    /// `on`: `true` draws, `false` erases.
    /// `cos_fn`, `sin_fn`: trigonometric functions.
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle: f32,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        if self.teeth < 3 { return; }

        let outer_r = self.pitch_r + self.tooth_h;

        // Circular body (circle at the pitch radius)
        ring(gfx, self.cx, self.cy, self.pitch_r, on);

        // Teeth
        // Angular pitch between teeth (in radians; floating point is used only here)
        let step = core::f32::consts::TAU / (self.teeth as f32);
        // Half angular width of a tooth
        let half_tooth_angle = step * (self.tooth_w_pct as f32) / 200.0;

        for i in 0..self.teeth {
            let center_angle = angle + (i as f32) * step;

            // Angles of the left and right tooth edges
            let a_left  = center_angle - half_tooth_angle;
            let a_right = center_angle + half_tooth_angle;

            // Four tooth corners in pixel coordinates
            // Bottom-left and bottom-right (on the pitch circle)
            // Top-left and top-right (on the outer circle)
            let (blx, bly) = polar_px(self.cx, self.cy, self.pitch_r, a_left,  cos_fn, sin_fn);
            let (brx, bry) = polar_px(self.cx, self.cy, self.pitch_r, a_right, cos_fn, sin_fn);
            let (tlx, tly) = polar_px(self.cx, self.cy, outer_r,      a_left,  cos_fn, sin_fn);
            let (trx, try_) = polar_px(self.cx, self.cy, outer_r,     a_right, cos_fn, sin_fn);

            // Left and right sides of the tooth
            segment(gfx, blx, bly, tlx, tly, on);
            segment(gfx, brx, bry, trx, try_, on);
            // Tooth tip
            segment(gfx, tlx, tly, trx, try_, on);

            // Arc between this tooth and the next (on the pitch circle)
            // Approximated by discrete points along the arc
            let a_next_left = center_angle + step - half_tooth_angle;
            draw_arc(gfx, self.cx, self.cy, self.pitch_r,
                     a_right, a_next_left, on, cos_fn, sin_fn);
        }

        // Hub
        if self.hub_r > 0 {
            ring(gfx, self.cx, self.cy, self.hub_r, on);
            // Center point
            gfx.pixel(self.cx, self.cy, on);
        } else {
            // Solid hub: small disk
            filled_disk(gfx, self.cx, self.cy, 2, on);
        }
    }

    /// Erases the gear.
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

    /// Outer radius (tooth tips).
    #[inline]
    pub fn outer_r(&self) -> i32 {
        self.pitch_r + self.tooth_h
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GearPair
// ─────────────────────────────────────────────────────────────────────────────

/// A pair of meshed gears.
///
/// The second gear is automatically positioned to mesh with the first.
/// Its center is placed at a distance of `pitch_r1 + pitch_r2` from the first
/// gear's center, in the `axis_angle` direction.
///
/// Rotation is reversed and scaled according to the gear ratio.
///
/// ## Transmission ratio
///
/// `ω2 = -ω1 × (teeth1 / teeth2)`.
///
/// ## Phasing
///
/// The second gear is automatically offset by `π / teeth2` so that the teeth
/// mesh cleanly at x=0.
///
/// ## Example
///
/// ```rust,no_run
/// use hazard3_ssd1306_physics::gear::{Gear, GearPair};
/// use embedded_trig_f32 as trig;
///
/// let pair = GearPair::new(
///     Gear::new(30, 32, 12, 8, 4, 70, 3),
///     Gear::new(0, 0, 18, 12, 4, 70, 5),
///     0.0,   // horizontal axis (second gear to the right of the first)
///     trig::cos, trig::sin,
/// );
/// pair.draw(&mut gfx, 0.3, true, trig::cos, trig::sin);
/// ```
#[derive(Clone, Copy, Debug)]
pub struct GearPair {
    /// First gear (driver).
    pub g1: Gear,
    /// Second gear (driven); cx/cy are calculated automatically.
    pub g2: Gear,
    /// Angle of the axis connecting the centers (radians).
    pub axis_angle: f32,
}

impl GearPair {
    /// Creates a pair of meshed gears.
    ///
    /// `g1`: driver gear (`cx/cy` define its actual center).
    /// `g2`: driven gear (`cx/cy` are **ignored** and calculated here).
    /// `axis_angle`: contact axis angle (0.0 = g2 is to the right of g1).
    /// `cos_fn`, `sin_fn`: trigonometric functions used to calculate the center.
    pub fn new(
        g1: Gear,
        mut g2: Gear,
        axis_angle: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) -> Self {
        let dist = g1.pitch_r + g2.pitch_r;
        g2.cx = g1.cx + ((dist as f32) * cos_fn(axis_angle) + 0.5) as i32;
        g2.cy = g1.cy + ((dist as f32) * sin_fn(axis_angle) + 0.5) as i32;
        Self { g1, g2, axis_angle }
    }

    /// Draws the gear pair.
    ///
    /// `angle1`: rotation angle of the first gear, in radians.
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle1: f32,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        // Transmission ratio
        let ratio = (self.g1.teeth as f32) / (self.g2.teeth as f32);
        // Phase offset: a g2 tooth aligns with the contact point
        let phase2 = self.axis_angle + core::f32::consts::PI / (self.g2.teeth as f32);
        let angle2 = phase2 - angle1 * ratio;

        self.g1.draw(gfx, angle1, on, cos_fn, sin_fn);
        self.g2.draw(gfx, angle2, on, cos_fn, sin_fn);
    }

    /// Erases the gear pair.
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle1: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(gfx, angle1, false, cos_fn, sin_fn);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GearTrain: chain of N gears
// ─────────────────────────────────────────────────────────────────────────────

/// A line of three gears (driver → intermediate → output).
///
/// Each gear meshes with the next. Centers and rotation angles are calculated
/// automatically.
///
/// ```text
///   G1 ──── G2 ──── G3
/// ```
///
/// ## Example
///
/// ```rust,no_run
/// use hazard3_ssd1306_physics::gear::{Gear, GearTrain};
/// use embedded_trig_f32 as trig;
///
/// let train = GearTrain::new(
///     Gear::new(18, 32, 10, 8, 3, 70, 2),   // G1
///     Gear::new(0, 0,  14, 12, 3, 70, 3),   // G2
///     Gear::new(0, 0,   8,  6, 3, 70, 2),   // G3
///     0.0,   // horizontal alignment
///     trig::cos, trig::sin,
/// );
/// train.draw(&mut gfx, 0.2, true, trig::cos, trig::sin);
/// ```
#[derive(Clone, Copy, Debug)]
pub struct GearTrain {
    /// Driver gear.
    pub g1: Gear,
    /// Intermediate gear.
    pub g2: Gear,
    /// Output gear.
    pub g3: Gear,
    /// Axis angle (0 = horizontal).
    pub axis_angle: f32,
}

impl GearTrain {
    /// Creates a chain of three gears.
    pub fn new(
        g1: Gear,
        mut g2: Gear,
        mut g3: Gear,
        axis_angle: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) -> Self {
        // Position g2 to mesh with g1
        let d12 = g1.pitch_r + g2.pitch_r;
        g2.cx = g1.cx + ((d12 as f32) * cos_fn(axis_angle) + 0.5) as i32;
        g2.cy = g1.cy + ((d12 as f32) * sin_fn(axis_angle) + 0.5) as i32;

        // Position g3 to mesh with g2
        let d23 = g2.pitch_r + g3.pitch_r;
        g3.cx = g2.cx + ((d23 as f32) * cos_fn(axis_angle) + 0.5) as i32;
        g3.cy = g2.cy + ((d23 as f32) * sin_fn(axis_angle) + 0.5) as i32;

        Self { g1, g2, g3, axis_angle }
    }

    /// Draws the gear train.
    ///
    /// - `angle1`: rotation angle of the first gear
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle1: f32,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        let r12 = (self.g1.teeth as f32) / (self.g2.teeth as f32);
        let phase2 = self.axis_angle + core::f32::consts::PI / (self.g2.teeth as f32);
        let angle2 = phase2 - angle1 * r12;

        let r23 = (self.g2.teeth as f32) / (self.g3.teeth as f32);
        let phase3 = self.axis_angle + core::f32::consts::PI / (self.g3.teeth as f32);
        let angle3 = phase3 - angle2 * r23;

        self.g1.draw(gfx, angle1, on, cos_fn, sin_fn);
        self.g2.draw(gfx, angle2, on, cos_fn, sin_fn);
        self.g3.draw(gfx, angle3, on, cos_fn, sin_fn);
    }

    /// Erases the gear train.
    #[inline]
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle1: f32,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(gfx, angle1, false, cos_fn, sin_fn);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Returns the pixel coordinates of a polar point.
#[inline]
fn polar_px(
    cx: i32, cy: i32, r: i32, angle: f32,
    cos_fn: fn(f32) -> f32,
    sin_fn: fn(f32) -> f32,
) -> (i32, i32) {
    let x = cx + ((r as f32) * cos_fn(angle) + 0.5) as i32;
    let y = cy + ((r as f32) * sin_fn(angle) + 0.5) as i32;
    (x, y)
}

/// Draws a circular arc using angular interpolation.
///
/// Draws discrete pixels on the circle of radius `r` between `a_start` and
/// `a_end`. The number of steps is proportional to the covered angle × radius
/// to ensure contiguous pixels.
fn draw_arc<I: I2c>(
    gfx: &mut Graphics<'_, I>,
    cx: i32, cy: i32, r: i32,
    a_start: f32, a_end: f32,
    on: bool,
    cos_fn: fn(f32) -> f32,
    sin_fn: fn(f32) -> f32,
) {
    // Number of steps: at least one per covered circumference pixel
    let span = a_end - a_start;
    if span <= 0.0 { return; }

    // Total circumference / angular step → approximately one pixel per arc step
    let steps = ((r as f32) * span + 1.5) as i32;
    if steps <= 0 { return; }

    for i in 0..=steps {
        let a = a_start + span * (i as f32) / (steps as f32);
        let x = cx + ((r as f32) * cos_fn(a) + 0.5) as i32;
        let y = cy + ((r as f32) * sin_fn(a) + 0.5) as i32;
        gfx.pixel(x, y, on);
    }
}

// Suppress the unused warning: TAU_FP is retained for future extensions.
#[allow(dead_code)]
const _: i32 = TAU_FP;