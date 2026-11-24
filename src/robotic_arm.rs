// Copyright (C) 2026 Jorge Andre Castro
// GPL-2.0-or-later

//! # RoboticArm: 2D industrial arm
//!
//! This module implements a simplified planar robotic arm for OLED displays
//! and `no_std` embedded simulation.
//!
//! ## Mechanical design
//!
//! The system consists of:
//! - an **industrial base** fixed to the floor (hatched rectangle)
//! - a **shoulder joint** (fixed pivot just above the base)
//! - a **first segment** (shoulder → elbow)
//! - a **second segment** (elbow → end effector)
//! - an industrial **parallel-jaw end gripper** (pneumatic type)
//!
//! ## Coordinate system
//!
//! ```text
//! (0,0) ──── X+
//!   │
//!   Y+   (Y increases downward, following screen convention)
//! ```
//!
//! The arm is **anchored at the bottom** (`base_y` = floor level) and extends
//! **upward**. Orientation is controlled by [`Facing`], applied as a **global
//! transformation** (2D reflection) rather than as an X-only adjustment.
//!
//! ## Visual model (`facing = Right`, zero angles)
//!
//! ```text
//!      ════╗  ← upper jaw
//!        ●   ← end effector
//!       /
//!      ●     ← elbow
//!       \
//!        ●   ← shoulder
//!        │
//!    ██████  ← industrial base
//! ──────────  floor
//! ```
//!
//! ## Angle convention (local space, before reflection)
//!
//! Angles are measured from the **upward vertical** (screen's −Y axis):
//!
//! ```text
//!        0°
//!        ↑
//! −90° ←   → +90°
//! ```
//!
//! - `angle_shoulder`: absolute orientation of segment 1 from the vertical.
//!   - `0.0` → segment points straight up
//!   - positive → tilts right in local space (before reflection)
//! - `angle_elbow`: **relative** rotation of segment 2 from segment 1.
//!   - `0.0` → segments are aligned (arm fully extended)
//!   - positive → elbow bends right in local space
//!
//! ## Mathematical model: Facing as a global transformation
//!
//! All directions are first calculated in **local space** (`Facing = Right`),
//! then transformed by [`Transform2D`] before conversion to pixels.
//!
//! ```text
//! local space  →  Transform2D::apply()  →  screen space
//!
//! Right: reflection across X (sx=−1)
//! Left:  identity             (sx=+1)
//! ```
//!
//! The perpendicular vector (`perp`) is derived **after** the transformation,
//! ensuring that the gripper geometry is consistent in both orientations.
//!
//! ## Internal dependencies
//!
//! This module exclusively uses the primitives in [`crate::draw_utils`]:
//! [`segment`], [`thick_segment`], [`filled_disk`], and [`filled_rect`].
//! It does not import the graphics layer directly.
//!
//! ## Physical properties
//!
//! This is a purely geometric model (no inertia or dynamics), designed for
//! real-time rendering on a 128×64 SSD1306 display.

use crate::draw_utils::{filled_disk, filled_rect, segment, thick_segment};
use embassy_ssd1306_graphics::Graphics;
use embedded_hal_async::i2c::I2c;

// ─────────────────────────────────────────────────────────────────────────────
// Transform2D: minimal 2D affine transformation (rotation + reflection)
// ─────────────────────────────────────────────────────────────────────────────

/// A minimal 2D affine transformation for screen space.
///
/// Represents a 2×2 matrix applied to direction vectors `(vx, vy)`.
/// There is no translation; it is handled explicitly by segment origins.
///
/// # Matrix
///
/// ```text
/// | sx  shx | × | vx |
/// | shy  sy | × | vy |
/// ```
///
/// For [`Facing::Right`]: X reflection `(sx=−1, sy=1, shx=0, shy=0)`.
/// For [`Facing::Left`]: identity `(sx=1, sy=1, shx=0, shy=0)`.
///
/// # Why not just use `sign`?
///
/// Applying a sign to X alone is insufficient when the perpendicular is
/// calculated from the transformed vector: `perp = (−dir_y, dir_x)` remains
/// consistent only if `dir` is already in global space.
/// [`Transform2D`] ensures that **all** directions pass through the same
/// transformation before use.
#[derive(Clone, Copy, Debug)]
 pub struct Transform2D {
    /// X scale factor (±1.0 for reflection, 1.0 for identity).
     sx: f32,
    /// Y scale factor (always 1.0 in this model).
     sy: f32,
    /// X-to-Y shear (0.0 in this model).
     shx: f32,
    /// Y-to-X shear (0.0 in this model).
     shy: f32,
}

impl Transform2D {
    /// Identity transformation (`Facing::Left`).
    #[inline]
    pub const fn identity() -> Self {
        Self { sx: 1.0, sy: 1.0, shx: 0.0, shy: 0.0 }
    }

    /// Reflection across the X axis (`Facing::Right`).
    ///
    /// Reverses the X component of every transformed vector, reflecting the
    /// entire arm (segments and gripper) consistently.
    #[inline]
    pub const fn mirror_x() -> Self {
        Self { sx: -1.0, sy: 1.0, shx: 0.0, shy: 0.0 }
    }

    /// Applies the transformation to a direction vector `(vx, vy)`.
    ///
    /// Returns `(vx', vy')` in screen space.
    ///
    /// ```text
    /// vx' = sx × vx + shx × vy
    /// vy' = shy × vx + sy  × vy
    /// ```
    #[inline]
    fn apply(self, vx: f32, vy: f32) -> (f32, f32) {
        (
            self.sx * vx + self.shx * vy,
            self.shy * vx + self.sy * vy,
        )
    }

    /// Calculates a segment endpoint from an origin using this transformation.
    ///
    /// `(ox, oy)`: origin in pixels.
    /// `(lx, ly)`: local (untransformed) direction.
    /// `len`: segment length in pixels.
    ///
    /// Returns the endpoint `(x, y)` in pixels.
    #[inline]
    fn endpoint(self, ox: i32, oy: i32, lx: f32, ly: f32, len: i32) -> (i32, i32) {
        let (dx, dy) = self.apply(lx, ly);
        (
            ox + (dx * len as f32 + 0.5) as i32,
            oy + (dy * len as f32 + 0.5) as i32,
        )
    }

    /// Calculates the **perpendicular** to a global vector `(gx, gy)`.
    ///
    /// The vector `(gx, gy)` must already be in screen space (post-transformation).
    /// Its perpendicular is `(−gy, gx)`, consistent for either [`Facing`].
    ///
    /// This is the key fix compared with the previous version: `perp` is always
    /// calculated **after** [`Transform2D::apply`], never before.
    #[inline]
    fn perp(gx: f32, gy: f32) -> (f32, f32) {
        (-gy, gx)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Facing
// ─────────────────────────────────────────────────────────────────────────────

/// Arm orientation, which determines the global transformation applied when rendering.
///
/// [`Facing`] is not merely a `sign()` applied to X; it produces a complete
/// [`Transform2D`] that keeps all vectors geometrically consistent
/// (segments, perpendiculars, and gripper).
///
/// # Example
///
/// ```rust
/// let t = Facing::Left.transform();
/// let (gx, gy) = t.apply(0.5, -0.866); // local direction → screen space
/// let (px, py) = Transform2D::perp(gx, gy); // correct perpendicular
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Facing {
    /// Arm facing right — reflection across the X axis.
    Right,
    /// Arm facing left — identity transformation.
    Left,
}


impl Facing {

    /// Returns the 2D transformation associated with this orientation.
    /// `Right` → [`Transform2D::mirror_x`], `Left` → [`Transform2D::identity`].
    #[inline]
    pub fn transform(self) -> Transform2D {
        match self {
            Facing::Right => Transform2D::mirror_x(),
            Facing::Left  => Transform2D::identity(),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// RoboticArm
// ─────────────────────────────────────────────────────────────────────────────

/// A two-segment 2D industrial robotic arm with a pneumatic gripper.
///
/// All dimensions are in **screen pixels** (`i32` integers).
/// The base position is defined by (`base_x`, `base_y`), where `base_y`
/// is the **floor level** (bottom of the base).
///
/// Rendering is mathematically consistent in both orientations:
/// [`Facing`] acts as a global transformation via [`Transform2D`],
/// rather than as a one-off adjustment to the X component.
///
/// # Construction
///
/// ```rust
/// let arm = RoboticArm::new(64, 63, 20, 18)
///     .with_wall(24, 8)
///     .with_gripper(12, 2);
/// ```
///
/// # Rendering
///
/// ```rust
/// arm.draw(
///     &mut gfx,
///     0.3,           // angle_shoulder (rad, local space)
///     -0.5,          // angle_elbow (rad, relative)
///     0.0,           // gripper closed
///     Facing::Right,
///     true,          // on = draw
///     libm::cosf,
///     libm::sinf,
/// );
/// ```
#[derive(Clone, Copy, Debug)]
pub struct RoboticArm {
    /// Horizontal center of the base and shoulder (pixels).
    pub base_x: i32,

    /// Floor level: bottom of the industrial base (pixels; Y increases downward).
    pub base_y: i32,

    /// Segment 1 length: shoulder → elbow (pixels).
    pub seg1_len: i32,

    /// Segment 2 length: elbow → end effector (pixels).
    pub seg2_len: i32,

    /// Industrial base width (pixels).
    pub wall_w: i32,

    /// Industrial base height (pixels).
    pub wall_h: i32,

    /// Length of the gripper jaw rods, along segment 2 (pixels).
    pub gripper_len: i32,

    /// Minimum half-gap between the jaws when the gripper is closed (pixels).
    ///
    /// Corresponds to the mechanical thickness of one jaw and is also used
    /// as the size of the gripping tips.
    pub gripper_thickness: i32,
}

impl RoboticArm {
    /// Creates an arm with the default base and gripper dimensions.
    ///
    /// Defaults:
    /// - base: 20 × 6 px
    /// - gripper: 10 px long, 2 px thick
    ///
    /// # Parameters
    ///
    /// | Parameter  | Description                            |
    /// |------------|----------------------------------------|
    /// | `base_x`   | Horizontal center of the robot (px)    |
    /// | `base_y`   | Floor level, bottom of the base (px)   |
    /// | `seg1_len` | Shoulder → elbow length (px)            |
    /// | `seg2_len` | Elbow → end effector length (px)        |
    pub fn new(base_x: i32, base_y: i32, seg1_len: i32, seg2_len: i32) -> Self {
        Self {
            base_x,
            base_y,
            seg1_len,
            seg2_len,
            wall_w: 20,
            wall_h: 6,
            gripper_len: 10,
            gripper_thickness: 2,
        }
    }

    /// Customizes the industrial base geometry.
    ///
    /// The base is rendered as a filled rectangle with diagonal hatching
    /// (4 px spacing) to simulate a metal surface.
    ///
    /// # Parameters
    ///
    /// - `wall_w`: total base width (px)
    /// - `wall_h`: total base height (px); extends upward from `base_y`
    pub fn with_wall(mut self, wall_w: i32, wall_h: i32) -> Self {
        self.wall_w = wall_w;
        self.wall_h = wall_h;
        self
    }

    /// Customizes the geometry of the industrial gripper jaws.
    ///
    /// The gripper is a **pneumatic parallel-translation** type: both jaws
    /// slide perpendicular to segment 2's axis after the global
    /// [`Transform2D`] is applied.
    ///
    /// # Parameters
    ///
    /// - `gripper_len`: length of each rod along segment 2 (px)
    /// - `gripper_thickness`: minimum half-gap (closed gripper) and tip size (px)
    pub fn with_gripper(mut self, gripper_len: i32, gripper_thickness: i32) -> Self {
        self.gripper_len = gripper_len;
        self.gripper_thickness = gripper_thickness;
        self
    }

    // ─────────────────────────────────────────────────────────────────────────
    // draw
    // ─────────────────────────────────────────────────────────────────────────

    /// Draws the complete robotic arm in its current state.
    ///
    /// # Rendering algorithm (call order)
    ///
    /// 1. **Base** — filled rectangle, diagonal hatching, and floor line
    /// 2. **Forward kinematics** — local directions transformed by [`Transform2D`]
    /// 3. **Thick segments** — shoulder→elbow and elbow→end effector via [`thick_segment`]
    /// 4. **Joints** — filled disks at the three pivots via [`filled_disk`]
    /// 5. **Gripper** — body, rail, rods, and tips via [`segment`]
    ///
    /// # Forward kinematics
    ///
    /// Directions are first expressed in **local space** (`Facing = Right`),
    /// then transformed into screen space using `t = facing.transform()`.
    ///
    /// ```text
    /// local:  lx = sin(a), ly = −cos(a)
    /// screen: (gx, gy) = t.apply(lx, ly)
    /// ```
    ///
    /// The perpendicular is calculated **after** the transformation:
    ///
    /// ```text
    /// (perp_x, perp_y) = Transform2D::perp(gx, gy) = (−gy, gx)
    /// ```
    ///
    /// This is the key fix compared with the previous version: the old
    /// perpendicular `(−dir2_y, dir2_x)` was inconsistent for `Facing::Left`
    /// because `dir2` was not yet in global space.
    ///
    /// # Gripper geometry
    ///
    /// ```text
    ///   end ──── guide      (body_len = 3 px along dir2_g)
    ///             │
    ///         ╠═════╣       (translation rail, perpendicular axis)
    ///         ║     ║
    ///        jaw1  jaw2     (rods parallel to dir2_g)
    ///        [▓]   [▓]      (tips perpendicular to dir2_g)
    /// ```
    ///
    /// Linear interpolation of the jaw gap:
    ///
    /// ```text
    /// half_gap = half_gap_closed + (half_gap_max − half_gap_closed) × open
    /// ```
    ///
    /// # Parameters
    ///
    /// | Parameter          | Type                | Description                              |
    /// |--------------------|---------------------|------------------------------------------|
    /// | `gfx`              | `&mut Graphics`     | SSD1306 rendering context                |
    /// | `angle_shoulder`   | `f32`               | Shoulder angle from vertical (rad)        |
    /// | `angle_elbow`      | `f32`               | Relative elbow rotation (rad)             |
    /// | `gripper_opening`  | `f32`               | Opening \[0.0 closed … 1.0 fully open\]   |
    /// | `facing`           | [`Facing`]          | Left/right orientation                    |
    /// | `on`               | `bool`              | `true` = draw, `false` = erase            |
    /// | `cos_fn`           | `fn(f32) -> f32`    | `no_std` cosine (e.g. `libm::cosf`)       |
    /// | `sin_fn`           | `fn(f32) -> f32`    | `no_std` sine (e.g. `libm::sinf`)         |
    pub fn draw<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle_shoulder: f32,
        angle_elbow: f32,
        gripper_opening: f32,
        facing: Facing,
        on: bool,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        // Global transformation: identity (Left) or X reflection (Right).
        // All directions pass through `t` before use.
        let t = facing.transform();

        // ── 1. BASE ──────────────────────────────────────────────────────────
        //
        // The base is unaffected by reflection: it is symmetric by design
        // and centered on base_x.
        // sy = base_y − wall_h → top of the base = shoulder joint.

        let half = self.wall_w / 2;
        let sx   = self.base_x - half;
        let sy   = self.base_y - self.wall_h;

        // Filled block using filled_rect (replaces the double pixel loop).
        filled_rect(gfx, sx, sy, self.wall_w, self.wall_h, on);

        // Diagonal hatching (4 px spacing), inverted for a metallic effect.
        let mut d = 0;
        while d < self.wall_w + self.wall_h {
            for dx in 0..self.wall_w {
                let dy = d - dx;
                if dy >= 0 && dy < self.wall_h {
                    gfx.pixel(sx + dx, sy + dy, !on);
                }
            }
            d += 4;
        }

        // Floor line, slightly wider than the base (±4 px).
        segment(gfx, sx - 4, self.base_y, sx + self.wall_w + 4, self.base_y, on);

        // ── 2. FORWARD KINEMATICS ────────────────────────────────────────────
        //
        // Step 1: local direction of segment 1 (Right space, vertical = 0°).
        //   lx = sin(a), ly = −cos(a)
        // Step 2: transform into screen space using t.apply().

        let shoulder_x = self.base_x;
        let shoulder_y = sy;

        // Segment 1 — local direction, then global direction.
        let (l1x, l1y) = (sin_fn(angle_shoulder), -cos_fn(angle_shoulder));
        let (g1x, g1y) = t.apply(l1x, l1y);
        let (elbow_x, elbow_y) = t.endpoint(shoulder_x, shoulder_y, l1x, l1y, self.seg1_len);

        // Segment 2 — absolute angle is the sum of both rotations (local space).
        let abs_angle = angle_shoulder + angle_elbow;
        let (l2x, l2y) = (sin_fn(abs_angle), -cos_fn(abs_angle));
        let (g2x, g2y) = t.apply(l2x, l2y);
        let (end_x, end_y) = t.endpoint(elbow_x, elbow_y, l2x, l2y, self.seg2_len);

        // Perpendicular to dir2 in global space, calculated AFTER transformation.
        // This is the key mathematical fix: perp must be derived from the
        // already-transformed vector (g2x, g2y), not the local vector.
        let (perp_x, perp_y) = Transform2D::perp(g2x, g2y);

        // g1x/g1y are reserved for future checks (IK, debugging).
        // Mark them explicitly to avoid an unused warning.
        let _ = (g1x, g1y);

        // ── 3. THICK SEGMENTS ────────────────────────────────────────────────
        //
        // thick_segment = main line + 1 px offset, perpendicular to the
        // dominant direction (see draw_utils::thick_segment).
        thick_segment(gfx, shoulder_x, shoulder_y, elbow_x, elbow_y, on);
        thick_segment(gfx, elbow_x, elbow_y, end_x, end_y, on);

        // ── 4. JOINTS ────────────────────────────────────────────────────────
        //
        // Filled disks at the three pivots.
        // Radii: shoulder r=3, elbow r=3, end effector r=2.
        filled_disk(gfx, shoulder_x, shoulder_y, 3, on);
        filled_disk(gfx, elbow_x,    elbow_y,    3, on);
        filled_disk(gfx, end_x,      end_y,      2, on);

        // ── 5. PNEUMATIC GRIPPER ─────────────────────────────────────────────
        //
        // Gripper body: extends 3 px from the end effector along dir2_global.
        let body_len  = 3_i32;
        let guide_x   = end_x + (g2x * body_len as f32 + 0.5) as i32;
        let guide_y   = end_y + (g2y * body_len as f32 + 0.5) as i32;
        segment(gfx, end_x, end_y, guide_x, guide_y, on);

        // Interpolate the current jaw gap.
        let open           = gripper_opening.clamp(0.0, 1.0);
        let half_gap_max    = (self.gripper_len as f32 * 0.6) as i32;
        let half_gap_closed = self.gripper_thickness;
        let half_gap        = half_gap_closed
            + ((half_gap_max - half_gap_closed) as f32 * open) as i32;

        // Jaw origins — symmetric around the dir2_global axis.
        let jaw1_ox = guide_x + (perp_x * half_gap as f32 + 0.5) as i32;
        let jaw1_oy = guide_y + (perp_y * half_gap as f32 + 0.5) as i32;
        let jaw2_ox = guide_x - (perp_x * half_gap as f32 + 0.5) as i32;
        let jaw2_oy = guide_y - (perp_y * half_gap as f32 + 0.5) as i32;

        // Rod endpoints (extend along dir2_global).
        let jaw1_ex = jaw1_ox + (g2x * self.gripper_len as f32 + 0.5) as i32;
        let jaw1_ey = jaw1_oy + (g2y * self.gripper_len as f32 + 0.5) as i32;
        let jaw2_ex = jaw2_ox + (g2x * self.gripper_len as f32 + 0.5) as i32;
        let jaw2_ey = jaw2_oy + (g2y * self.gripper_len as f32 + 0.5) as i32;

        // Translation rail (connects the two origins).
        segment(gfx, jaw1_ox, jaw1_oy, jaw2_ox, jaw2_oy, on);

        // Jaw rods (parallel to dir2_global).
        segment(gfx, jaw1_ox, jaw1_oy, jaw1_ex, jaw1_ey, on);
        segment(gfx, jaw2_ox, jaw2_oy, jaw2_ex, jaw2_ey, on);

        // Gripping tips (perpendicular to the rods, ±tip px).
        let tip       = self.gripper_thickness.max(2);
        let tip_off_x = (perp_x * tip as f32 + 0.5) as i32;
        let tip_off_y = (perp_y * tip as f32 + 0.5) as i32;

        segment(gfx,
            jaw1_ex + tip_off_x, jaw1_ey + tip_off_y,
            jaw1_ex - tip_off_x, jaw1_ey - tip_off_y,
            on,
        );
        segment(gfx,
            jaw2_ex + tip_off_x, jaw2_ey + tip_off_y,
            jaw2_ex - tip_off_x, jaw2_ey - tip_off_y,
            on,
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // erase
    // ─────────────────────────────────────────────────────────────────────────

    /// Erases the robotic arm in the specified state.
    ///
    /// Délègue à [`draw`](Self::draw) avec `on = false`.
    /// Parameters must be **identical** to those of the last `draw` call to
    /// ensure pixel-perfect erasure without artifacts.
    ///
    /// # Typical usage (animation loop)
    ///
    /// ```rust
    /// // 1. Erase the previous position
    /// arm.erase(&mut gfx, old_shoulder, old_elbow, 0.0, Facing::Right, cosf, sinf);
    /// // 2. Draw the new position
    /// arm.draw(&mut gfx, new_shoulder, new_elbow, 0.0, Facing::Right, true, cosf, sinf);
    /// // 3. Send the framebuffer
    /// gfx.flush().await?;
    /// ```
    pub fn erase<I: I2c>(
        &self,
        gfx: &mut Graphics<'_, I>,
        angle_shoulder: f32,
        angle_elbow: f32,
        gripper_opening: f32,
        facing: Facing,
        cos_fn: fn(f32) -> f32,
        sin_fn: fn(f32) -> f32,
    ) {
        self.draw(
            gfx,
            angle_shoulder,
            angle_elbow,
            gripper_opening,
            facing,
            false, // on = false → erase
            cos_fn,
            sin_fn,
        );
    }
}