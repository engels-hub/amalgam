//! Placement of a body on a plate (S16 AM-03). AM-01 only defines the stored data types;
//! AM-03 adds the algorithms (`effective_transform`, `check_fit`, ...).

use amg_core::glam::{DQuat, DVec2};
use serde::{Deserialize, Serialize};

/// Vertical placement mode.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ZMode {
    /// Lowest point at `elevation` above the plate (0 = on the plate).
    OnPlate {
        /// Height of the lowest point above the plate, mm.
        elevation: f64,
    },
    /// Fixed plate-frame z offset, mm.
    Fixed {
        /// The offset.
        z: f64,
    },
}

/// Where and how a body sits on the plate.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Placement {
    /// Unit rotation, body-local → plate orientation.
    pub rotation: DQuat,
    /// Plate position of the rotated footprint-AABB centre, mm.
    pub xy: DVec2,
    /// Vertical mode.
    pub z: ZMode,
}

impl Placement {
    /// On the plate (elevation 0) with `rotation` at `xy`.
    pub fn on_plate(rotation: DQuat, xy: DVec2) -> Self {
        Placement {
            rotation,
            xy,
            z: ZMode::OnPlate { elevation: 0.0 },
        }
    }

    /// Re-normalises the rotation (call after composing rotations).
    pub fn normalized(self) -> Self {
        Placement {
            rotation: self.rotation.normalize(),
            ..self
        }
    }
}
