// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Native geometric point-grid coincidence and local monotonicity queries.
//!
//! Upstream: `SmPointSet.cpp` (1983--2007,2094--2341), `SmPointSet.h`
//! (431--550), maintained `d8ed45a`.
//! | Rust | Upstream |
//! |------|----------|
//! | [`PointGrid`] | `SmPointGrid` |
//! | [`PointGrid::coincident`] | `GetCoincidentCol/Row`, `HasCoincidentPoints` |
//! | [`PointGrid::monotonic`] | `GetMonotonicCol/Row`, `HasNonMonotonicPoints` |
//! Deviations: owned points follow the existing PointSet3d convention. Checked
//! indexing replaces undefined native undersized-grid access. Uninitialized
//! cache storage becomes None; native cache invalidation and algorithm quirks
//! are retained. No recursion or input-count preallocation is introduced.

use crate::error::{Result, SolidError};
use crate::geom::point_set::PointSet3d;
use crate::math::{Point3, Vec3, EFF_ZERO_SQ};
use core::ops::Deref;

/// Column and row results, in native accessor order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridDirections {
    /// Column-direction result (vary row index).
    pub column: bool,
    /// Row-direction result (vary column index).
    pub row: bool,
}

/// A point grid with immutable point data and native lazy query caches.
#[derive(Debug, Clone)]
pub struct PointGrid {
    base: PointSet3d,
    row_size: usize,
    closed_column: bool,
    closed_row: bool,
    tolerance: f64,
    coincident: Option<GridDirections>,
    monotonic: Option<GridDirections>,
}
impl Deref for PointGrid {
    type Target = PointSet3d;
    fn deref(&self) -> &Self::Target {
        &self.base
    }
}

impl PointGrid {
    /// Construct with the native point order and closed-column/closed-row flags.
    /// A nonmultiple row size is retained (the native constructor warns only).
    /// A zero row size returns a checked error instead of native division by zero.
    pub fn new(
        points: Vec<Point3>,
        row_size: usize,
        tolerance: f64,
        closed_column: bool,
        closed_row: bool,
        normal: Option<&mut Vec3>,
    ) -> Result<Self> {
        if row_size == 0 {
            return Err(invalid());
        }
        Ok(Self {
            base: PointSet3d::new(points, tolerance, normal),
            row_size,
            closed_column,
            closed_row,
            tolerance,
            coincident: None,
            monotonic: None,
        })
    }
    /// Number of points per row (`GetColCnt`).
    pub const fn column_count(&self) -> usize {
        self.row_size
    }
    /// Number of complete rows (`GetRowCnt`), retaining native integer division.
    pub fn row_count(&self) -> usize {
        self.points().len() / self.row_size
    }
    /// Whether the supplied row size partitions every point (`IsGoodRowSize`).
    pub fn is_good_row_size(&self) -> bool {
        self.points().len() % self.row_size == 0
    }
    /// Current native closed-column flag.
    pub const fn closed_column(&self) -> bool {
        self.closed_column
    }
    /// Current native closed-row flag.
    pub const fn closed_row(&self) -> bool {
        self.closed_row
    }
    /// The native setter does not invalidate cached queries.
    pub fn set_closed_column(&mut self, closed: bool) {
        self.closed_column = closed;
    }
    /// The native setter does not invalidate cached queries.
    pub fn set_closed_row(&mut self, closed: bool) {
        self.closed_row = closed;
    }
    /// Native `ClearCache` resets base and monotonic caches, NOT coincidence.
    pub fn clear_cache(&mut self) {
        self.base.clear_cache();
        self.monotonic = None;
    }

    /// Whether a neighboring pair in any column/row is closer than the supplied
    /// distance tolerance. Native `HasCoincidentPoints` ANDs these two results.
    pub fn coincident(&mut self) -> Result<GridDirections> {
        if let Some(cached) = self.coincident {
            return Ok(cached);
        }
        let mut result = GridDirections {
            column: false,
            row: false,
        };
        for row in 0..self.row_count() {
            let start = row * self.row_size;
            let end = start + self.row_size;
            let (mut previous, mut current) = if self.closed_row {
                (end - 1, start)
            } else {
                (start, start + 1)
            };
            while current < end {
                if point(self.points(), previous)?
                    .closer_than(self.tolerance, point(self.points(), current)?)
                {
                    result.row = true;
                    break;
                }
                current += 1;
                previous = current - 1;
            }
            if result.row {
                break;
            }
        }
        for column in 0..self.row_size {
            let (mut previous, mut current) = if self.closed_column {
                (
                    column
                        .wrapping_add(self.points().len())
                        .wrapping_sub(self.row_size),
                    column,
                )
            } else {
                (
                    column,
                    column.checked_add(self.row_size).ok_or_else(invalid)?,
                )
            };
            while current < self.points().len() {
                if point(self.points(), previous)?
                    .closer_than(self.tolerance, point(self.points(), current)?)
                {
                    result.column = true;
                    break;
                }
                current = current.checked_add(self.row_size).ok_or_else(invalid)?;
                previous = current - self.row_size;
            }
            if result.column {
                break;
            }
        }
        self.coincident = Some(result);
        Ok(result)
    }

    /// Native local-turn predicate (>170 degrees), preserving the stale first
    /// squared length and the column branch writing the ROW result. It is not a
    /// proof of injectivity or geometric validity of a surface/control net.
    pub fn monotonic(&mut self) -> Result<GridDirections> {
        if let Some(cached) = self.monotonic {
            return Ok(cached);
        }
        let mut result = GridDirections {
            column: true,
            row: true,
        };
        for row in 0..self.row_count() {
            let start = row * self.row_size;
            let end = start + self.row_size;
            let (previous, current) = if self.closed_row {
                (end - 1, start)
            } else {
                (start, start + 1)
            };
            let mut vector = point(self.points(), current)? - point(self.points(), previous)?;
            let mut length = vector.length_squared();
            for current in current + 1..end {
                let next = point(self.points(), current)? - point(self.points(), current - 1)?;
                let next_length = next.length_squared();
                if length < EFF_ZERO_SQ || next_length < EFF_ZERO_SQ {
                    vector = next;
                    length = next_length;
                    continue;
                }
                if angle(next, vector)? > 170. {
                    result.row = false;
                    break;
                }
                vector = next; // Native deliberately leaves length unchanged here.
            }
            if !result.row {
                break;
            }
        }
        for column in 0..self.row_size {
            let (previous, current) = if self.closed_column {
                (
                    column
                        .wrapping_add(self.points().len())
                        .wrapping_sub(self.row_size),
                    column,
                )
            } else {
                (
                    column,
                    column.checked_add(self.row_size).ok_or_else(invalid)?,
                )
            };
            let mut vector = point(self.points(), current)? - point(self.points(), previous)?;
            let mut length = vector.length_squared();
            let mut current = current.checked_add(self.row_size).ok_or_else(invalid)?;
            while current < self.points().len() {
                let next =
                    point(self.points(), current)? - point(self.points(), current - self.row_size)?;
                let next_length = next.length_squared();
                if length < EFF_ZERO_SQ || next_length < EFF_ZERO_SQ {
                    vector = next;
                    length = next_length;
                } else {
                    if angle(next, vector)? > 170. {
                        result.row = false;
                        break;
                    } // Source writes m_bMonotonicRow.
                    vector = next;
                }
                current = current.checked_add(self.row_size).ok_or_else(invalid)?;
            }
            // Source tests m_bMonotonicCol, which remains true, so all columns
            // are still visited even after the row flag was cleared.
        }
        self.monotonic = Some(result);
        Ok(result)
    }
}
fn invalid() -> SolidError {
    SolidError::invalid_input("geom::point_grid", "invalid native point-grid access")
}
fn point(points: &[Point3], index: usize) -> Result<Point3> {
    points.get(index).copied().ok_or_else(invalid)
}
fn angle(first: Vec3, second: Vec3) -> Result<f64> {
    match first.angle_between(second) {
        Ok(radians) => Ok(crate::math::rad_to_deg(radians)),
        // Native AngleBetween writes zero before its failure status is ignored.
        Err(error) if error.is_unported() => Err(error),
        Err(_) => Ok(0.),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn regular() -> Vec<Point3> {
        (0..3)
            .flat_map(|i| (0..3).map(move |j| Point3::new(f64::from(j), f64::from(i), 0.)))
            .collect()
    }

    #[test]
    fn a_column_reversal_retains_the_original_row_flag_assignment() {
        let points = (0..3)
            .flat_map(|i| {
                (0..3).map(move |j| Point3::new(f64::from(j), if i == 1 { 1. } else { 0. }, 0.))
            })
            .collect();
        let mut grid = PointGrid::new(points, 3, 1e-6, false, false, None).unwrap();
        assert_eq!(
            grid.monotonic().unwrap(),
            GridDirections {
                column: true,
                row: false
            }
        );
    }

    #[test]
    fn closed_flag_setters_retain_cache_until_native_clear_cache() {
        let mut grid = PointGrid::new(regular(), 3, 1e-6, false, false, None).unwrap();
        assert!(grid.monotonic().unwrap().row);
        grid.set_closed_row(true);
        assert!(grid.monotonic().unwrap().row);
        grid.clear_cache();
        assert!(!grid.monotonic().unwrap().row);
    }

    #[test]
    fn clear_cache_does_not_reset_the_original_coincidence_cache() {
        let points = (0..3)
            .flat_map(|i| {
                (0..3).map(move |j| Point3::new(if j == 1 { 1. } else { 0. }, f64::from(i), 0.))
            })
            .collect::<Vec<_>>();
        let mut grid = PointGrid::new(points.clone(), 3, 1e-6, false, false, None).unwrap();
        assert!(!grid.coincident().unwrap().row);
        grid.set_closed_row(true);
        grid.clear_cache();
        assert!(!grid.coincident().unwrap().row);
        assert!(
            PointGrid::new(points, 3, 1e-6, false, true, None)
                .unwrap()
                .coincident()
                .unwrap()
                .row
        );
    }

    #[test]
    fn native_undersized_access_is_checked_without_blanket_rejection_of_closed_single_rows() {
        assert!(PointGrid::new(vec![Point3::ZERO], 0, 1e-6, false, false, None).is_err());
        let points = vec![Point3::ZERO, Vec3::X, Vec3::new(2., 0., 0.)];
        assert!(PointGrid::new(points.clone(), 3, 1e-6, false, false, None)
            .unwrap()
            .monotonic()
            .is_err());
        assert!(
            PointGrid::new(points, 3, 1e-6, true, false, None)
                .unwrap()
                .monotonic()
                .unwrap()
                .row
        );
    }
}
