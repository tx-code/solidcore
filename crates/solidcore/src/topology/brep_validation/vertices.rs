//! Source-ordered Vertex assertions and the real corner/incidence callees.
//!
//! Upstream: maintained `d8ed45a`, `SmVertex.cpp`2550--2579,2711--2776.
//! | Rust | Upstream |
//! |------|----------|
//! | [`assert_vertex`] | `SmVertex::AssertValid` |
//! Deviations: checked ids/Result, explicit unavailable callees; no recursion.
//! The native routine ignores walk and conditionally reports a failed corner.

use super::{local_base, AssertValidOptions, Brep, CheckLevel, Object, Reporter, VertexId};
use crate::topology::vertex::{self, validity};
use crate::{
    error::Result,
    geom::assert_array::{labels, AssertArray},
    kernel::Kernel,
};

/// Check the actual vertex, including the unconditionally called geometry helper
/// and real child-use assertions. Failure reports and the returned Boolean are
/// separate evidence; neither establishes an import acceptance decision.
pub fn assert_vertex(
    k: &mut Kernel,
    b: &mut Brep,
    id: VertexId,
    options: AssertValidOptions<'_>,
    reports: Option<&mut AssertArray>,
) -> Result<bool> {
    b.vertex(id)?;
    let who = Object::Vertex(id);
    let mut report = Reporter { reports };
    let mut valid = if matches!(options.level, CheckLevel::Given(_)) {
        true
    } else {
        local_base::owning(k, b, who, &mut report)?
    };
    // Source helper always checks cached face gaps; it computes a corner only
    // when that first Boolean succeeds. Its max-gap output is NOT the movement.
    let gap = validity::within_faces(k, b, id, false)?;
    let mut location = gap.within;
    if location {
        location &= validity::is_on_face_corner(k, b, id)?.0;
    }
    if !location {
        let threshold = validity::xsect(k, b, id)?;
        if gap.max_gap > threshold && options.level.native_code() >= 2 {
            valid &=
                report.class_value(labels::VERTEX_TYPE, 1, who, false, threshold, gap.max_gap)?;
        }
    }
    let pointers = validity::check_pointers(b, id)?;
    valid &= report.class_predicate(labels::VERTEX_TYPE, 2, who, pointers)?;
    let options = AssertValidOptions {
        level: if matches!(options.level, CheckLevel::Given(_)) {
            CheckLevel::Given(&[])
        } else {
            options.level
        },
        walk: false,
    };
    for use_ in vertex::vertexuses(b, id)? {
        valid &= super::assert_vertexuse(k, b, use_, options, report.reports.as_deref_mut())?;
    }
    Ok(valid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        geom::tol,
        math::Vec3,
        topology::{vertexuse, VertexuseOwner},
    };

    #[test]
    fn fast_owned_point_drop_keeps_uv_storage_absent() {
        let (mut b, face, _, _) = super::super::tests::fixture();
        let owner = b.face(face).unwrap().surface.as_ref().unwrap().owner();
        let d = crate::geom::extent2d::Extent2d::from_bounds(0., 0., 1., 1.);
        let drop = crate::topology::surface_cache::drop_face_point(
            &Kernel::default(),
            &mut b,
            face,
            Vec3::new(0.2, 0.3, 0.0005),
            &d,
            &crate::surface::surface::p5::DropPointOptions::default(),
        )
        .unwrap();
        assert!(drop.success);
        assert_eq!(drop.result_uv, crate::math::Vec2::new(0.2, 0.3));
        assert_eq!(
            b.face(face).unwrap().surface.as_ref().unwrap().owner(),
            owner
        );
        assert!(b.edgeuses().iter().all(|(_, u)| u.uv_trim_curve.is_none()));
    }

    #[test]
    fn owned_bspline_point_drop_uses_existing_trims_and_model_classification() {
        use crate::surface::surface::{Surface, SurfaceKind};
        let (mut b, face, loop_, _) = super::super::tests::fixture();
        let k = Kernel::default();
        let lu = b.loop_(loop_).unwrap().loopuse.unwrap();
        for use_ in crate::topology::edgeuse::p1::loopuse_edgeuses(&b, lu).unwrap() {
            crate::topology::edgeuse::p3::get_or_create_uv_trim_curve(
                &k, &mut b, use_, false, false, true,
            )
            .unwrap()
            .status
            .unwrap();
        }
        let surface = b.face(face).unwrap().surface.as_ref().unwrap();
        let owner = surface.owner();
        let SurfaceKind::Plane(plane) = surface.kind() else {
            panic!("fixture")
        };
        let mut surface = Surface::from(plane.base_with_nurb().unwrap().clone_exact());
        surface.set_owner(owner);
        b.face_mut(face).unwrap().surface = Some(surface);
        let d = crate::geom::extent2d::Extent2d::from_bounds(0., 0., 1., 1.);
        let drop = crate::topology::surface_cache::drop_face_point(
            &k,
            &mut b,
            face,
            Vec3::new(0.2, 0.3, 0.0005),
            &d,
            &crate::surface::surface::p5::DropPointOptions::default(),
        )
        .unwrap();
        assert!(drop.success);
        assert!((drop.result_uv - crate::math::Vec2::new(0.2, 0.3)).length() < 1e-12);
        assert!((drop.gap - 0.0005).abs() < 1e-14);
        assert_eq!(
            b.face(face).unwrap().surface.as_ref().unwrap().owner(),
            owner
        );
    }

    #[test]
    fn cached_gap_equality_fails_within_but_does_not_create_vertex_report_one() {
        let (mut b, _, _, use_) = super::super::tests::fixture();
        let vertex = b
            .vertexuse(b.edgeuse(use_).unwrap().vertexuse.unwrap())
            .unwrap()
            .vertex
            .unwrap();
        let mut k = Kernel::default();
        let threshold = validity::xsect(&k, &b, vertex).unwrap();
        for (value, expected) in [
            (threshold, true),
            (f64::from_bits(threshold.to_bits() + 1), false),
        ] {
            for vu in vertex::vertexuses(&b, vertex).unwrap() {
                let g = &mut b.vertexuse_mut(vu).unwrap().vertex_face_gap;
                g.vertexuse = Some(vu);
                g.face_uv = crate::math::Vec2::ZERO;
                g.gap_3d = value;
            }
            assert!(
                !validity::within_faces(&k, &mut b, vertex, false)
                    .unwrap()
                    .within
            );
            let mut reports = AssertArray::new();
            assert_eq!(
                assert_vertex(
                    &mut k,
                    &mut b,
                    vertex,
                    AssertValidOptions {
                        level: CheckLevel::Two,
                        walk: true
                    },
                    Some(&mut reports)
                )
                .unwrap(),
                expected
            );
            assert_eq!(reports.len(), usize::from(!expected));
            if !expected {
                assert_eq!(
                    (
                        reports.reports()[0].reporting_type,
                        reports.reports()[0].test_index
                    ),
                    (labels::VERTEX_TYPE, 1)
                );
            }
        }
    }

    #[test]
    fn corner_projection_preserves_original_vertex_and_parameter_definition() {
        let (mut b, _, _, use_) = super::super::tests::fixture();
        let vertex = b
            .vertexuse(b.edgeuse(use_).unwrap().vertexuse.unwrap())
            .unwrap()
            .vertex
            .unwrap();
        let point = Vec3::new(0.2, 0.3, 0.0005);
        b.vertex_mut(vertex).unwrap().point = point;
        let c = validity::calc_corner_from_faces(&Kernel::default(), &mut b, vertex).unwrap();
        assert!(c.found);
        assert_eq!(c.point, Vec3::new(0.2, 0.3, 0.));
        assert_eq!(c.max_gap, 0.0005);
        assert_eq!(b.vertex(vertex).unwrap().point, point);
        assert!(!tol::in_tol(
            c.max_gap,
            tol::ScaledZero::scaled_zero(&point)
        ));
    }

    #[test]
    fn pointer_check_rejects_wrong_use_kind_but_preserves_empty_list_semantics() {
        let (mut b, _, _, use_) = super::super::tests::fixture();
        let vu = b.edgeuse(use_).unwrap().vertexuse.unwrap();
        let vertex = b.vertexuse(vu).unwrap().vertex.unwrap();
        assert!(validity::check_pointers(&b, vertex).unwrap());
        b.vertexuse_mut(vu).unwrap().owner = VertexuseOwner::Unknown;
        assert!(!validity::check_pointers(&b, vertex).unwrap());
        b.vertex_mut(vertex).unwrap().vertexuses.clear();
        assert!(validity::check_pointers(&b, vertex).unwrap());
    }

    #[test]
    fn shell_and_wire_vertex_incidence_are_checked_without_curve_geometry() {
        let mut b = Brep::new_with_infinite_region().unwrap();
        let (_, v) = crate::topology::brep::p2::make_shell_vertex_topology(&mut b, None).unwrap();
        let mut k = Kernel::default();
        let options = AssertValidOptions {
            level: CheckLevel::Given(&[]),
            walk: true,
        };
        assert!(assert_vertex(&mut k, &mut b, v, options, None).unwrap());
        let (_, tip) =
            crate::topology::brep::p2::make_edge_vertex_topology(&mut b, None, v).unwrap();
        for vertex in [v, tip] {
            assert!(validity::check_pointers(&b, vertex).unwrap());
            assert!(assert_vertex(&mut k, &mut b, vertex, options, None).unwrap());
            for vu in vertex::vertexuses(&b, vertex).unwrap() {
                assert!(!vertexuse::has_vertex_face_gap(&b, vu).unwrap());
            }
        }
    }
}
