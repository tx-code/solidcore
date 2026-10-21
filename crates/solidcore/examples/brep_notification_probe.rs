// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Host observer of actual root notification state and callback order.
use serde_json::{json, Value};
use solidcore::curve::{curve::Curve, line::Line};
use solidcore::error::{EntityRef, SolidError};
use solidcore::geom::attribute::{Attribute, AttributeValue, Behavior, ObjectKey};
use solidcore::geom::context::{with_mark_lock, MarkType, NotifyCallback};
use solidcore::geom::extent2d::Extent2d;
use solidcore::geom::object::{NotifyOperation as N, ObjectRef};
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec2, Vec3};
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::surface::{plane::Plane, surface::Surface};
use solidcore::topology::brep::p2::{notify_owned as notify, object_ref, root_ref};
use solidcore::topology::brep_cache::{BrepCache, CacheObject};
use solidcore::topology::brep_constructor::{BrepConstructor, VertexNode};
use solidcore::topology::{Brep, BrepId, FaceId, VertexId};
use std::error::Error;
use std::sync::{Arc, Mutex};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn at<T: Copy>(values: &[T], index: usize) -> solidcore::Result<T> {
    values
        .get(index)
        .copied()
        .ok_or_else(|| SolidError::invalid_input("notification probe", "synthetic index"))
}

#[derive(Debug)]
struct Capture {
    tag: u64,
    rows: Arc<Mutex<Vec<[u64; 6]>>>,
}
impl NotifyCallback for Capture {
    fn execute(
        &mut self,
        n: N,
        c: ObjectRef,
        a: Option<ObjectRef>,
        b: Option<ObjectRef>,
        d: Option<ObjectRef>,
    ) -> solidcore::Result<()> {
        self.rows
            .lock()
            .map_err(|_| SolidError::invalid_input("notification probe", "sink poisoned"))?
            .push([
                self.tag,
                u64::try_from(n as i32).unwrap_or(u64::MAX),
                c.key.0,
                a.map_or(0, |x| x.key.0),
                b.map_or(0, |x| x.key.0),
                d.map_or(0, |x| x.key.0),
            ]);
        Err(SolidError::sm_err("synthetic callback status ignored"))
    }
}
fn fixture(k: &mut Kernel) -> Result<(Brep, FaceId, Vec<VertexId>)> {
    let mut c = BrepConstructor::new()?;
    c.start_shell()?;
    let f = c.start_face(OrientType::Same)?;
    let d = Extent2d::from_bounds(0., 0., 1., 1.);
    c.set_face_surface(
        Surface::from(Plane::new(
            Point3::ZERO,
            Vec3::X,
            Vec3::Y,
            Vec2::new(1., 1.),
            &d,
        )),
        d,
    )?;
    c.start_loop(OrientType::Same)?;
    let points = [Point3::ZERO, Vec3::X, Vec3::Y];
    let v = points
        .iter()
        .map(|p| c.start_vertex_of_loop(*p))
        .collect::<solidcore::Result<Vec<_>>>()?;
    for i in 0..3 {
        let j = (i + 1) % 3;
        c.start_edge(
            k,
            OrientType::Same,
            None,
            Some(at(&v, i)?),
            Some(at(&v, j)?),
        )?;
        c.set_edge_curve(
            k,
            Curve::from(Line::from_points(at(&points, i)?, at(&points, j)?, 3)),
            false,
        )?;
        c.set_limits(k, VertexNode::Parameter(0.), VertexNode::Parameter(1.))?;
        c.end_edge()?;
    }
    c.end_loop(k)?;
    c.end_face()?;
    c.end_shell(k, false)?;
    Ok((c.finish()?, f, v))
}
fn trees(cache: Option<&BrepCache>) -> solidcore::Result<Value> {
    let Some(cache) = cache else {
        return Ok(Value::Null);
    };
    let mut rows = Vec::new();
    for tree in [
        cache.vertex_tree(),
        cache.curve_tree(),
        cache.surface_tree(),
    ] {
        let tree = tree.ok_or_else(|| SolidError::null_pointer("notification tree"))?;
        let mut keys = std::collections::BTreeSet::new();
        for id in tree.object_list()? {
            match tree.list(id)?.object {
                Some(CacheObject::Vertex(v)) => {
                    keys.insert(v.index());
                }
                Some(CacheObject::Curve(e)) => {
                    keys.insert(e.index());
                }
                Some(CacheObject::Surface(f)) => {
                    keys.insert(f.index());
                }
                None => {}
                _ => {
                    return Err(SolidError::invalid_input(
                        "notification tree",
                        "unexpected object",
                    ))
                }
            }
        }
        rows.push(keys.into_iter().collect::<Vec<_>>());
    }
    Ok(json!(rows))
}
fn run() -> Result<Value> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let mode = args.first().ok_or("event cache repeat")?;
    let mut k = Kernel::default();
    let (mut b, face, v) = fixture(&mut k)?;
    b.set_key(ObjectKey(
        u64::try_from(solidcore::geom::assert_array::labels::BREP_TYPE)? << 32,
    ));
    let root = EntityRef::Brep(BrepId::from_index(0));
    let objects = [
        root,
        EntityRef::Vertex(at(&v, 0)?),
        EntityRef::Vertex(at(&v, 1)?),
        EntityRef::Vertex(at(&v, 2)?),
    ];
    for (i, object) in objects.iter().enumerate() {
        b.notification_topology_mut(*object)?
            .set_all_flags(if i == 0 { 16 } else { 1 << (i - 1) });
        for mark in [MarkType::Mark, MarkType::Mark2, MarkType::Mark3] {
            with_mark_lock(&mut k, mark as i32, |kernel, _| {
                b.notification_topology_mut(*object)
                    .map(|r| r.mark(Some(&kernel.context), mark))
            })??;
        }
        let key = object_ref(&b, *object)?.key;
        let h = b.notifications_mut().attributes.insert(Attribute::long(
            90001,
            21 + 10 * i64::try_from(i)?,
            Behavior::Copy,
        ))?;
        b.notifications_mut().attributes.add_attribute(key, h)?;
    }
    if args.get(1).is_some_and(|a| a == "1") {
        b.build_notification_cache(&k)?;
    }
    let br = root_ref(&b);
    let vr = |i: usize| object_ref(&b, EntityRef::Vertex(at(&v, i)?));
    let edge = *b.edge_list().first().ok_or("edge")?;
    let er = object_ref(&b, EntityRef::Edge(edge))?;
    let fr = object_ref(&b, EntityRef::Face(face))?;
    let (op, data) = match mode.as_str() {
        "pre_edit" => (N::PreEdit, [Some(br), None, None]),
        "destruction" => (N::Destruction, [Some(br), None, None]),
        "post_edit" => (N::PostEdit, [Some(br), None, None]),
        "add_v" => (N::AddToBrep, [Some(vr(0)?), Some(br), None]),
        "remove_v" => (N::RmFromBrep, [Some(vr(0)?), Some(br), None]),
        "add_e" => (N::AddToBrep, [Some(er), Some(br), None]),
        "remove_e" => (N::RmFromBrep, [Some(er), Some(br), None]),
        "add_f" => (N::AddToBrep, [Some(fr), Some(br), None]),
        "remove_f" => (N::RmFromBrep, [Some(fr), Some(br), None]),
        "split" => (N::SplitInBrep, [Some(vr(0)?), Some(vr(1)?), Some(vr(2)?)]),
        "split_alias" => (N::SplitInBrep, [Some(vr(0)?), Some(vr(0)?), Some(vr(2)?)]),
        "merge" => (N::MergeInBrep, [Some(vr(0)?), Some(vr(1)?), Some(br)]),
        "copy" => (N::Copy, [Some(vr(0)?), None, None]),
        "construct" => (N::Construction, [Some(br), Some(vr(0)?), None]),
        "coincident" => (N::Coincident, [Some(vr(0)?), Some(vr(1)?), Some(br)]),
        "unknown" => (N::Unknown, [None, None, None]),
        _ => return Err("unknown event".into()),
    };
    let rows = Arc::new(Mutex::new(Vec::new()));
    k.context.set_user_notify_callback(Some(Box::new(Capture {
        tag: 0,
        rows: Arc::clone(&rows),
    })));
    k.context.set_sys_notify_callback(Some(Box::new(Capture {
        tag: 1,
        rows: Arc::clone(&rows),
    })));
    notify(&mut k, &mut b, op, data)?;
    if args.get(2).is_some_and(|a| a == "1") {
        notify(&mut k, &mut b, op, data)?;
    }
    let mut metadata = Vec::new();
    for object in objects {
        let key = object_ref(&b, object)?.key;
        let record = b.notification_topology_mut(object)?;
        let fields = [
            record.all_flags(),
            record.get_mark(MarkType::Mark),
            record.get_mark(MarkType::Mark2),
            record.get_mark(MarkType::Mark3),
        ];
        let attrs = b
            .notifications()
            .attributes
            .attributes(key)
            .into_iter()
            .map(
                |h| match b.notifications().attributes.get(h).map(Attribute::value) {
                    Some(AttributeValue::Long(v)) => Ok(*v),
                    _ => Err("attribute kind"),
                },
            )
            .collect::<std::result::Result<Vec<_>, _>>()?;
        metadata.push(json!([fields[0], fields[1], fields[2], fields[3], attrs]));
    }
    let events = rows.lock().map_err(|_| "sink poisoned")?.clone();
    Ok(
        json!({"events":events,"metadata":metadata,"trees":trees(b.notifications().spatial_cache.as_ref())?}),
    )
}
fn main() -> Result<()> {
    match run() {
        Ok(value) => println!("{value}"),
        Err(e) => {
            if let Some(SolidError::Unported { upstream, unit }) = e.downcast_ref::<SolidError>() {
                println!("{}", json!({"deferred":{"upstream":upstream,"unit":unit}}));
            } else {
                return Err(e);
            }
        }
    }
    Ok(())
}
