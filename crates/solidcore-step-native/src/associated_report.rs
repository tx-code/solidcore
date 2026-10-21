// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Local CLI inventory of complete independently owned source geometry.
//!
//! The library's borrowed view owns no second semantic interpretation. JSON
//! indices name only definitions within this one result; they are not transport
//! handles to a live model. Original definitions remain in its source snapshot.

use serde_json::{json, Value};
use solidcore::math::Vec3;
use solidcore_step_native::{
    AssociatedCoordinateFrame, AssociatedDefinitionRef, AssociatedGeometry, AssociatedOccurrence,
    AssociatedRepresentationItem,
};
use std::io::{self, Write};

/// Synthetic-development CLI summary ceiling, separate from source parsing,
/// numerical quality and native-operation budgets. No production preset implied.
pub(super) const SUMMARY_LIMIT_BYTES: usize = 16 * 1024 * 1024;

fn point(p: Vec3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn frame(f: AssociatedCoordinateFrame) -> Value {
    json!({"origin_mm":point(f.origin_mm),"x":point(f.x),"y":point(f.y),"z":point(f.z)})
}

fn definition(d: AssociatedDefinitionRef) -> Value {
    let (kind, index) = match d {
        AssociatedDefinitionRef::Curve(i) => ("Curve", i),
        AssociatedDefinitionRef::Point(i) => ("Point", i),
        AssociatedDefinitionRef::Frame(i) => ("Frame", i),
        AssociatedDefinitionRef::Plane(i) => ("Plane", i),
    };
    json!({"kind":kind,"owner_local_index":index})
}

fn occurrence(o: &AssociatedOccurrence) -> Value {
    json!({"source_record":o.source_record,"source_id":o.source_id,
        "source_types":o.source_types,"definition":definition(o.definition)})
}

pub(super) fn inventory(view: &AssociatedGeometry<'_>) -> Value {
    let representations: Vec<_> = view
        .representations()
        .iter()
        .map(|r| {
            let items: Vec<_> = r
                .items
                .iter()
                .map(|i| match i {
                    AssociatedRepresentationItem::CurveSet {
                        source_record,
                        source_id,
                        group_index,
                    } => json!({"kind":"CurveSet","source_record":source_record,
                    "source_id":source_id,"group_index":group_index}),
                    AssociatedRepresentationItem::Definition(o) => {
                        json!({"kind":"Definition","occurrence":occurrence(o)})
                    }
                })
                .collect();
            json!({"role":format!("{:?}",r.role),"source_record":r.representation_record,
            "source_id":r.representation_id,"relationship_record":r.relationship_record,
            "relationship_id":r.relationship_id,"context_record":r.context_record,
            "context_id":r.context_id,"items":items})
        })
        .collect();
    let groups: Vec<_> = view
        .groups()
        .iter()
        .map(|g| {
            json!({
                "role":format!("{:?}",g.role),"representation_record":g.representation_record,
                "representation_id":g.representation_id,"relationship_record":g.relationship_record,
                "relationship_id":g.relationship_id,"context_record":g.context_record,
                "context_id":g.context_id,"set_record":g.set_record,"set_id":g.set_id,
                "items":g.items.iter().map(occurrence).collect::<Vec<_>>(),
                "bounds_mm":g.bounds_mm.map(|b|b.map(point)),
                "source_residual_mm":g.source_residual_mm,"endpoint_error_mm":g.endpoint_error_mm
            })
        })
        .collect();
    let curves:Vec<_> = view.curve_definitions().iter().map(|c|json!({
        "source_record":c.source_record,"source_id":c.source_id,"basis_record":c.basis_record,
        "basis_id":c.basis_id,"parameter_domain":c.parameter_domain,
        "source_forward_in_native":c.source_forward_in_native,
        "source_residual_mm":c.source_residual_mm,"endpoint_error_mm":c.endpoint_error_mm
    })).collect();
    let points: Vec<_> = view
        .point_definitions()
        .iter()
        .map(|p| {
            json!({
                "source_record":p.source_record,"source_id":p.source_id,"point_mm":point(p.point_mm)
            })
        })
        .collect();
    let frames: Vec<_> = view
        .frame_definitions()
        .iter()
        .map(|f| {
            json!({
                "source_record":f.source_record,"source_id":f.source_id,"frame":frame(f.frame)
            })
        })
        .collect();
    let planes:Vec<_> = view.plane_definitions().iter().map(|p|json!({
        "source_record":p.source_record,"source_id":p.source_id,"position_record":p.position_record,
        "position_id":p.position_id,"frame":frame(p.frame),"parameter_domain":p.parameter_domain,
        "source_trim":"Unbounded"
    })).collect();
    json!({"scope":"independent_associated_geometry_not_material_topology",
        "indices":"owner_local_definition_indices_not_live_model_transport_handles",
        "representations":representations,"groups":groups,"curves":curves,
        "points":points,"frames":frames,"planes":planes})
}

struct Bounded {
    bytes: Vec<u8>,
    limit: usize,
}
impl Write for Bounded {
    fn write(&mut self, input: &[u8]) -> io::Result<usize> {
        let next = self
            .bytes
            .len()
            .checked_add(input.len())
            .ok_or_else(|| io::Error::other("Associated native report limit exceeded"))?;
        if next > self.limit {
            return Err(io::Error::other("Associated native report limit exceeded"));
        }
        self.bytes.extend_from_slice(input);
        Ok(input.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// Serialize the complete required inventory before writing anything to stdout.
/// A budget failure releases the unpublished candidate and private buffer; no
/// body-only or truncated-success JSON is emitted. This is not a process-RSS cap.
pub(super) fn complete_report(value: &Value, limit: usize) -> io::Result<Vec<u8>> {
    let mut output = Bounded {
        bytes: Vec::new(),
        limit,
    };
    serde_json::to_writer(&mut output, value).map_err(io::Error::other)?;
    output.write_all(b"\n")?;
    Ok(output.bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn complete_inventory_serialization_charges_final_newline_and_exact_budget() {
        let value = json!({"original":"圈\n\\","definitions":[{"domain":[0.,1.]}]});
        let expected = format!("{value}\n").into_bytes();
        assert_eq!(complete_report(&value, expected.len()).unwrap(), expected);
        assert!(complete_report(&value, expected.len() - 1).is_err());
        assert!(complete_report(&value, 0).is_err());
        assert_eq!(complete_report(&value, expected.len()).unwrap(), expected);
    }
}
