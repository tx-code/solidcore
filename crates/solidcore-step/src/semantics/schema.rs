// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Fixed schema identity recognition, independent of type/geometry qualification.
//! Upstream: `source/STEP_SEMANTICS/src/StepSchemaSelection.cpp` (Identify/SelectSchema).
//! Deviations: safe source views and explicit bounded implementation assumptions;
//! an applied choice never authenticates the original schema declaration.
use super::{
    generated::CATALOGS, GeometryScope, Profile, SchemaIdentifier, SchemaInterpretation,
    SchemaSelection,
};
use crate::{Snapshot, Value};

const AP203: &str =
    "AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF";
const VERSIONS: &[(&str, &str, &str)] = &[
    (AP203, "1 0 10303 403 2 1 2", "ap203e2-n2635"),
    (
        "AUTOMOTIVE_DESIGN",
        "1 0 10303 214 1 1 1",
        "ap214-edition-1",
    ),
    (
        "AUTOMOTIVE_DESIGN",
        "1 0 10303 214 2 1 1",
        "ap214-edition-2",
    ),
    (
        "AUTOMOTIVE_DESIGN",
        "1 0 10303 214 3 1 1",
        "ap214-edition-3",
    ),
    (
        "AUTOMOTIVE_DESIGN",
        "1 0 10303 214 1 1 1 1",
        "ap214-mbx-2010",
    ),
];
fn words(s: &str) -> Vec<&str> {
    s.split([' ', '\t', '\r', '\n', '\u{000b}', '\u{000c}'])
        .filter(|word| !word.is_empty())
        .collect()
}
fn identify(value: &Value) -> SchemaIdentifier {
    let mut result = SchemaIdentifier {
        literal: value.text.clone(),
        name: String::new(),
        oid: Vec::new(),
        span: value.span,
        protocol: "Unknown".into(),
        matched: "Malformed".into(),
        registry_id: String::new(),
    };
    let opening = value.text.find('{');
    let closing = value.text.find('}');
    let has_oid = opening.is_some() || closing.is_some();
    let prefix = opening
        .and_then(|i| value.text.get(..i))
        .unwrap_or(&value.text);
    let names = words(prefix);
    let Some(name) = names.first().filter(|_| names.len() == 1) else {
        return result;
    };
    result.name = (*name).into();
    let letter = |c: u8| c.is_ascii_alphabetic() || c == b'_';
    if !name.as_bytes().first().copied().is_some_and(letter)
        || !name.bytes().all(|c| letter(c) || c.is_ascii_digit())
    {
        return result;
    }
    if has_oid {
        let (Some(a), Some(b)) = (opening, closing) else {
            return result;
        };
        if a >= b
            || value.text.matches('{').count() != 1
            || value.text.matches('}').count() != 1
            || value.text.get(b + 1..).is_none_or(|s| !words(s).is_empty())
        {
            return result;
        }
        let Some(raw) = value.text.get(a + 1..b) else {
            return result;
        };
        result.oid = words(raw).into_iter().map(str::to_owned).collect();
        if result.oid.is_empty()
            || !result
                .oid
                .iter()
                .all(|arc| arc.bytes().all(|c| c.is_ascii_digit()))
        {
            return result;
        }
    }
    result.protocol = match *name {
        AP203 => "AP203e2",
        "CONFIG_CONTROL_DESIGN" => "AP203e1",
        "AUTOMOTIVE_DESIGN" | "AUTOMOTIVE_DESIGN_CC2" => "AP214",
        "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF" => "AP242",
        _ => "Unknown",
    }
    .into();
    result.matched = if result.protocol == "Unknown" {
        "UnknownName"
    } else if has_oid {
        "UnregisteredVersion"
    } else {
        "NameOnly"
    }
    .into();
    if !has_oid && result.name == "AUTOMOTIVE_DESIGN_CC2" {
        result.matched = "LegacyAlias".into();
        result.registry_id = "ap214-legacy-cc2".into();
    }
    for (n, oid, id) in VERSIONS {
        if *n == result.name
            && words(oid)
                .iter()
                .copied()
                .eq(result.oid.iter().map(String::as_str))
        {
            result.matched = "RegisteredVersion".into();
            result.registry_id = (*id).into();
            break;
        }
    }
    result
}
pub(super) fn select(source: &Snapshot, scope: GeometryScope) -> SchemaSelection {
    let identifiers = source
        .header()
        .get(2)
        .and_then(|c| c.parameters.first())
        .map_or_else(Vec::new, |v| v.children.iter().map(identify).collect());
    let mut selection = SchemaSelection {
        identifiers,
        qualified: None,
        applied_interpretation: SchemaInterpretation::DeclaredOnly,
    };
    if selection.identifiers.len() != 1 {
        return selection;
    }
    let Some(identity) = selection.identifiers.first() else {
        return selection;
    };
    let chosen = match identity.registry_id.as_str() {
        "ap203e2-n2635" => Some(if scope == GeometryScope::GeneralBrep {
            "Ap203e2GeneralSource"
        } else {
            "Ap203e2Planar"
        }),
        "ap214-edition-3" => Some("Ap214e3Source"),
        "ap214-mbx-2010" => Some("Ap214MbxSource"),
        _ if scope == GeometryScope::GeneralBrep
            && identity.name == "CONFIG_CONTROL_DESIGN"
            && identity.matched == "NameOnly" =>
        {
            Some("Ap203N916Source")
        }
        _ => None,
    };
    if let Some(catalog) = chosen.and_then(|kind| CATALOGS.iter().find(|c| c.kind == kind)) {
        selection.qualified = Some(Profile {
            registry_id: if catalog.kind == "Ap203N916Source" {
                "ap203-nameonly-n916-source".into()
            } else {
                identity.registry_id.clone()
            },
            profile: catalog.profile,
            express_sha256: catalog.schema_sha256,
            kind: catalog.kind,
        });
    }
    selection
}

pub(super) fn explicit(
    selection: &SchemaSelection,
    policy: SchemaInterpretation,
) -> Option<Profile> {
    if selection.qualified.is_some() || selection.identifiers.len() != 1 {
        return None;
    }
    let identity = selection.identifiers.first()?;
    // StepSchemaSelection.cpp at d8ed45a: these are disjoint exact spellings,
    // not an unknown-schema override or a proof of HEADER edition identity.
    let eight = policy == SchemaInterpretation::UnregisteredAp214As2010
        && identity.matched == "UnregisteredVersion"
        && identity
            .oid
            .iter()
            .map(String::as_str)
            .eq(words("1 0 10303 214 3 1 1 1"));
    let bare = policy == SchemaInterpretation::Ap214NameOnlyAs2010
        && identity.matched == "NameOnly"
        && identity.oid.is_empty();
    if identity.name != "AUTOMOTIVE_DESIGN" || (!eight && !bare) {
        return None;
    }
    let catalog = CATALOGS.iter().find(|c| c.kind == "Ap214MbxSource")?;
    Some(Profile {
        registry_id: "ap214-mbx-2010".into(),
        profile: catalog.profile,
        express_sha256: catalog.schema_sha256,
        kind: catalog.kind,
    })
}

pub(super) const AP203_EXPECTED:&str="AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF {1 0 10303 403 2 1 2}";
