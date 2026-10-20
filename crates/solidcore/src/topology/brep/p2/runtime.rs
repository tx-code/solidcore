//! `BRep` notifications and their owning runtime state.
//!
//! Upstream: `SmBrep.cpp::Notify`, `sm_AddToSpatialTree`, `sm_RmFromSpatialTree`
//! at maintained fork `d8ed45a`; `SmSAGObject` / `SmTopology` / `SmAObject::Notify`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`notify`] | `SmBrep::Notify` |
//! | [`NotificationState`] | owned topology base records, attributes and `BRep` cache |
//! | [`object_ref`] | model-scoped native object identity |
//!
//! Deviations: checked model-local identities replace pointer casts. The caller
//! sets the model key before attaching attributes; cross-model propagation needs
//! a shared object resolver and is not inferred from coincident arena indices.
//! Snapshot Clone retains value metadata but drops the spatial cache; it does
//! not claim the full native Copy protocol. Graphics display lists are absent.

use std::collections::BTreeMap;

use crate::error::{EntityRef, Result, SolidError};
use crate::geom::assert_array::labels::{
    BREP_TYPE, EDGEUSE_TYPE, EDGE_TYPE, FACEUSE_TYPE, FACE_TYPE, FILLET_EDGE_TYPE, LOOPUSE_TYPE,
    LOOP_TYPE, REGION_TYPE, SHELL_TYPE, VERTEXUSE_TYPE, VERTEX_TYPE,
};
use crate::geom::attribute::{AttributeStore, ObjectKey};
use crate::geom::object::{NotifyOperation, ObjectRef};
use crate::kernel::Kernel;
use crate::topology::brep_cache::{arena_identity, make_brep_cache, BrepCache, CacheSource};
use crate::topology::topology::Topology;
use crate::topology::{Brep, BrepId, EdgeId, FaceId, RegionId, VertexId};

const OP: &str = "topology::brep::p2::notify";

/// Runtime data belongs to one model, rather than a discarded per-call map.
/// Notification keys are local to this model; context callbacks receive the
/// notifying `BRep` key as their caller. Set a distinct model key for each model.
#[derive(Debug, Default)]
pub struct NotificationState {
    /// Notification marks, flags and user fields of this model's objects. The
    /// arena graph owns list links; these records are not a second graph and
    /// must not be used as a topology validator's ownership/list evidence.
    pub topology: BTreeMap<ObjectKey, Topology>,
    /// The bidirectional attribute/user lists, used by the existing handlers.
    pub attributes: AttributeStore,
    /// The model's actual optional spatial cache.
    pub spatial_cache: Option<BrepCache>,
}

impl NotificationState {
    /// Whether an arena-only adapter would lose configured model state.
    pub(crate) fn has_owned_effects(&self) -> bool {
        use crate::geom::context::MarkType;
        self.spatial_cache.is_some()
            || self.attributes != AttributeStore::default()
            || self.topology.values().any(|v| {
                v.all_flags() != 0
                    || v.user_index1() != 0
                    || v.user_index2() != 0
                    || v.user_ptr1() != 0
                    || [
                        MarkType::Mark,
                        MarkType::Mark2,
                        MarkType::Mark3,
                        MarkType::MarkIo,
                        MarkType::MarkAssert,
                    ]
                    .into_iter()
                    .any(|m| v.get_mark(m) != 0)
            })
    }
}

impl Clone for NotificationState {
    fn clone(&self) -> Self {
        Self {
            topology: self.topology.clone(),
            attributes: self.attributes.clone(),
            spatial_cache: None,
        }
    }
}

/// The notifying root's identity, in its model's notification namespace.
pub fn root_ref(b: &Brep) -> ObjectRef {
    ObjectRef {
        type_code: BREP_TYPE,
        key: b.key,
    }
}

/// A live local topology identity. Type bits distinguish equal arena indices;
/// the root uses the caller-assigned model key. Geometry owner handles retain
/// their existing geometry namespace and are not rewritten by this adapter.
pub fn object_ref(b: &Brep, entity: EntityRef) -> Result<ObjectRef> {
    let (kind, index) = match entity {
        EntityRef::Brep(_) => return Ok(root_ref(b)),
        EntityRef::Region(id) => {
            b.region(id)?;
            (REGION_TYPE, id.index())
        }
        EntityRef::Shell(id) => {
            b.shell(id)?;
            (SHELL_TYPE, id.index())
        }
        EntityRef::Face(id) => {
            b.face(id)?;
            (FACE_TYPE, id.index())
        }
        EntityRef::Loop(id) => {
            b.loop_(id)?;
            (LOOP_TYPE, id.index())
        }
        EntityRef::Edge(id) => {
            b.edge(id)?;
            (EDGE_TYPE, id.index())
        }
        EntityRef::Vertex(id) => {
            b.vertex(id)?;
            (VERTEX_TYPE, id.index())
        }
        EntityRef::Faceuse(id) => {
            b.faceuse(id)?;
            (FACEUSE_TYPE, id.index())
        }
        EntityRef::Loopuse(id) => {
            b.loopuse(id)?;
            (LOOPUSE_TYPE, id.index())
        }
        EntityRef::Edgeuse(id) => {
            b.edgeuse(id)?;
            (EDGEUSE_TYPE, id.index())
        }
        EntityRef::Vertexuse(id) => {
            b.vertexuse(id)?;
            (VERTEXUSE_TYPE, id.index())
        }
    };
    let kind_bits =
        u64::try_from(kind).map_err(|_| SolidError::invalid_input(OP, "negative class code"))?;
    let key = ObjectKey((kind_bits << 32) | u64::from(index));
    if key == b.key {
        return Err(SolidError::invalid_input(
            OP,
            "model key aliases a topology key",
        ));
    }
    Ok(ObjectRef {
        type_code: kind,
        key,
    })
}

fn index(object: ObjectRef) -> Result<u32> {
    let kind = u64::try_from(object.type_code)
        .map_err(|_| SolidError::invalid_input(OP, "negative class code"))?;
    if object.key.0 >> 32 != kind {
        return Err(SolidError::invalid_input(
            OP,
            "object is outside this topology namespace",
        ));
    }
    u32::try_from(object.key.0 & u64::from(u32::MAX))
        .map_err(|_| SolidError::invalid_input(OP, "topology index overflow"))
}

struct Regions<'a>(&'a Brep);
impl crate::topology::shell::RegionLookup for Regions<'_> {
    fn resolve_region(&self, key: ObjectKey) -> Result<crate::topology::shell::RegionRef<'_>> {
        let id = RegionId::from_index(index(ObjectRef {
            type_code: REGION_TYPE,
            key,
        })?);
        self.0.region(id)?;
        Ok(crate::topology::shell::RegionRef {
            brep: self.0,
            brep_id: BrepId::from_index(0),
            region_id: id,
        })
    }
}

impl Brep {
    /// Persistent notification metadata/attributes/cache for this exact model.
    pub fn notifications(&self) -> &NotificationState {
        &self.notifications
    }
    /// Mutable native notification state. Model keys and entity identity mappings
    /// must remain fixed while attributes and callback references are in use.
    pub fn notifications_mut(&mut self) -> &mut NotificationState {
        &mut self.notifications
    }
    /// Mutable notification fields of one live entity, initialized once. Marks,
    /// flags and user values persist; ownership/list queries use the arena graph.
    pub fn notification_topology_mut(&mut self, entity: EntityRef) -> Result<&mut Topology> {
        let key = object_ref(self, entity)?.key;
        Ok(self.notifications.topology.entry(key).or_default())
    }
    /// Build and own the real `BRep` spatial cache using existing geometry queries.
    pub fn build_notification_cache(&mut self, k: &Kernel) -> Result<()> {
        let cache = make_brep_cache(k, CacheSource::Brep(self), &arena_identity)?;
        self.notifications.spatial_cache = Some(cache);
        Ok(())
    }
}

/// Run the actual root notification chain: spatial changes first, topology base
/// propagation, attribute handlers, then the actual context's user/system hooks.
/// Callback statuses are ignored by the existing native-compatible object layer.
/// Errors retain already-applied native effects and restore the owning state;
/// this operation is not an import transaction or permission to repair geometry.
pub fn notify(
    k: &mut Kernel,
    b: &mut Brep,
    operation: NotifyOperation,
    data: [Option<ObjectRef>; 3],
) -> Result<()> {
    // A foreign root makes the need for a shared multi-model resolver explicit.
    // Entity-only references are model-local by contract; their equal indices
    // cannot be used to identify objects from another model.
    if data
        .iter()
        .flatten()
        .any(|object| object.type_code == BREP_TYPE && object.key != b.key)
    {
        // UNPORTED(topology.brep_p2): cross-model notification identity resolution.
        return Err(SolidError::unported(
            "cross-model SmBrep::Notify",
            "topology.brep_p2",
        ));
    }
    if b.notifications.spatial_cache.is_some() {
        let relevant = match operation {
            NotifyOperation::AddToBrep | NotifyOperation::RmFromBrep => {
                [data.first().copied().flatten(), None]
            }
            NotifyOperation::MergeInBrep => [data.get(1).copied().flatten(), None],
            NotifyOperation::SplitInBrep => [
                data.get(1).copied().flatten(),
                data.get(2).copied().flatten(),
            ],
            _ => [None, None],
        };
        if relevant
            .into_iter()
            .flatten()
            .any(|r| r.type_code == FILLET_EDGE_TYPE)
        {
            // UNPORTED(topology.brep_p2): actual subtype geometry resolution.
            return Err(SolidError::unported(
                "SmFilletEdge spatial notification",
                "topology.brep_p2",
            ));
        }
    }
    let mut state = std::mem::take(&mut b.notifications);
    let this = root_ref(b);
    let result = (|| {
        use super::{BrepCacheSlot, NotifyArg, NotifyServices};
        let mut args = [None; 3];
        let mut keys = BTreeMap::new();
        state.topology.entry(this.key).or_default();
        for (slot, object) in args.iter_mut().zip(data) {
            if let Some(object) = object {
                *slot = Some(if let Ok(entity) = local_entity(b, object) {
                    let registered = object_ref(b, entity)?;
                    state.topology.entry(registered.key).or_default();
                    keys.insert(entity, registered);
                    NotifyArg::Entity(entity)
                } else {
                    NotifyArg::Object(object)
                });
            }
        }
        // The generic upstream dispatcher remains the one algorithm. This
        // adapter supplies real owning state, distinct geometry-owner keys and
        // region access instead of a disposable/default environment.
        // The generic arena adapter uses its own identity convention. Resolve
        // through this call's validated owning registrations instead; an unknown
        // key is deliberately invalid so propagation cannot silently use defaults.
        let key_of = |entity| match entity {
            EntityRef::Brep(_) => this,
            _ => keys.get(&entity).copied().unwrap_or(ObjectRef {
                type_code: 0,
                key: ObjectKey(u64::MAX),
            }),
        };
        let mut env = crate::topology::edge::NotifyEnv {
            records: &mut state.topology,
            attributes: &mut state.attributes,
            context: None,
            key_of: &key_of,
        };
        let mut cache = BrepCacheSlot {
            kernel: k,
            cache: &mut state.spatial_cache,
        };
        super::notify_with_services(
            b,
            &mut env,
            Some(&mut cache),
            operation,
            args,
            NotifyServices {
                geometry_keys: Some(&arena_identity),
                regions: Some(&Regions(b)),
            },
        )
    })();
    b.notifications = state;
    result?;
    let [a, c, d] = data;
    crate::geom::object::notify(Some(&mut k.context), operation, this, a, c, d);
    Ok(())
}

fn local_entity(b: &Brep, object: ObjectRef) -> Result<EntityRef> {
    let i = index(object)?;
    let e = match object.type_code {
        REGION_TYPE => EntityRef::Region(RegionId::from_index(i)),
        SHELL_TYPE => EntityRef::Shell(crate::topology::ShellId::from_index(i)),
        FACE_TYPE => EntityRef::Face(FaceId::from_index(i)),
        LOOP_TYPE => EntityRef::Loop(crate::topology::LoopId::from_index(i)),
        EDGE_TYPE => EntityRef::Edge(EdgeId::from_index(i)),
        VERTEX_TYPE => EntityRef::Vertex(VertexId::from_index(i)),
        FACEUSE_TYPE => EntityRef::Faceuse(crate::topology::FaceuseId::from_index(i)),
        LOOPUSE_TYPE => EntityRef::Loopuse(crate::topology::LoopuseId::from_index(i)),
        EDGEUSE_TYPE => EntityRef::Edgeuse(crate::topology::EdgeuseId::from_index(i)),
        VERTEXUSE_TYPE => EntityRef::Vertexuse(crate::topology::VertexuseId::from_index(i)),
        _ => {
            return Err(SolidError::invalid_input(
                OP,
                "object is not local topology",
            ))
        }
    };
    object_ref(b, e)?;
    Ok(e)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::attribute::{Attribute, Behavior};
    use crate::geom::context::NotifyCallback;
    use crate::math::Point3;
    use crate::topology::Vertex;
    use std::sync::{Arc, Mutex};

    #[derive(Debug)]
    struct Capture(u8, Arc<Mutex<Vec<u8>>>);
    impl NotifyCallback for Capture {
        fn execute(
            &mut self,
            _: NotifyOperation,
            _: ObjectRef,
            _: Option<ObjectRef>,
            _: Option<ObjectRef>,
            _: Option<ObjectRef>,
        ) -> Result<()> {
            self.1.lock().unwrap().push(self.0);
            Err(SolidError::sm_err("callback status ignored"))
        }
    }
    fn model() -> Brep {
        let mut b = Brep::new();
        b.add_vertex_record(Vertex {
            point: Point3::new(1., 2., 3.),
            ..Vertex::default()
        })
        .unwrap();
        b
    }
    fn root_attribute(b: &mut Brep) {
        let h = b
            .notifications
            .attributes
            .insert(Attribute::long(90001, 42, Behavior::Copy))
            .unwrap();
        b.notifications.attributes.add_attribute(b.key, h).unwrap();
    }

    #[test]
    fn pre_edit_drops_real_cache_and_delivers_both_callbacks_despite_error_statuses() {
        let mut k = Kernel::default();
        let mut b = model();
        root_attribute(&mut b);
        b.build_notification_cache(&k).unwrap();
        assert!(b.notifications.spatial_cache.is_some());
        let events = Arc::new(Mutex::new(Vec::new()));
        k.context
            .set_user_notify_callback(Some(Box::new(Capture(0, Arc::clone(&events)))));
        k.context
            .set_sys_notify_callback(Some(Box::new(Capture(1, Arc::clone(&events)))));
        let this = root_ref(&b);
        notify(
            &mut k,
            &mut b,
            NotifyOperation::PreEdit,
            [Some(this), None, None],
        )
        .unwrap();
        assert!(b.notifications.spatial_cache.is_none());
        assert_eq!(*events.lock().unwrap(), vec![0, 1]);
        assert!(b
            .notifications
            .attributes
            .find_attribute(b.key, 90001)
            .is_some());
    }

    #[test]
    fn unsupported_foreign_root_does_not_consume_the_local_state() {
        let mut k = Kernel::default();
        let mut b = model();
        root_attribute(&mut b);
        b.build_notification_cache(&k).unwrap();
        let foreign = ObjectRef {
            type_code: BREP_TYPE,
            key: ObjectKey(999),
        };
        assert!(notify(
            &mut k,
            &mut b,
            NotifyOperation::PreEdit,
            [Some(foreign), None, None]
        )
        .unwrap_err()
        .is_unported());
        assert!(b.notifications.spatial_cache.is_some());
        assert!(b
            .notifications
            .attributes
            .find_attribute(b.key, 90001)
            .is_some());
    }

    #[test]
    fn propagation_error_restores_the_owning_attribute_and_metadata_store() {
        let mut k = Kernel::default();
        let mut b = model();
        root_attribute(&mut b);
        let v = object_ref(&b, EntityRef::Vertex(VertexId::from_index(0))).unwrap();
        let absent = ObjectRef {
            type_code: VERTEX_TYPE,
            key: ObjectKey((16021_u64 << 32) | 0x63),
        };
        assert!(notify(
            &mut k,
            &mut b,
            NotifyOperation::SplitInBrep,
            [Some(absent), Some(v), Some(v)]
        )
        .is_err());
        assert!(b
            .notifications
            .attributes
            .find_attribute(b.key, 90001)
            .is_some());
        assert!(b.notifications.topology.contains_key(&v.key));
    }

    #[test]
    fn value_snapshot_clone_does_not_copy_a_live_spatial_cache() {
        let k = Kernel::default();
        let mut b = model();
        root_attribute(&mut b);
        b.build_notification_cache(&k).unwrap();
        let copy = b.clone();
        assert!(b.notifications.spatial_cache.is_some());
        assert!(copy.notifications.spatial_cache.is_none());
        assert_eq!(copy.notifications.attributes, b.notifications.attributes);
    }

    #[test]
    fn an_arena_only_producer_cannot_silently_drop_configured_owning_attributes() {
        let mut b = model();
        root_attribute(&mut b);
        let mut external = super::super::ArenaNotifyState::new(&b);
        let this = super::super::NotifyArg::Object(root_ref(&b));
        let error = super::super::notify(
            &b,
            &mut external.env(None),
            None,
            NotifyOperation::PreEdit,
            [Some(this), None, None],
        )
        .unwrap_err();
        assert!(error.is_unported());
        assert!(b
            .notifications
            .attributes
            .find_attribute(b.key, 90001)
            .is_some());
    }
}
