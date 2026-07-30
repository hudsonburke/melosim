# Future Considerations

## Shipyard ECS

Shipyard (by Catherine West / kyren) provides component queries, generational entity IDs, sparse set iteration, and system scheduling. We evaluated it against melosim's current architecture (AnyMap of `Vec<Option<T>>`, dense `EntityID(u32)`, explicit cross-entity references).

**What Shipyard would add:** Combined component iteration (`View<A>` + `View<B>` filter to entities with both), generational safety on entity deletion, better iteration (sparse sets skip Nones), and workloads for system scheduling.

**What it would cost:** Opaque EntityId (not u32) breaks direct Vec indexing. Derive macros on all components. Closure-based `run()` API changes how systems are written. Double indirection for lookup (sparse array → dense array).

**Why we're not adopting it now:** At 200-entity biomechanics models with static entity sets and explicit cross-entity references (HingeJoint.body_a, Frame.parent), manual iteration with `world.get::<T>(entity)` lookups is simple, fast, and debuggable. Shipyard's query power shines at 10K+ entities with dynamic component addition/removal during simulation.

**When to reconsider:** If the FK solver or muscle force solver develops complex multi-component iteration patterns that manual iteration can't express cleanly, or if parallel system execution becomes necessary for solver performance, revisit Shipyard. Keep explicit cross-entity references (components store EntityID fields) — this makes any future migration cleaner since you're replacing storage and iteration, not rearchitecting entity relationships.

## SparseSet storage

Shipyard uses SparseSet internally — a dense packed array of components plus a sparse index array. This provides O(1) lookup, O(min(n,m)) combined iteration, and skip-Nones iteration. We evaluated replacing `Vec<Option<T>>` with a custom SparseSet. Not adopted because: the performance gain is negligible at 200 entities, direct array indexing (`vec[id.0 as usize]`) is simpler and GPU-mappable, and the double indirection (sparse → dense → component) is unnecessary overhead for our scale. Revisit if entity counts grow to thousands or if solver iteration becomes a bottleneck.

## Simulation

FK solver, muscle force computation, and wrapping solver are deferred to when simulation work begins. The current focus is the editing workflow (import → attach → route → export). Simulation will use the same plugin system (inventory-based `System` registration) and can be added incrementally.

## FlatWorld

The FlatWorld / freeze pattern was designed for GPU-ready simulation snapshots. It was removed to simplify the architecture while focusing on editing. If simulation is added later, a similar pattern (dense `Vec<Option<T>>` per component type, indexed by EntityID) can be reintroduced — the storage layout is already correct for it.
