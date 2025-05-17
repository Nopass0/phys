use crate::bodies::RigidBody;
use crate::core::{BodyStorage, EventQueue};
use crate::core::events::{CollisionEvent, CollisionEventType};
use crate::collision::{
    CollisionDetector,
    broad_phase::BruteForceBroadPhase,
    narrow_phase::{SphereNarrowPhase, GjkNarrowPhase},
    contact_generator::GjkContactGenerator,
    contact_solver::SequentialImpulseSolver,
    collision_filter::GroupMaskFilter,
};

/// Detect collisions and resolve contacts using the engine's collision pipeline.
///
/// This function performs the following steps:
/// 1. Build a `CollisionDetector` with broad phase, narrow phase and contact
///    generation algorithms.
/// 2. Update the detector with the current body states to obtain contact
///    manifolds.
/// 3. Resolve the contacts using a sequential impulse solver.
/// 4. Generate collision events based on the detector's collision states.
pub fn detect_collisions(bodies: &mut BodyStorage<RigidBody>, events: &mut EventQueue) {
    // Create the collision detector each frame. This keeps the implementation
    // self-contained and avoids storing additional state in `PhysicsWorld`.
    let mut detector = CollisionDetector::new(
        Box::new(BruteForceBroadPhase::new()),
        Box::new(SphereNarrowPhase::new(Box::new(GjkNarrowPhase::new()))),
        Box::new(GjkContactGenerator::new(4)),
        Box::new(GroupMaskFilter::new()),
    );

    // Update detector using an immutable borrow of the bodies.
    detector.update(&*bodies);

    // Retrieve contact manifolds produced this frame.
    let mut manifolds = detector.get_contact_manifolds().to_vec();

    // Resolve contacts using a sequential impulse solver.  The bias factor and
    // restitution threshold values mirror the defaults used elsewhere in the
    // engine.  A fixed time step is used here because the detector does not
    // have direct access to the world's configuration.
    let mut solver = SequentialImpulseSolver::new(0.2, 0.5);
    solver.prepare(&mut manifolds, bodies);
    let dt = 1.0 / 60.0;
    for _ in 0..10 {
        solver.solve_velocity(&mut manifolds, bodies, dt);
    }
    for _ in 0..10 {
        solver.solve_position(&mut manifolds, bodies, dt);
    }

    // Generate collision start and persist events.
    for state in detector.get_collision_states().values() {
        if state.is_new_collision() || state.is_persistent_collision() {
            let event_type = if state.is_new_collision() {
                CollisionEventType::Begin
            } else {
                CollisionEventType::Persist
            };

            let event = CollisionEvent {
                event_type,
                body_a: state.pair.body_a,
                body_b: state.pair.body_b,
                contacts: state.manifold.contacts.clone(),
                normal_impulse: None,
                tangent_impulse: None,
            };
            events.add_collision_event(event);
        }
    }

    // Generate collision end events.
    for state in detector.get_collision_states().values() {
        if state.is_collision_end() {
            let event = CollisionEvent {
                event_type: CollisionEventType::End,
                body_a: state.pair.body_a,
                body_b: state.pair.body_b,
                contacts: Vec::new(),
                normal_impulse: None,
                tangent_impulse: None,
            };
            events.add_collision_event(event);
        }
    }
}
