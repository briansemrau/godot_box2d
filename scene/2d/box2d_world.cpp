#include "box2d_world.h"

#include <core/engine.h>
#include <core/method_bind_ext.gen.inc>
#include <core/os/os.h>

#include <box2d/b2_collision.h>
#include <box2d/b2_time_of_impact.h>

#include "box2d_fixtures.h"
#include "box2d_joints.h"
#include "box2d_physics_body.h"

#include <chrono>
#include <string>

/**
* @author Brian Semrau
*/

void Box2DShapeQueryParameters::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_shape", "shape"), &Box2DShapeQueryParameters::set_shape);
	ClassDB::bind_method(D_METHOD("get_shape"), &Box2DShapeQueryParameters::get_shape);

	ClassDB::bind_method(D_METHOD("set_transform", "transform"), &Box2DShapeQueryParameters::set_transform);
	ClassDB::bind_method(D_METHOD("get_transform"), &Box2DShapeQueryParameters::get_transform);

	ClassDB::bind_method(D_METHOD("set_motion", "motion"), &Box2DShapeQueryParameters::set_motion);
	ClassDB::bind_method(D_METHOD("get_motion"), &Box2DShapeQueryParameters::get_motion);

	ClassDB::bind_method(D_METHOD("set_motion_rotation", "rotation"), &Box2DShapeQueryParameters::set_motion_rotation);
	ClassDB::bind_method(D_METHOD("get_motion_rotation"), &Box2DShapeQueryParameters::get_motion_rotation);

	ClassDB::bind_method(D_METHOD("set_motion_transform", "transform"), &Box2DShapeQueryParameters::set_motion_transform);
	ClassDB::bind_method(D_METHOD("get_motion_transform"), &Box2DShapeQueryParameters::get_motion_transform);

	ClassDB::bind_method(D_METHOD("set_motion_local_center", "local_center"), &Box2DShapeQueryParameters::set_motion_local_center);
	ClassDB::bind_method(D_METHOD("get_motion_local_center"), &Box2DShapeQueryParameters::get_motion_local_center);

	ClassDB::bind_method(D_METHOD("set_collision_layer", "collision_layer"), &Box2DShapeQueryParameters::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &Box2DShapeQueryParameters::get_collision_layer);

	ClassDB::bind_method(D_METHOD("set_collision_mask", "collision_mask"), &Box2DShapeQueryParameters::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &Box2DShapeQueryParameters::get_collision_mask);

	ClassDB::bind_method(D_METHOD("set_group_index", "group_index"), &Box2DShapeQueryParameters::set_group_index);
	ClassDB::bind_method(D_METHOD("get_group_index"), &Box2DShapeQueryParameters::get_group_index);

	ClassDB::bind_method(D_METHOD("set_exclude", "exclude"), &Box2DShapeQueryParameters::set_exclude);
	ClassDB::bind_method(D_METHOD("get_exclude"), &Box2DShapeQueryParameters::get_exclude);

	ClassDB::bind_method(D_METHOD("set_collide_with_bodies", "enable"), &Box2DShapeQueryParameters::set_collide_with_bodies);
	ClassDB::bind_method(D_METHOD("is_collide_with_bodies_enabled"), &Box2DShapeQueryParameters::is_collide_with_bodies_enabled);

	ClassDB::bind_method(D_METHOD("set_collide_with_sensors", "enable"), &Box2DShapeQueryParameters::set_collide_with_sensors);
	ClassDB::bind_method(D_METHOD("is_collide_with_sensors_enabled"), &Box2DShapeQueryParameters::is_collide_with_sensors_enabled);

	ClassDB::bind_method(D_METHOD("set_ignore_rigid", "enable"), &Box2DShapeQueryParameters::set_ignore_rigid);
	ClassDB::bind_method(D_METHOD("is_ignoring_rigid"), &Box2DShapeQueryParameters::is_ignoring_rigid);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_2D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_2D_PHYSICS), "set_collision_mask", "get_collision_mask");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "group_index"), "set_group_index", "get_group_index");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "exclude", PROPERTY_HINT_NONE, itos(Variant::INT) + ":"), "set_exclude", "get_exclude");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "motion"), "set_motion", "get_motion");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "motion_rotation"), "set_motion_rotation", "get_motion_rotation");
	ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM2D, "motion_transform"), "set_motion_transform", "get_motion_transform");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "local_center"), "set_motion_local_center", "get_motion_local_center");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "shape", PROPERTY_HINT_RESOURCE_TYPE, "Box2DShape"), "set_shape", "get_shape");
	ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM2D, "transform"), "set_transform", "get_transform");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collide_with_bodies"), "set_collide_with_bodies", "is_collide_with_bodies_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collide_with_sensors"), "set_collide_with_sensors", "is_collide_with_sensors_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "ignore_rigid"), "set_ignore_rigid", "is_ignoring_rigid");
}

const b2Filter &Box2DShapeQueryParameters::_get_filter() const {
	return parameters.filter;
}

Set<const Box2DPhysicsBody *> Box2DShapeQueryParameters::_get_exclude() const {
	return parameters.exclude;
}

void Box2DShapeQueryParameters::set_shape(const RES &p_shape_ref) {
	ERR_FAIL_COND(p_shape_ref.is_null());
	shape_ref = p_shape_ref;
}

RES Box2DShapeQueryParameters::get_shape() const {
	return shape_ref;
}

void Box2DShapeQueryParameters::set_transform(const Transform2D &p_transform) {
	ERR_FAIL_COND_MSG(p_transform.get_scale() != Size2(1, 1), "Box2DShapeQueryParameters does not support scaled transforms.");
	parameters.transform = p_transform;
}

Transform2D Box2DShapeQueryParameters::get_transform() const {
	return parameters.transform;
}

void Box2DShapeQueryParameters::set_motion(const Vector2 &p_motion) {
	parameters.motion = p_motion;
}

Vector2 Box2DShapeQueryParameters::get_motion() const {
	return parameters.motion;
}

void Box2DShapeQueryParameters::set_motion_rotation(float p_rotation) {
	parameters.rotation = p_rotation;
}

float Box2DShapeQueryParameters::get_motion_rotation() const {
	return parameters.rotation;
}

void Box2DShapeQueryParameters::set_motion_transform(const Transform2D &p_transform) {
	ERR_FAIL_COND_MSG(p_transform.get_scale() != Size2(1, 1), "Box2DShapeQueryParameters does not support scaled motion transforms.");
	parameters.motion = p_transform.get_origin();
	parameters.rotation = p_transform.get_rotation();
}

Transform2D Box2DShapeQueryParameters::get_motion_transform() const {
	return Transform2D(parameters.rotation, parameters.motion);
}

void Box2DShapeQueryParameters::set_motion_local_center(const Vector2 &p_local_center) {
	parameters.local_center = p_local_center;
}

Vector2 Box2DShapeQueryParameters::get_motion_local_center() const {
	return parameters.local_center;
}

void Box2DShapeQueryParameters::set_collision_layer(int p_layer) {
	parameters.filter.categoryBits = p_layer;
}

int Box2DShapeQueryParameters::get_collision_layer() const {
	return parameters.filter.categoryBits;
}

void Box2DShapeQueryParameters::set_collision_mask(int p_collision_mask) {
	parameters.filter.maskBits = p_collision_mask;
}

int Box2DShapeQueryParameters::get_collision_mask() const {
	return parameters.filter.maskBits;
}

void Box2DShapeQueryParameters::set_group_index(int p_group_index) {
	parameters.filter.groupIndex = p_group_index;
}

int Box2DShapeQueryParameters::get_group_index() const {
	return parameters.filter.groupIndex;
}

void Box2DShapeQueryParameters::set_exclude(const Array &p_exclude) {
	parameters.exclude.clear();
	for (int i = 0; i < p_exclude.size(); i++) {
		Object *obj = ObjectDB::get_instance(ObjectID(p_exclude[i]));
		Box2DPhysicsBody *node = Object::cast_to<Box2DPhysicsBody>(obj);
		if (node)
			parameters.exclude.insert(node);
	}
}

Array Box2DShapeQueryParameters::get_exclude() const {
	Array ret;
	ret.resize(parameters.exclude.size());
	int idx = 0;
	for (Set<const Box2DPhysicsBody *>::Element *E = parameters.exclude.front(); E; E = E->next()) {
		ret[idx] = int64_t(E->get()->get_instance_id());
	}
	return ret;
}

void Box2DShapeQueryParameters::set_collide_with_bodies(bool p_enable) {
	parameters.collide_with_bodies = p_enable;
}

bool Box2DShapeQueryParameters::is_collide_with_bodies_enabled() const {
	return parameters.collide_with_bodies;
}

void Box2DShapeQueryParameters::set_collide_with_sensors(bool p_enable) {
	parameters.collide_with_sensors = p_enable;
}

bool Box2DShapeQueryParameters::is_collide_with_sensors_enabled() const {
	return parameters.collide_with_sensors;
}

void Box2DShapeQueryParameters::set_ignore_rigid(bool p_enable) {
	parameters.ignore_dynamic = p_enable;
}

bool Box2DShapeQueryParameters::is_ignoring_rigid() const {
	return parameters.ignore_dynamic;
}

void Box2DWorld::SayGoodbye(b2Joint *joint) {
	joint->GetUserData().owner->on_b2Joint_destroyed();
}

void Box2DWorld::SayGoodbye(b2Fixture *fixture) {
	fixture->GetUserData().owner->on_b2Fixture_destroyed(fixture);
}

bool Box2DWorld::ShouldCollide(b2Fixture *fixtureA, b2Fixture *fixtureB) {
	// Default Box2D contact filtering

	const b2Filter &filterA = fixtureA->GetFilterData();
	const b2Filter &filterB = fixtureB->GetFilterData();

	bool collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;

	if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0) {
		collide = filterA.groupIndex > 0;
	}

	// Custom contact filtering

	if (!collide) {
		return false;
	}

	// TODO Most bodies won't use explicit exclusions. Can/should this be optimized?

	// Check for fixture exclusions
	Box2DFixture *const &ownerA = fixtureA->GetUserData().owner;
	Box2DFixture *const &ownerB = fixtureB->GetUserData().owner;
	if (ownerA->filtered.has(ownerB) || ownerB->filtered.has(ownerA)) {
		return false;
	}

	// Check for body exclusions
	Box2DPhysicsBody *const &bodyA = Object::cast_to<Box2DPhysicsBody>(ownerA->owner_node);
	Box2DPhysicsBody *const &bodyB = Object::cast_to<Box2DPhysicsBody>(ownerB->owner_node);
	if (bodyA && bodyB) {
		if ((ownerA->accept_body_collision_exceptions && bodyA->filtered.has(bodyB)) || (ownerB->accept_body_collision_exceptions && bodyB->filtered.has(bodyA))) {
			return false;
		}
	}

	// TODO should we bother to let bodies exclude fixtures?
	return true;
}

inline void Box2DWorld::try_buffer_contact(b2Contact *contact, int i) {
	if (contact->GetFixtureA()->IsSensor() || contact->GetFixtureB()->IsSensor()) {
		return;
	}

	Box2DFixture *fnode_a = contact->GetFixtureA()->GetUserData().owner;
	Box2DFixture *fnode_b = contact->GetFixtureB()->GetUserData().owner;
	Box2DCollisionObject *body_a = fnode_a->owner_node;
	Box2DCollisionObject *body_b = fnode_b->owner_node;

	const bool monitoringA = fnode_a->owner_node->_is_contact_monitor_enabled();
	const bool monitoringB = fnode_b->owner_node->_is_contact_monitor_enabled();

	// Only buffer contacts that are being monitored, if contact monitor report count isn't exceeded

	// If the manifold is already buffered, make sure to buffer all points in the manifold (ignoring whether the monitors have capacity)
	// We do this so that we don't have to worry about managing half-buffered manifolds.

	const bool hasCapacityA = monitoringA && (body_a->contact_monitor->contacts.size() < body_a->max_contacts_reported);
	const bool hasCapacityB = monitoringB && (body_b->contact_monitor->contacts.size() < body_b->max_contacts_reported);

	// Note: The b2Contact pointer address seems to be the only way to create a unique key. Please prove me wrong.
	ContactBufferManifold *buffer_manifold = contact_buffer.getptr(reinterpret_cast<uint64_t>(contact));

	if (hasCapacityA || hasCapacityB || buffer_manifold) {
		if (!buffer_manifold) {
			buffer_manifold = &contact_buffer.set(reinterpret_cast<uint64_t>(contact), ContactBufferManifold())->value();
		}

		// Init contact
		Box2DContactPoint c;
		c.id = ++next_contact_id;
		c.fixture_a = fnode_a;
		c.fixture_b = fnode_b;

		buffer_manifold->insert(c, i);

		// Buffer again into monitoring node
		if (hasCapacityA) {
			auto contacts = &fnode_a->owner_node->contact_monitor->contacts;
			contacts->insert(c);
		}
		if (hasCapacityB) {
			auto contacts = &fnode_b->owner_node->contact_monitor->contacts;
			contacts->insert(c);
		}
	}
}

void Box2DWorld::BeginContact(b2Contact *contact) {
	Box2DFixture *fnode_a = contact->GetFixtureA()->GetUserData().owner;
	Box2DFixture *fnode_b = contact->GetFixtureB()->GetUserData().owner;
	Box2DCollisionObject *body_a = fnode_a->owner_node;
	Box2DCollisionObject *body_b = fnode_b->owner_node;

	const bool monitoringA = fnode_a->owner_node->_is_contact_monitor_enabled();
	const bool monitoringB = fnode_b->owner_node->_is_contact_monitor_enabled();

	// Deliver signals to bodies with monitoring enabled
	// Only emit body_entered once per body. Begin/EndContact are called for each b2Fixture.
	// Similar case for Box2DFixture nodes. One Box2DFixture may have several b2Fixtures.
	if (monitoringA) {
		int *body_count_ptr = body_a->contact_monitor->entered_objects.getptr(body_b->get_instance_id());
		if (!body_count_ptr) {
			body_count_ptr = &(body_a->contact_monitor->entered_objects.set(body_b->get_instance_id(), 0)->value());
		}
		++(*body_count_ptr);

		if (*body_count_ptr == 1) {
			object_entered_queue.enqueue(body_a, body_b);
		}

		int *fix_count_ptr = body_a->contact_monitor->entered_objects.getptr(fnode_b->get_instance_id());
		if (!fix_count_ptr) {
			fix_count_ptr = &(body_a->contact_monitor->entered_objects.set(fnode_b->get_instance_id(), 0)->value());
		}
		++(*fix_count_ptr);

		if (*fix_count_ptr == 1) {
			fixture_entered_queue.enqueue(body_a, fnode_b, fnode_a);
		}
	}
	if (monitoringB) {
		int *body_count_ptr = body_b->contact_monitor->entered_objects.getptr(body_a->get_instance_id());
		if (!body_count_ptr) {
			body_count_ptr = &(body_b->contact_monitor->entered_objects.set(body_a->get_instance_id(), 0)->value());
		}
		++(*body_count_ptr);

		if (*body_count_ptr == 1) {
			object_entered_queue.enqueue(body_b, body_a);
		}

		int *fix_count_ptr = body_b->contact_monitor->entered_objects.getptr(fnode_a->get_instance_id());
		if (!fix_count_ptr) {
			fix_count_ptr = &(body_b->contact_monitor->entered_objects.set(fnode_a->get_instance_id(), 0)->value());
		}
		++(*fix_count_ptr);

		if (*fix_count_ptr == 1) {
			fixture_entered_queue.enqueue(body_b, fnode_a, fnode_b);
		}
	}
}

void Box2DWorld::EndContact(b2Contact *contact) {
	Box2DFixture *fnode_a = contact->GetFixtureA()->GetUserData().owner;
	Box2DFixture *fnode_b = contact->GetFixtureB()->GetUserData().owner;
	Box2DCollisionObject *body_a = fnode_a->owner_node;
	Box2DCollisionObject *body_b = fnode_b->owner_node;

	const bool monitoringA = fnode_a->owner_node->_is_contact_monitor_enabled();
	const bool monitoringB = fnode_b->owner_node->_is_contact_monitor_enabled();

	// EndContact may occur outside of timestep. No need to defer signal calls when world is unlocked.
	bool queue_inout = world->IsLocked();

	// Deliver signals to bodies with contact monitoring enabled
	if (monitoringA) {
		int *body_count_ptr = body_a->contact_monitor->entered_objects.getptr(body_b->get_instance_id());
		--(*body_count_ptr);

		if ((*body_count_ptr) == 0) {
			body_a->contact_monitor->entered_objects.erase(body_b->get_instance_id());
			if (queue_inout)
				object_exited_queue.enqueue(body_a, body_b);
			else
				object_exited_queue.call_immediate(body_a, body_b);
		}

		int *fix_count_ptr = body_a->contact_monitor->entered_objects.getptr(fnode_b->get_instance_id());
		--(*fix_count_ptr);

		if ((*fix_count_ptr) == 0) {
			body_a->contact_monitor->entered_objects.erase(fnode_b->get_instance_id());
			if (queue_inout)
				fixture_exited_queue.enqueue(body_a, fnode_b, fnode_a);
			else
				fixture_exited_queue.call_immediate(body_a, fnode_b, fnode_a);
		}
	}
	if (monitoringB) {
		int *body_count_ptr = body_b->contact_monitor->entered_objects.getptr(body_a->get_instance_id());
		--(*body_count_ptr);

		if ((*body_count_ptr) == 0) {
			body_b->contact_monitor->entered_objects.erase(body_a->get_instance_id());
			if (queue_inout)
				object_exited_queue.enqueue(body_b, body_a);
			else
				object_exited_queue.call_immediate(body_b, body_a);
		}

		int *fix_count_ptr = body_b->contact_monitor->entered_objects.getptr(fnode_a->get_instance_id());
		--(*fix_count_ptr);

		if ((*fix_count_ptr) == 0) {
			body_b->contact_monitor->entered_objects.erase(fnode_a->get_instance_id());
			if (queue_inout)
				fixture_exited_queue.enqueue(body_b, fnode_a, fnode_b);
			else
				fixture_exited_queue.call_immediate(body_b, fnode_a, fnode_b);
		}
	}

	// Clean up all buffered contacts in the manifold
	ContactBufferManifold *buffer_manifold = contact_buffer.getptr(reinterpret_cast<uint64_t>(contact));

	if (buffer_manifold) {
		for (int i = 0; i < buffer_manifold->count; ++i) {
			Box2DContactPoint *c_ptr = &buffer_manifold->points[i];

			if (c_ptr->fixture_a->owner_node->_is_contact_monitor_enabled()) {
				// TODO lock/unlock
				c_ptr->fixture_a->owner_node->contact_monitor->contacts.erase(*c_ptr);
			}
			if (c_ptr->fixture_b->owner_node->_is_contact_monitor_enabled()) {
				// TODO lock/unlock
				c_ptr->fixture_b->owner_node->contact_monitor->contacts.erase(*c_ptr);
			}
		}

		ERR_FAIL_COND(!contact_buffer.erase(reinterpret_cast<uint64_t>(contact)));
	}
}

void Box2DWorld::PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {
	b2PointState state1[2], state2[2];
	b2GetPointStates(state1, state2, oldManifold, contact->GetManifold());

	Box2DFixture *fnode_a = contact->GetFixtureA()->GetUserData().owner;
	Box2DFixture *fnode_b = contact->GetFixtureB()->GetUserData().owner;

	ContactBufferManifold *buffer_manifold = contact_buffer.getptr(reinterpret_cast<uint64_t>(contact));

	if (unlikely(flag_rescan_contacts_monitored) && !buffer_manifold) {
		// Buffer a contact that only started being monitored after it transitioned from b2_addState
		for (int i = 0; i < b2_maxManifoldPoints; ++i) {
			if (state1[i] == b2PointState::b2_persistState) {
				try_buffer_contact(contact, i);
			}
		}
		buffer_manifold = contact_buffer.getptr(reinterpret_cast<uint64_t>(contact)); // possible optimization: try_buffer_contact could return buffer_manifold ptr
	}

	// Handle removed/added points within the manifold
	for (int i = b2_maxManifoldPoints - 1; i >= 0; --i) {
		if (state1[i] == b2PointState::b2_removeState) {
			// Remove this contact
			if (buffer_manifold && i < buffer_manifold->count) {
				Box2DContactPoint *c_ptr = &buffer_manifold->points[i];

				if (c_ptr->fixture_a->owner_node->_is_contact_monitor_enabled()) {
					// TODO lock/unlock
					c_ptr->fixture_a->owner_node->contact_monitor->contacts.erase(*c_ptr);
				}
				if (c_ptr->fixture_b->owner_node->_is_contact_monitor_enabled()) {
					// TODO lock/unlock
					c_ptr->fixture_b->owner_node->contact_monitor->contacts.erase(*c_ptr);
				}

				buffer_manifold->remove(i);
				if (buffer_manifold->count == 0) {
					ERR_FAIL_COND(!contact_buffer.erase(reinterpret_cast<uint64_t>(contact)));
				}
			}
		}
	}
	for (int i = 0; i < b2_maxManifoldPoints; ++i) {
		if (state2[i] == b2PointState::b2_addState) {
			try_buffer_contact(contact, i);
		}
	}

	if (buffer_manifold) {
		// Only handle the first PreSolve for this contact this step (don't overwrite initial impact_velocity, world_pos)
		for (int i = 0; i < buffer_manifold->count; ++i) {
			Box2DContactPoint *c_ptr = &buffer_manifold->points[i];

			if (c_ptr->solves == 0) {
				c_ptr->solves += 1;

				b2WorldManifold worldManifold;
				contact->GetWorldManifold(&worldManifold);

				Vector2 manifold_norm = Vector2(worldManifold.normal.x, worldManifold.normal.y);
				c_ptr->normal = manifold_norm;

				c_ptr->world_pos = b2_to_gd(worldManifold.points[i]);

				// Reset accumulated values
				c_ptr->normal_impulse = 0.0f;
				c_ptr->tangent_impulse = Vector2();

				b2Vec2 point = worldManifold.points[i];
				b2Vec2 relV = contact->GetFixtureB()->GetBody()->GetLinearVelocityFromWorldPoint(point);
				relV -= contact->GetFixtureA()->GetBody()->GetLinearVelocityFromWorldPoint(point);

				c_ptr->impact_velocity = b2_to_gd(relV);
			}
		}
	}
}

void Box2DWorld::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {
	const Box2DFixture *fnode_a = contact->GetFixtureA()->GetUserData().owner;
	const Box2DFixture *fnode_b = contact->GetFixtureB()->GetUserData().owner;

	const bool monitoringA = fnode_a->owner_node->_is_contact_monitor_enabled();
	const bool monitoringB = fnode_b->owner_node->_is_contact_monitor_enabled();
	if (monitoringA || monitoringB) {
		b2WorldManifold worldManifold;
		contact->GetWorldManifold(&worldManifold);

		ContactBufferManifold *buffer_manifold = contact_buffer.getptr(reinterpret_cast<uint64_t>(contact));

		if (buffer_manifold) {
			for (int i = 0; i < buffer_manifold->count; ++i) {
				Box2DContactPoint *c_ptr = &buffer_manifold->points[i];

				Vector2 manifold_tan = c_ptr->normal.rotated(Math_PI * 0.5);
				// TODO test: should impulse be accumulated (relevant to TOI solve), or does Box2D accumulate them itself?
				c_ptr->normal_impulse += impulse->normalImpulses[i];
				c_ptr->tangent_impulse += manifold_tan * impulse->tangentImpulses[i];

				// Update contacts buffered in listening nodes
				if (monitoringA) {
					//fnode_a->body_node->contact_monitor.locked = true; TODO
					auto contacts = &fnode_a->owner_node->contact_monitor->contacts;
					int idx = contacts->find(*c_ptr);
					if (idx >= 0)
						(*contacts)[idx] = (*c_ptr);
					//fnode_a->body_node->contact_monitor.locked = false;
				}
				if (monitoringB) {
					// Invert contact so A is always owned by the monitor
					Box2DContactPoint cB = c_ptr->flipped_a_b();

					//fnode_b->body_node->contact_monitor.locked = true; TODO
					auto contacts = &fnode_b->owner_node->contact_monitor->contacts;
					int idx = contacts->find(cB);
					if (idx >= 0)
						(*contacts)[idx] = (cB);
					//fnode_b->body_node->contact_monitor.locked = false;
				}
			}
		}
	}
}

void Box2DWorld::create_b2World() {
	if (!world) {
		world = memnew(b2World(gd_to_b2(gravity)));

		world->SetDestructionListener(this);
		world->SetContactFilter(this);
		world->SetContactListener(this);

		Set<Box2DCollisionObject *>::Element *body = body_owners.front();
		while (body) {
			body->get()->on_parent_created(this);
			body = body->next();
		}
		Set<Box2DJoint *>::Element *joint = joint_owners.front();
		while (joint) {
			joint->get()->on_parent_created(this);
			joint = joint->next();
		}

		if (!Engine::get_singleton()->is_editor_hint()) {
			set_physics_process_internal(true);
		}
	}
}

void Box2DWorld::destroy_b2World() {
	if (world) {
		// Nullify bodies, joints, and fixtures so that nothing calls their b2 Destroy func.
		// Normally our wrapper nodes call b2World.DestroyX, but that seems to be slow (vaguely tested, could be wrong) when doing them all at once, in indeterminant order.
		// Instead we let the b2 allocators free themselves.

		b2Body *body = world->GetBodyList();
		while (body) {
			// nullify body
			body->GetUserData().owner->body = NULL;

			// nullify fixtures
			b2Fixture *fixture = body->GetFixtureList();
			while (fixture) {
				fixture->GetUserData().owner->fixtures.clear();
				fixture = fixture->GetNext();
			}

			body = body->GetNext();
		}

		b2Joint *joint = world->GetJointList();
		while (joint) {
			// nullify joint
			joint->GetUserData().owner->joint = NULL;

			joint = joint->GetNext();
		}

		memdelete(world);
		world = NULL;
	}
}

void Box2DWorld::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PREDELETE: {
			destroy_b2World();
		} break;

		case NOTIFICATION_ENTER_TREE: {
			// Create world
			create_b2World();
		} break;

		case NOTIFICATION_EXIT_TREE: {
			// Don't destroy world. It could be exiting/entering.
			// World should be destroyed in PREDELETE if node is being freed.
		} break;

		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (auto_step) {
				float time = get_physics_process_delta_time();
				step(time);
			}
		} break;
	}
}

void Box2DWorld::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &Box2DWorld::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &Box2DWorld::get_gravity);
	ClassDB::bind_method(D_METHOD("set_auto_step", "auto_setp"), &Box2DWorld::set_auto_step);
	ClassDB::bind_method(D_METHOD("get_auto_step"), &Box2DWorld::get_auto_step);

	// TODO should default collision_mask really be 0x7FFFFFFF? This is copied from Godot physics API
	ClassDB::bind_method(D_METHOD("intersect_point", "point", "max_results", "exclude", "collision_mask", "collide_with_bodies", "collide_with_sensors", "collision_layer", "group"), &Box2DWorld::intersect_point, DEFVAL(32), DEFVAL(Array()), DEFVAL(0x7FFFFFFF), DEFVAL(true), DEFVAL(false), DEFVAL(0x0), DEFVAL(0));
	ClassDB::bind_method(D_METHOD("intersect_ray", "from", "to", "exclude", "collision_mask", "collide_with_bodies", "collide_with_sensors", "collision_layer", "group"), &Box2DWorld::intersect_ray, DEFVAL(Array()), DEFVAL(0x7FFFFFFF), DEFVAL(true), DEFVAL(false), DEFVAL(0x0), DEFVAL(0));
	ClassDB::bind_method(D_METHOD("intersect_shape", "query", "max_results"), &Box2DWorld::intersect_shape, DEFVAL(32));
	ClassDB::bind_method(D_METHOD("cast_motion", "query"), &Box2DWorld::cast_motion);
	//ClassDB::bind_method(D_METHOD("collide_shape", "shape", "max_results"), &PhysicsDirectSpaceState2D::_collide_shape, DEFVAL(32));

	ClassDB::bind_method(D_METHOD("query_aabb", "aabb", "target", "method"), &Box2DWorld::query_aabb);
	ClassDB::bind_method(D_METHOD("raycast", "from", "to", "target", "method"), &Box2DWorld::raycast);

	ClassDB::bind_method(D_METHOD("body_test_motion", "body", "from", "motion", "infinite_inertia", "result"), &Box2DWorld::_body_test_motion_binding, DEFVAL(Variant()));

	ClassDB::bind_method(D_METHOD("step", "delta"), &Box2DWorld::step);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "gravity"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_step"), "set_auto_step", "get_auto_step");
}

inline void _get_aabb_from_shapes(const Vector<const b2Shape *> &p_b2shapes, const b2Transform &p_xform, b2AABB &r_aabb) {
	p_b2shapes[0]->ComputeAABB(&r_aabb, p_xform, 0);
	for (int i = 0; i < p_b2shapes.size(); ++i) {
		const b2Shape *test_b2shape = p_b2shapes[i];
		for (int child_index = 0; child_index < test_b2shape->GetChildCount(); ++child_index) {
			b2AABB aabb;
			test_b2shape->ComputeAABB(&aabb, p_xform, child_index);
			r_aabb.Combine(aabb);
		}
	}
}

inline void _get_aabb_from_shapes_with_motion(const Vector<const b2Shape *> &p_b2shapes, const b2Transform &p_xform_t0, const b2Transform &p_xform_t1, b2AABB &r_aabb) {
	if (p_b2shapes.size() == 0)
		return;

	p_b2shapes[0]->ComputeAABB(&r_aabb, p_xform_t0, 0);

	for (int i = 0; i < p_b2shapes.size(); ++i) {
		const b2Shape *cast_b2shape = p_b2shapes[i];

		for (int child_index = 0; child_index < cast_b2shape->GetChildCount(); ++child_index) {
			b2AABB aabb_t0, aabb_t1;
			cast_b2shape->ComputeAABB(&aabb_t0, p_xform_t0, child_index);
			cast_b2shape->ComputeAABB(&aabb_t1, p_xform_t1, child_index);

			r_aabb.Combine(aabb_t0);
			r_aabb.Combine(aabb_t1);
		}
	}
}

struct IntersectionManifoldResult {
	b2Manifold manifold;
	bool flipped;

	inline bool intersecting() const {
		return manifold.pointCount > 0;
	}
};

IntersectionManifoldResult _evaluate_intersection_manifold(const b2Shape *p_shapeA, const int p_child_index_A, const b2Transform &p_xfA, const b2Shape *p_shapeB, const int p_child_index_B, const b2Transform &p_xfB) {
	b2Manifold manifold{};
	bool flipped = false;

	// Convert chains to edges
	b2EdgeShape shapeA_as_edge;
	if (p_shapeA->GetType() == b2Shape::Type::e_chain) {
		static_cast<const b2ChainShape *>(p_shapeA)->GetChildEdge(&shapeA_as_edge, p_child_index_A);
		p_shapeA = &shapeA_as_edge;
	}

	b2EdgeShape shapeB_as_edge;
	if (p_shapeB->GetType() == b2Shape::Type::e_chain) {
		static_cast<const b2ChainShape *>(p_shapeB)->GetChildEdge(&shapeB_as_edge, p_child_index_B);
		p_shapeA = &shapeB_as_edge;
	}

	// This is, as far as I know, the cleanest way to implement this.
	switch (p_shapeA->GetType()) {
		case b2Shape::Type::e_circle: {
			switch (p_shapeB->GetType()) {
				case b2Shape::Type::e_circle: {
					b2CollideCircles(&manifold, static_cast<const b2CircleShape *>(p_shapeA), p_xfA, static_cast<const b2CircleShape *>(p_shapeB), p_xfB);
				} break;
				case b2Shape::Type::e_edge: {
					b2CollideEdgeAndCircle(&manifold, static_cast<const b2EdgeShape *>(p_shapeB), p_xfB, static_cast<const b2CircleShape *>(p_shapeA), p_xfA);
					flipped = true;
				} break;
				case b2Shape::Type::e_polygon: {
					b2CollidePolygonAndCircle(&manifold, static_cast<const b2PolygonShape *>(p_shapeB), p_xfB, static_cast<const b2CircleShape *>(p_shapeA), p_xfA);
					flipped = true;
				} break;
			}
		} break;
		case b2Shape::Type::e_edge: {
			switch (p_shapeB->GetType()) {
				case b2Shape::Type::e_circle: {
					b2CollideEdgeAndCircle(&manifold, static_cast<const b2EdgeShape *>(p_shapeA), p_xfA, static_cast<const b2CircleShape *>(p_shapeB), p_xfB);
				} break;
				case b2Shape::Type::e_edge: {
					ERR_FAIL_V_MSG((IntersectionManifoldResult{ manifold, flipped }), "There are no contacts between two edges in Box2D. This is an invalid manifold query.");
				} break;
				case b2Shape::Type::e_polygon: {
					b2CollideEdgeAndPolygon(&manifold, static_cast<const b2EdgeShape *>(p_shapeA), p_xfA, static_cast<const b2PolygonShape *>(p_shapeB), p_xfB);
				} break;
			}
		} break;
		case b2Shape::Type::e_polygon: {
			switch (p_shapeB->GetType()) {
				case b2Shape::Type::e_circle: {
					b2CollidePolygonAndCircle(&manifold, static_cast<const b2PolygonShape *>(p_shapeA), p_xfA, static_cast<const b2CircleShape *>(p_shapeB), p_xfB);
				} break;
				case b2Shape::Type::e_edge: {
					b2CollideEdgeAndPolygon(&manifold, static_cast<const b2EdgeShape *>(p_shapeB), p_xfB, static_cast<const b2PolygonShape *>(p_shapeA), p_xfA);
					flipped = true;
				} break;
				case b2Shape::Type::e_polygon: {
					b2CollidePolygons(&manifold, static_cast<const b2PolygonShape *>(p_shapeA), p_xfA, static_cast<const b2PolygonShape *>(p_shapeB), p_xfB);
				} break;
			}
		} break;
	}

	return IntersectionManifoldResult{ manifold, flipped };
}

float Box2DWorld::_test_motion_toi(const Vector<const b2Shape *> &p_test_shapes, const MotionQueryParameters &p_params, TestMotionTOIResult *r_result) {
	if (p_test_shapes.size() == 0) {
		return 1.0f;
	}
	ERR_FAIL_COND_V_MSG(!(p_params.motion == p_params.motion), 1.0f, "Motion is NaN.");

	const b2Vec2 b2motion = gd_to_b2(p_params.motion);

	const b2Transform xform_t0 = b2Transform(gd_to_b2(p_params.transform.get_origin()), b2Rot(p_params.transform.get_rotation()));
	const b2Transform xform_t1 = b2Transform(xform_t0.p + b2motion, b2Rot(p_params.transform.get_rotation() + p_params.rotation));

	CastQueryWrapper query_callback;
	query_callback.broadPhase = &world->GetContactManager().m_broadPhase;
	query_callback.params = p_params;

	b2AABB query_aabb;
	_get_aabb_from_shapes_with_motion(p_test_shapes, xform_t0, xform_t1, query_aabb);

	world->GetContactManager().m_broadPhase.Query(&query_callback, query_aabb);

	// Calculate all TOI pairs and report the closest one

	b2TOIInput input;
	input.tMax = 1.0f;
	input.sweepA.localCenter = gd_to_b2(p_params.local_center);
	input.sweepA.c0 = xform_t0.p;
	input.sweepA.c = xform_t1.p;
	input.sweepA.a0 = xform_t0.q.GetAngle();
	input.sweepA.a = xform_t1.q.GetAngle();
	input.sweepA.alpha0 = 0.0f;

	b2TOIOutput min_output;
	min_output.state = b2TOIOutput::e_unknown;
	min_output.t = input.tMax;

	b2TOIOutput output;

	TestMotionTOIResult result{};

	// TODO optimize this
	// With longer motion casts, especially in the diagonal direction, there will
	// be many irrelevant shapes being tested using TOI.

	// iterate test shapes
	for (int i = 0; i < p_test_shapes.size(); ++i) {
		const b2Shape *test_b2shape = p_test_shapes[i];
		// Increase shape/proxy radii so that the reported TOI is at the position solver separation distance (3.0*linearSlop).
		// This allows us to return a TOI at resting position.
		// We must buffer the shape radius (not just proxy radius) or no intersection will be reported.
		constexpr float buffer = b2_linearSlop;
		const_cast<b2Shape *>(test_b2shape)->m_radius += buffer;

		for (int test_child_index = 0; test_child_index < test_b2shape->GetChildCount(); ++test_child_index) {
			input.proxyA.Set(test_b2shape, test_child_index);
			input.proxyA.m_radius += b2_linearSlop;

			// iterate query result shapes
			for (int j = 0; j < query_callback.results.size(); ++j) {
				const b2FixtureProxy *result_proxy = query_callback.results[j];
				const b2Body *result_body = result_proxy->fixture->GetBody();
				const b2Shape *result_b2shape = result_proxy->fixture->GetShape();

				for (int result_child_index = 0; result_child_index < result_b2shape->GetChildCount(); ++result_child_index) {
					input.proxyB.Set(result_b2shape, result_child_index);
					input.proxyB.m_radius += b2_linearSlop;

					input.sweepB.localCenter = result_body->GetLocalCenter();
					input.sweepB.alpha0 = 0.0f;
					input.sweepB.c = result_body->GetWorldCenter();
					input.sweepB.c0 = input.sweepB.c;
					input.sweepB.a = result_body->GetAngle();
					input.sweepB.a0 = input.sweepB.a;

					// Evaluate TOI between test and query shape

					b2TimeOfImpact(&output, &input);

					switch (output.state) {
						case b2TOIOutput::State::e_failed: // failed still gives a result, it just doesn't guarantee accuracy
						case b2TOIOutput::State::e_overlapped:
						case b2TOIOutput::State::e_touching: {
							if (output.t < min_output.t) {
								// Check if motion and local contact normal are opposing
								// <----- []->[]
								// Allows motion to exit overlapping bodies

								b2Transform xform_t;
								input.sweepA.GetTransform(&xform_t, output.t);

								IntersectionManifoldResult intersection = _evaluate_intersection_manifold(
										test_b2shape, test_child_index, xform_t,
										result_b2shape, result_child_index, result_body->GetTransform());
								b2Manifold local_manifold = intersection.manifold;

								if (!intersection.intersecting()) {
									// If we can't find any collision, skip
									if (output.state == b2TOIOutput::State::e_failed) {
										break;
									}
									WARN_PRINT("`test_motion_toi` failed intersection! Report this!");
									break;
								}

								b2WorldManifold manifold;
								manifold.Initialize(&local_manifold, xform_t0, test_b2shape->m_radius, result_body->GetTransform(), result_b2shape->m_radius);
								if (intersection.flipped)
									manifold.normal = -manifold.normal;

								const b2Vec2 normal = manifold.normal;
								if (b2Dot(normal, b2motion) <= FLT_EPSILON) {
									break;
								}

								min_output = output;
								result.collision = true;
								result.col_fixture = result_proxy->fixture;
								result.col_child_index = result_child_index;
								result.test_shape_index = i;
								result.test_shape_child_index = test_child_index;
								result.manifold_pt_count = local_manifold.pointCount;
								result.manifold = manifold;
							}
						} break;
					}
				}
			}
		}

		const_cast<b2Shape *>(test_b2shape)->m_radius -= buffer;
	}

	if (r_result != nullptr) {
		*r_result = result;
	}

	const float t_norm = min_output.t; // min_output.t / input.tMax;
	return t_norm;
}

bool Box2DWorld::_solve_position_step(const Vector<const b2Shape *> &p_body_shapes, const MotionQueryParameters &p_params, b2Vec2 &r_correction) const {
	// Part of this implementation was adapted from b2_contact_solver.cpp.

	// Query existing intersections
	CastQueryWrapper query_callback;
	query_callback.broadPhase = &world->GetContactManager().m_broadPhase;
	query_callback.params = p_params;
	query_callback.max_results = 32;

	b2Transform body_xform = gd_to_b2(p_params.transform);
	body_xform.p += r_correction;

	b2AABB query_aabb;
	_get_aabb_from_shapes(p_body_shapes, body_xform, query_aabb);

	world->GetContactManager().m_broadPhase.Query(&query_callback, query_aabb);

	float minSeparation = 0.0f;

	// iterate results
	for (int i = 0; i < p_body_shapes.size(); ++i) {
		const b2Shape *body_b2shape = p_body_shapes[i];
		for (int body_child_index = 0; body_child_index < body_b2shape->GetChildCount(); ++body_child_index) {
			for (int j = 0; j < query_callback.results.size(); ++j) {
				const b2FixtureProxy *result_proxy = query_callback.results[j];
				const b2Body *result_body = result_proxy->fixture->GetBody();
				const b2Shape *result_b2shape = result_proxy->fixture->GetShape();
				const b2Transform result_xform = result_body->GetTransform();
				for (int result_child_index = 0; result_child_index < result_b2shape->GetChildCount(); ++result_child_index) {
					// Calculate contact and separations
					IntersectionManifoldResult intersection = _evaluate_intersection_manifold(
							body_b2shape, body_child_index, body_xform,
							result_b2shape, result_child_index, result_xform);
					b2Manifold local_manifold = intersection.manifold;

					b2Transform xfA, xfB;
					float radiusA, radiusB; // TODO radius might not need be flipped
					if (intersection.flipped) {
						xfA = result_xform;
						radiusA = result_b2shape->m_radius;
						xfB = body_xform;
						radiusB = body_b2shape->m_radius;
					} else {
						xfA = body_xform;
						radiusA = body_b2shape->m_radius;
						xfB = result_xform;
						radiusB = result_b2shape->m_radius;
					}

					for (int m_i = 0; m_i < local_manifold.pointCount; ++m_i) {
						b2Vec2 normal;
						b2Vec2 point;
						float separation;

						switch (local_manifold.type) {
							case b2Manifold::e_circles: {
								b2Vec2 pointA = b2Mul(xfA, local_manifold.localPoint);
								b2Vec2 pointB = b2Mul(xfB, local_manifold.points[0].localPoint);
								normal = pointB - pointA;
								normal.Normalize();
								point = 0.5f * (pointA + pointB);
								separation = b2Dot(pointB - pointA, normal) - radiusA - radiusB;
							} break;

							case b2Manifold::e_faceA: {
								normal = b2Mul(xfA.q, local_manifold.localNormal);
								b2Vec2 planePoint = b2Mul(xfA, local_manifold.localPoint);

								b2Vec2 clipPoint = b2Mul(xfB, local_manifold.points[m_i].localPoint);
								separation = b2Dot(clipPoint - planePoint, normal) - radiusA - radiusB;
								point = clipPoint;
							} break;

							case b2Manifold::e_faceB: {
								normal = b2Mul(xfB.q, local_manifold.localNormal);
								b2Vec2 planePoint = b2Mul(xfB, local_manifold.localPoint);

								b2Vec2 clipPoint = b2Mul(xfA, local_manifold.points[m_i].localPoint);
								separation = b2Dot(clipPoint - planePoint, normal) - radiusA - radiusB;
								point = clipPoint;

								normal = -normal;
							} break;
						}
						if (intersection.flipped)
							normal = -normal;

						minSeparation = b2Min(minSeparation, separation);

						// Correct for separation
						float C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

						r_correction += C * normal;
					}
				}
			}
		}
	}

	return minSeparation >= -3.0f * b2_linearSlop;
}

b2Vec2 Box2DWorld::_solve_position(const Vector<const b2Shape *> &p_body_shapes, const MotionQueryParameters &p_params, int p_solve_steps) const {
	b2Vec2 correction = b2Vec2_zero;
	for (int i = 0; i < p_solve_steps; ++i) {
		const bool solved = _solve_position_step(p_body_shapes, p_params, correction);
		if (solved)
			break;
	}
	return correction;
}

void Box2DWorld::step(float p_step) {
	// Reset contact "solves" counter to 0
	const uint64_t *k = NULL;
	while ((k = contact_buffer.next(k))) {
		ContactBufferManifold *buffer_manifold = contact_buffer.getptr(*k);
		for (int i = 0; i < buffer_manifold->count; ++i) {
			buffer_manifold->points[i].reset_accum();
		}
	}

	// Handle pre-step logic
	for (Set<Box2DCollisionObject *>::Element *obj = body_owners.front(); obj; obj = obj->next()) {
		obj->get()->pre_step(p_step);
	}

	// Step world
	world->Step(p_step, 8, 8);
	flag_rescan_contacts_monitored = false;

	last_step_delta = p_step;

	// Step bodies/joints
	for (Set<Box2DCollisionObject *>::Element *obj = body_owners.front(); obj; obj = obj->next()) {
		obj->get()->step(p_step);
	}
	
	for (Set<Box2DJoint *>::Element *joint = joint_owners.front(); joint; joint = joint->next()) {
		joint->get()->step(p_step);
	}

	// Body/fixture inout callbacks
	object_entered_queue.call_and_clear();
	object_exited_queue.call_and_clear();
	fixture_entered_queue.call_and_clear();
	fixture_exited_queue.call_and_clear();
}

float Box2DWorld::get_last_step_delta() const {
	return last_step_delta;
}

void Box2DWorld::set_gravity(const Vector2 &p_gravity) {
	if (world)
		world->SetGravity(gd_to_b2(p_gravity));
	gravity = p_gravity;
}

Vector2 Box2DWorld::get_gravity() const {
	return gravity;
}

void Box2DWorld::set_auto_step(bool p_auto_step) {
	auto_step = p_auto_step;
}

bool Box2DWorld::get_auto_step() const {
	return auto_step;
}

Array Box2DWorld::intersect_point(const Vector2 &p_point, int p_max_results, const Array &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_sensors, uint32_t p_collision_layer, int32_t p_group_index) {
	// This function uses queries in Box2DWorld-local space, not global space

	point_callback.results.clear();
	point_callback.point = gd_to_b2(p_point);
	point_callback.max_results = p_max_results;

	point_callback.exclude.clear();
	for (int i = 0; i < p_exclude.size(); i++) {
		Object *obj = ObjectDB::get_instance(ObjectID(p_exclude[i]));
		Box2DPhysicsBody *node = Object::cast_to<Box2DPhysicsBody>(obj);
		if (node)
			point_callback.exclude.insert(node);
	}

	point_callback.filter.maskBits = p_collision_mask;
	point_callback.filter.categoryBits = p_collision_layer;
	point_callback.filter.groupIndex = p_group_index;
	point_callback.collide_with_bodies = p_collide_with_bodies;
	point_callback.collide_with_sensors = p_collide_with_sensors;

	world->QueryAABB(&point_callback, gd_to_b2(Rect2(p_point, Size2(0, 0))));

	int n = point_callback.results.size();
	Array arr;
	arr.resize(n);

	int i = 0;
	for (auto element = point_callback.results.front(); element; element = element->next()) {
		Box2DFixture *fixture = element->get();

		Dictionary d;
		d["body"] = fixture->owner_node;
		d["fixture"] = fixture;
		// TODO do we really need to return a dict, or can we just return an
		//      array of Box2DFixture objects and let the user get data from just that?

		// TODO don't return a dictionary... The "body" element is just too strange after the refactor

		arr[i] = d;
		++i;
	}

	return arr;
}

Dictionary Box2DWorld::intersect_ray(const Vector2 &p_from, const Vector2 &p_to, const Array &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_sensors, uint32_t p_collision_layer, int32_t p_group_index) {
	// This function uses queries in Box2DWorld-local space, not global space

	ERR_FAIL_COND_V_MSG(!((p_to - p_from).length_squared() > 0.0f), Dictionary{}, "Raycast queries must have valid vector inputs.");

	ray_callback.result.fixture = NULL;

	ray_callback.exclude.clear();
	for (int i = 0; i < p_exclude.size(); i++) {
		Object *obj = ObjectDB::get_instance(ObjectID(p_exclude[i]));
		Box2DPhysicsBody *node = Object::cast_to<Box2DPhysicsBody>(obj);
		if (node)
			ray_callback.exclude.insert(node);
	}

	ray_callback.filter.maskBits = p_collision_mask;
	ray_callback.filter.categoryBits = p_collision_layer;
	ray_callback.filter.groupIndex = p_group_index;
	ray_callback.collide_with_bodies = p_collide_with_bodies;
	ray_callback.collide_with_sensors = p_collide_with_sensors;

	// TODO resolve possible bug introduction
	// Does b2 RayCast behave differently than Godot space query raycast?
	// Box2D does not report bodies that the ray starts inside. Does godot do the same?
	// This can easily be fixed by adding an intersect_point check at p_from
	world->RayCast(&ray_callback, gd_to_b2(p_from), gd_to_b2(p_to));

	Dictionary dict;
	if (ray_callback.result.fixture != NULL) {
		dict["fixture"] = ray_callback.result.fixture->GetUserData().owner;
		dict["position"] = b2_to_gd(ray_callback.result.point);
		dict["normal"] = b2_to_gd(ray_callback.result.normal);
	}

	return dict;
}

Array Box2DWorld::intersect_shape(const Ref<Box2DShapeQueryParameters> &p_query, int p_max_results) {
	// This function uses queries in Box2DWorld-local space, not global space

	shape_callback.results.clear();
	shape_callback.params = p_query;
	shape_callback.max_results = p_max_results;

	// Calculate shape aabb
	const Box2DShape *shape = p_query.ptr()->shape_ref.ptr();
	b2AABB aabb;
	if (shape->is_composite_shape()) {
		const Vector<const b2Shape *> b2shapes = shape->get_shapes();
		for (int i = 0; i < b2shapes.size(); ++i) {
			const b2Shape *b2shape = b2shapes[i];
			for (int j = 0; j < b2shape->GetChildCount(); ++j) {
				b2AABB child_aabb;
				b2shape->ComputeAABB(&child_aabb, gd_to_b2(p_query->get_transform()), j);
				aabb.Combine(child_aabb);
			}
		}
	} else {
		const b2Shape *b2shape = shape->get_shape();
		for (int i = 0; i < b2shape->GetChildCount(); ++i) {
			b2AABB child_aabb;
			b2shape->ComputeAABB(&child_aabb, gd_to_b2(p_query->get_transform()), i);
			aabb.Combine(child_aabb);
		}
	}

	world->QueryAABB(&shape_callback, aabb);

	int n = point_callback.results.size();
	Array arr;
	arr.resize(n);

	int i = 0;
	for (auto element = point_callback.results.front(); element; element = element->next()) {
		Box2DFixture *fixture = element->get();

		Dictionary d;
		d["body"] = fixture->owner_node;
		d["fixture"] = fixture;

		arr[i] = d;
		++i;
	}

	return arr;
}

// Are file-scoped inline functions (for duplicate code) good practice for code cleanliness?
// no, for purpose of extensibility they should be members
inline bool _query_should_ignore_fixture(b2Fixture *p_fixture, const bool p_collide_with_sensors, const bool p_collide_with_bodies, const bool p_ignore_dynamic, const b2Filter &p_filter, const Set<const Box2DPhysicsBody *> &p_exclude) {
	// Check sensor flags
	if (!(p_collide_with_sensors && p_fixture->IsSensor()) && !(p_collide_with_bodies && !p_fixture->IsSensor())) {
		return true;
	}

	// "Infinite inertia"
	if (p_ignore_dynamic && p_fixture->GetBody()->GetType() == b2BodyType::b2_dynamicBody) {
		return true;
	}

	// Check filter
	// TODO move this to a separate function for maintainability. This logic is duplicated from Box2DWorld::ShouldCollide
	bool filtered = false;
	const b2Filter filterB = p_fixture->GetFilterData();
	if (filterB.groupIndex == p_filter.groupIndex && filterB.groupIndex != 0) {
		filtered = p_filter.groupIndex < 0;
	}

	filtered |= (filterB.categoryBits & p_filter.maskBits) == 0 && (p_filter.categoryBits & filterB.maskBits) == 0;

	if (filtered) {
		return true;
	}

	// Check exclusion
	if (p_exclude.has(Object::cast_to<Box2DPhysicsBody>(p_fixture->GetBody()->GetUserData().owner))) {
		return true;
	}

	// This fixture should not be filtered
	return false;
}

Array Box2DWorld::cast_motion(const Ref<Box2DShapeQueryParameters> &p_query) {
	// This function uses queries in Box2DWorld-local space, not global space

	// Godot cast_motion does a binary search collision test and returns the "TOI"
	// when motion is intersecting and when it is safe. The error is minimal but
	// the implementation allows for tunneling (no CCD).

	// This function differs from the Godot API because we can find the *exact*
	// time of collision using TOI.

	float toi = _test_motion_toi(p_query->shape_ref->get_shapes(), p_query->parameters, nullptr);

	const real_t motion_len = p_query->get_motion().length();
	const float t_safe = MAX(0, toi);

	Array ret;
	ret.append(toi);
	ret.append(t_safe);
	return ret;
}

bool Box2DWorld::body_test_motion(const Box2DPhysicsBody *p_body, const Transform2D &p_from, const Vector2 &p_motion, bool p_infinite_inertia, MotionResult *r_result) {
	ERR_FAIL_COND_V(!p_body, false);
	ERR_FAIL_COND_V(!p_body->body, false);

	MotionQueryParameters params;
	params.transform = p_from;
	params.motion = p_motion;
	params.ignore_dynamic = p_infinite_inertia;
	params.exclude.insert(p_body);

	Vector<const b2Shape *> query_b2shapes;
	for (b2Fixture *f = p_body->body->GetFixtureList(); f; f = f->GetNext()) {
		query_b2shapes.push_back(f->GetShape());
	}

	// Unstuck body

	const Vector2 correction = b2_to_gd(_solve_position(query_b2shapes, params, 4));
	params.transform.translate(correction);

	// Test motion

	TestMotionTOIResult toi_result;
	float toi = _test_motion_toi(query_b2shapes, params, &toi_result);

	if (r_result) {
		r_result->motion = p_motion * toi;
		r_result->motion += correction;
		r_result->remainder = p_motion * (1.0f - toi);
		r_result->t = toi;
		r_result->colliding = false;
	}

	if (toi == 1.0f) {
		return false;
	}

	// Collision has occurred

	if (r_result) {
		// Calculate collision point and other data

		const b2Shape *local_shape = query_b2shapes[toi_result.test_shape_index];
		const b2Shape *col_shape = toi_result.col_fixture->GetShape();

		// Find the collided local fixture
		// TODO if fixture list wasn't a LL this could be optimized
		b2Fixture *local_fixture = p_body->body->GetFixtureList();
		for (int i = 0; i < toi_result.test_shape_index; ++i) {
			local_fixture = local_fixture->GetNext();
		}

		const int pt_count = toi_result.manifold_pt_count;
		const b2WorldManifold manifold = toi_result.manifold;

		// Average the values at each manifold point because we can only return one point
		b2Vec2 col_point = b2Vec2_zero;
		b2Vec2 col_normal = b2Vec2_zero;
		for (int i = 0; i < pt_count; ++i) {
			col_point += manifold.points[i];
			col_normal += manifold.normal;
		}
		col_point *= 1.0f / static_cast<float>(pt_count);
		col_normal *= 1.0f / static_cast<float>(pt_count);

		r_result->collision_point = b2_to_gd(col_point);
		r_result->collision_normal = -Vector2(col_normal.x, col_normal.y);
		r_result->collider_velocity = b2_to_gd(toi_result.col_fixture->GetBody()->GetLinearVelocityFromWorldPoint(col_point));
		r_result->collider_fixture = toi_result.col_fixture->GetUserData().owner;
		r_result->local_fixture = local_fixture->GetUserData().owner;
		r_result->colliding = true;
	}

	return true;
}

bool Box2DWorld::_body_test_motion_binding(const Object *p_body, const Transform2D &p_from, const Vector2 &p_motion, bool p_infinite_inertia, const Ref<Box2DPhysicsTestMotionResult> &r_result) {
	const Box2DPhysicsBody *body = Object::cast_to<Box2DPhysicsBody>(p_body);
	ERR_FAIL_COND_V(!body, false);

	MotionResult *r = nullptr;
	if (r_result.is_valid()) {
		r = const_cast<MotionResult *>(&r_result->result);
	}
	return body_test_motion(body, p_from, p_motion, p_infinite_inertia, r);
}

void Box2DWorld::query_aabb(const Rect2 &p_bounds, Object *p_callback_owner, const String &p_callback_func) {
	// This function uses queries in Box2DWorld-local space, not global space
	user_query_callback.handled_fixtures.clear();
	user_query_callback.callback_owner = p_callback_owner;
	user_query_callback.callback_func = p_callback_func;
	world->QueryAABB(&user_query_callback, gd_to_b2(p_bounds));
}

void Box2DWorld::raycast(const Vector2 &p_from, const Vector2 &p_to, Object *p_callback_owner, const String &p_callback_func) {
	ERR_FAIL_COND_MSG(!((p_to - p_from).length_squared() > 0.0f), "Raycast queries must have valid vector inputs.");

	// This function uses queries in Box2DWorld-local space, not global space
	user_raycast_callback.handled_fixtures.clear();
	user_raycast_callback.callback_owner = p_callback_owner;
	user_raycast_callback.callback_func = p_callback_func;
	world->RayCast(&user_raycast_callback, gd_to_b2(p_from), gd_to_b2(p_to));
}

Box2DWorld::Box2DWorld() {
	gravity = GLOBAL_GET("physics/2d/default_gravity_vector");
	gravity *= real_t(GLOBAL_GET("physics/2d/default_gravity"));
}

Box2DWorld::~Box2DWorld() {
	// Make sure Box2D memory is cleaned up
	if (world) {
		WARN_PRINT("b2World is being deleted in destructor, not NOTIFICATION_PREDELETE.");
		//destroy_b2World();
		memdelete(world);
	}
}

bool Box2DWorld::PointQueryCallback::ReportFixture(b2Fixture *fixture) {
	if (_query_should_ignore_fixture(fixture, collide_with_sensors, collide_with_bodies, false, filter, exclude))
		return true;

	// Check intersection
	if (!fixture->TestPoint(point))
		return true;

	// Add to results
	results.insert(fixture->GetUserData().owner);
	return results.size() < max_results;
}

float Box2DWorld::RaycastQueryCallback::ReportFixture(b2Fixture *fixture, const b2Vec2 &point, const b2Vec2 &normal, float fraction) {
	if (_query_should_ignore_fixture(fixture, collide_with_sensors, collide_with_bodies, false, filter, exclude))
		return -1;

	// Record result
	result.fixture = fixture;
	result.point = point;
	result.normal = normal;
	//result.fraction = fraction;

	// Keep clipping ray until we get the closest fixture
	return fraction;
}

inline bool _check_shape_overlaps_with_shapes(const b2Shape *p_b2shape, const b2Transform &p_transform, const Vector<const b2Shape *> p_other_b2shapes, const b2Transform &p_shapes_transform) {
	for (int index_A = 0; index_A < p_b2shape->GetChildCount(); ++index_A) {

		for (int i = 0; i < p_other_b2shapes.size(); ++i) {

			const b2Shape *other_b2shape = p_other_b2shapes[i];
			for (int index_B = 0; index_B < other_b2shape->GetChildCount(); ++index_B) {

				if (b2TestOverlap(p_b2shape, index_A, other_b2shape, index_B, p_transform, p_shapes_transform)) {
					return true;
				}
			}
		}
	}

	return false;
}

bool Box2DWorld::ShapeQueryCallback::ReportFixture(b2Fixture *fixture) {
	if (_query_should_ignore_fixture(fixture, params->is_collide_with_sensors_enabled(), params->is_collide_with_bodies_enabled(), params->is_ignoring_rigid(), params->_get_filter(), params->_get_exclude()))
		return true;

	// Check intersection
	if (!_check_shape_overlaps_with_shapes(fixture->GetShape(), fixture->GetBody()->GetTransform(), params->shape_ref->get_shapes(), gd_to_b2(params->get_transform()))) {
		return true;
	}

	// Add to results
	results.insert(fixture->GetUserData().owner);
	return results.size() < max_results;
}

bool Box2DWorld::UserAABBQueryCallback::ReportFixture(b2Fixture *fixture) {
	const Box2DFixture *fixture_node = fixture->GetUserData().owner;

	if (handled_fixtures.find(fixture_node) == handled_fixtures.end()) {
		handled_fixtures.insert(fixture_node);
	} else {
		// Box2DFixture is already handled. We're getting a report of another of its composite b2Fixtures.
		return true;
	}

	const int argcount = 1;
	Variant arg0 = Variant(fixture_node);
	Variant *args[argcount] = {
		&arg0
	};
	Variant ret;
	Variant::CallError ce;
	ret = callback_owner->call(callback_func, (const Variant **)&args, argcount, ce);

	if (ce.error != Variant::CallError::CALL_OK) {
		String err = Variant::get_call_error_text(callback_owner, callback_func, (const Variant **)&args, argcount, ce);
		ERR_PRINT("Error calling function from query_aabb: " + err);
		return false;
	}

	if (ret.get_type() == Variant::Type::BOOL) {
		return bool(ret);
	} else {
		ERR_PRINT("Error returning from query_aabb callback: Wrong return type. Was expecting bool but instead got " + ret.get_type_name(ret.get_type()) + ".");
		return false;
	}
}

float Box2DWorld::UserRaycastQueryCallback::ReportFixture(b2Fixture *fixture, const b2Vec2 &point, const b2Vec2 &normal, float fraction) {
	const Box2DFixture *fixture_node = fixture->GetUserData().owner;

	if (handled_fixtures.find(fixture_node) == handled_fixtures.end()) {
		handled_fixtures.insert(fixture_node);
	} else {
		// Box2DFixture is already handled. We're getting a report of another of its composite b2Fixtures.
		return true;
	}

	const int argcount = 4;
	Variant arg0 = Variant(fixture_node);
	Variant arg1 = Variant(b2_to_gd(point));
	Variant arg2 = Variant(Vector2(normal.x, normal.y));
	Variant arg3 = Variant(fraction);
	Variant *args[argcount] = {
		&arg0,
		&arg1,
		&arg2,
		&arg3
	};
	Variant ret;
	Variant::CallError ce;
	ret = callback_owner->call(callback_func, (const Variant **)&args, argcount, ce);

	if (ce.error != Variant::CallError::CALL_OK) {
		String err = Variant::get_call_error_text(callback_owner, callback_func, (const Variant **)&args, argcount, ce);
		ERR_PRINT("Error calling function from raycast: " + err);
		return false;
	}

	if (ret.get_type() == Variant::Type::REAL) {
		return float(ret);
	} else {
		ERR_PRINT("Error returning from raycast callback: Wrong return type. Was expecting float but instead got " + ret.get_type_name(ret.get_type()) + ".");
		return false;
	}
}

bool Box2DWorld::CastQueryWrapper::QueryCallback(int32 proxyId) {
	b2FixtureProxy *proxy = (b2FixtureProxy *)broadPhase->GetUserData(proxyId);

	if (_query_should_ignore_fixture(proxy->fixture, params.collide_with_sensors, params.collide_with_bodies, params.ignore_dynamic, params.filter, params.exclude))
		return true;

	// There could be some optimization here for cast_motion in cases of large motion vectors.
	// We could compute TOI here and store the minimum fraction of motion [0, 1].
	// Using that fraction, we can discard results immediately by calculating the projected
	// distance between AABB_queryt0 and AABB_callback. If the distance > min_fraction, discard.

	// There is also potential optimization in the case of a large diagonal motion vector.
	// We can filter AABBs in the irrelevant corners of the enlarged query AABB.
	// First, measure the bounds of the query AABB tangent projection on the motion vector line.
	// Then measure the closest point of each callback AABB to the motion vector line (tangential projection).
	// If the measured distance is outside of our query shape bounds in tangent space, we can discard it.

	results.push_back(proxy);

	return max_results == -1 || results.size() < max_results;
}

void Box2DPhysicsTestMotionResult::_bind_methods() {
	ClassDB::bind_method(D_METHOD("is_colliding"), &Box2DPhysicsTestMotionResult::is_colliding);
	ClassDB::bind_method(D_METHOD("get_motion"), &Box2DPhysicsTestMotionResult::get_motion);
	ClassDB::bind_method(D_METHOD("get_motion_remainder"), &Box2DPhysicsTestMotionResult::get_motion_remainder);
	ClassDB::bind_method(D_METHOD("get_collision_point"), &Box2DPhysicsTestMotionResult::get_collision_point);
	ClassDB::bind_method(D_METHOD("get_collision_normal"), &Box2DPhysicsTestMotionResult::get_collision_normal);
	ClassDB::bind_method(D_METHOD("get_collider_velocity"), &Box2DPhysicsTestMotionResult::get_collider_velocity);
	ClassDB::bind_method(D_METHOD("get_collider_fixture_id"), &Box2DPhysicsTestMotionResult::get_collider_fixture_id);
	ClassDB::bind_method(D_METHOD("get_collider_fixture"), &Box2DPhysicsTestMotionResult::get_collider_fixture);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "motion"), "", "get_motion");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "motion_remainder"), "", "get_motion_remainder");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "collision_point"), "", "get_collision_point");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "collision_normal"), "", "get_collision_normal");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "collider_velocity"), "", "get_collider_velocity");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collider_fixture_id", PROPERTY_HINT_OBJECT_ID), "", "get_collider_fixture_id");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "collider_fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture"), "", "get_collider_fixture");
}

bool Box2DPhysicsTestMotionResult::is_colliding() const {
	return result.colliding;
}

Vector2 Box2DPhysicsTestMotionResult::get_motion() const {
	return result.motion;
}

Vector2 Box2DPhysicsTestMotionResult::get_motion_remainder() const {
	return result.remainder;
}

Vector2 Box2DPhysicsTestMotionResult::get_collision_point() const {
	return result.collision_point;
}

Vector2 Box2DPhysicsTestMotionResult::get_collision_normal() const {
	return result.collision_normal;
}

Vector2 Box2DPhysicsTestMotionResult::get_collider_velocity() const {
	return result.collider_velocity;
}

Box2DFixture *Box2DPhysicsTestMotionResult::get_collider_fixture() const {
	return result.collider_fixture;
}

ObjectID Box2DPhysicsTestMotionResult::get_collider_fixture_id() const {
	if (result.collider_fixture != nullptr)
		return result.collider_fixture->get_instance_id();
	return ObjectID();
}

Box2DFixture *Box2DPhysicsTestMotionResult::get_local_fixture() const {
	return result.local_fixture;
}

ObjectID Box2DPhysicsTestMotionResult::get_local_fixture_id() const {
	if (result.local_fixture != nullptr)
		return result.local_fixture->get_instance_id();
	return ObjectID();
}
