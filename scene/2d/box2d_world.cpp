#include "box2d_world.h"

#include <core/config/engine.h>
#include <core/os/os.h>

#include <box2d/b2_collision.h>
#include <box2d/b2_time_of_impact.h>

#include "box2d_fixtures.h"
#include "box2d_joints.h"

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

	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_2D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_2D_PHYSICS), "set_collision_mask", "get_collision_mask");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "group_index"), "set_group_index", "get_group_index");
	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "exclude", PROPERTY_HINT_NONE, itos(Variant::INT) + ":"), "set_exclude", "get_exclude");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "motion"), "set_motion", "get_motion");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "motion_rotation"), "set_motion_rotation", "get_motion_rotation");
	ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM2D, "motion_transform"), "set_motion_transform", "get_motion_transform");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "local_center"), "set_motion_local_center", "get_motion_local_center");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "shape", PROPERTY_HINT_RESOURCE_TYPE, "Box2DShape"), "set_shape", "get_shape");
	ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM2D, "transform"), "set_transform", "get_transform");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collide_with_bodies"), "set_collide_with_bodies", "is_collide_with_bodies_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collide_with_sensors"), "set_collide_with_sensors", "is_collide_with_sensors_enabled");
}

const b2Filter &Box2DShapeQueryParameters::_get_filter() const {
	return filter;
}

Set<Box2DPhysicsBody *> Box2DShapeQueryParameters::_get_exclude() const {
	return exclude;
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
	transform = p_transform;
}

Transform2D Box2DShapeQueryParameters::get_transform() const {
	return transform;
}

void Box2DShapeQueryParameters::set_motion(const Vector2 &p_motion) {
	motion = p_motion;
}

Vector2 Box2DShapeQueryParameters::get_motion() const {
	return motion;
}

void Box2DShapeQueryParameters::set_motion_rotation(float p_rotation) {
	rotation = p_rotation;
}

float Box2DShapeQueryParameters::get_motion_rotation() const {
	return rotation;
}

void Box2DShapeQueryParameters::set_motion_transform(const Transform2D &p_transform) {
	ERR_FAIL_COND_MSG(p_transform.get_scale() != Size2(1, 1), "Box2DShapeQueryParameters does not support scaled motion transforms.");
	motion = p_transform.get_origin();
	rotation = p_transform.get_rotation();
}

Transform2D Box2DShapeQueryParameters::get_motion_transform() const {
	return Transform2D(rotation, motion);
}

void Box2DShapeQueryParameters::set_motion_local_center(const Vector2 &p_local_center) {
	local_center = p_local_center;
}

Vector2 Box2DShapeQueryParameters::get_motion_local_center() const {
	return local_center;
}

void Box2DShapeQueryParameters::set_collision_layer(int p_layer) {
	filter.categoryBits = p_layer;
}

int Box2DShapeQueryParameters::get_collision_layer() const {
	return filter.categoryBits;
}

void Box2DShapeQueryParameters::set_collision_mask(int p_collision_mask) {
	filter.maskBits = p_collision_mask;
}

int Box2DShapeQueryParameters::get_collision_mask() const {
	return filter.maskBits;
}

void Box2DShapeQueryParameters::set_group_index(int p_group_index) {
	filter.groupIndex = p_group_index;
}

int Box2DShapeQueryParameters::get_group_index() const {
	return filter.groupIndex;
}

void Box2DShapeQueryParameters::set_exclude(const Vector<int64_t> &p_exclude) {
	exclude.clear();
	for (int i = 0; i < p_exclude.size(); i++) {
		Object *obj = ObjectDB::get_instance(ObjectID(p_exclude[i]));
		Box2DPhysicsBody *node = Object::cast_to<Box2DPhysicsBody>(obj);
		if (node)
			exclude.insert(node);
	}
}

Vector<int64_t> Box2DShapeQueryParameters::get_exclude() const {
	Vector<int64_t> ret;
	ret.resize(exclude.size());
	int idx = 0;
	for (Set<Box2DPhysicsBody *>::Element *E = exclude.front(); E; E = E->next()) {
		ret.write[idx] = int64_t(E->get()->get_instance_id());
	}
	return ret;
}

void Box2DShapeQueryParameters::set_collide_with_bodies(bool p_enable) {
	collide_with_bodies = p_enable;
}

bool Box2DShapeQueryParameters::is_collide_with_bodies_enabled() const {
	return collide_with_bodies;
}

void Box2DShapeQueryParameters::set_collide_with_sensors(bool p_enable) {
	collide_with_sensors = p_enable;
}

bool Box2DShapeQueryParameters::is_collide_with_sensors_enabled() const {
	return collide_with_sensors;
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
	Box2DPhysicsBody *const &bodyA = ownerA->body_node;
	Box2DPhysicsBody *const &bodyB = ownerB->body_node;
	if ((ownerA->accept_body_collision_exceptions && bodyA->filtered.has(bodyB)) || (ownerB->accept_body_collision_exceptions && bodyB->filtered.has(bodyA))) {
		return false;
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
	Box2DPhysicsBody *body_a = fnode_a->body_node;
	Box2DPhysicsBody *body_b = fnode_b->body_node;

	const bool monitoringA = fnode_a->body_node->is_contact_monitor_enabled();
	const bool monitoringB = fnode_b->body_node->is_contact_monitor_enabled();

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
			auto contacts = &fnode_a->body_node->contact_monitor->contacts;
			contacts->insert(c);
		}
		if (hasCapacityB) {
			auto contacts = &fnode_b->body_node->contact_monitor->contacts;
			contacts->insert(c);
		}
	}
}

void Box2DWorld::BeginContact(b2Contact *contact) {
	Box2DFixture *fnode_a = contact->GetFixtureA()->GetUserData().owner;
	Box2DFixture *fnode_b = contact->GetFixtureB()->GetUserData().owner;
	Box2DPhysicsBody *body_a = fnode_a->body_node;
	Box2DPhysicsBody *body_b = fnode_b->body_node;

	const bool monitoringA = fnode_a->body_node->is_contact_monitor_enabled();
	const bool monitoringB = fnode_b->body_node->is_contact_monitor_enabled();

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
			collision_callback_queue.push_back(GodotSignalCaller("body_entered", body_a, body_b, nullptr));
		}

		int *fix_count_ptr = body_a->contact_monitor->entered_objects.getptr(fnode_b->get_instance_id());
		if (!fix_count_ptr) {
			fix_count_ptr = &(body_a->contact_monitor->entered_objects.set(fnode_b->get_instance_id(), 0)->value());
		}
		++(*fix_count_ptr);

		if (*fix_count_ptr == 1) {
			collision_callback_queue.push_back(GodotSignalCaller("body_fixture_entered", body_a, fnode_b, fnode_a));
		}
	}
	if (monitoringB) {
		int *body_count_ptr = body_b->contact_monitor->entered_objects.getptr(body_a->get_instance_id());
		if (!body_count_ptr) {
			body_count_ptr = &(body_b->contact_monitor->entered_objects.set(body_a->get_instance_id(), 0)->value());
		}
		++(*body_count_ptr);

		if (*body_count_ptr == 1) {
			collision_callback_queue.push_back(GodotSignalCaller("body_entered", body_b, body_a, nullptr));
		}

		int *fix_count_ptr = body_b->contact_monitor->entered_objects.getptr(fnode_a->get_instance_id());
		if (!fix_count_ptr) {
			fix_count_ptr = &(body_b->contact_monitor->entered_objects.set(fnode_a->get_instance_id(), 0)->value());
		}
		++(*fix_count_ptr);

		if (*fix_count_ptr == 1) {
			collision_callback_queue.push_back(GodotSignalCaller("body_fixture_entered", body_b, fnode_a, fnode_b));
		}
	}
}

void Box2DWorld::EndContact(b2Contact *contact) {
	Box2DFixture *fnode_a = contact->GetFixtureA()->GetUserData().owner;
	Box2DFixture *fnode_b = contact->GetFixtureB()->GetUserData().owner;
	Box2DPhysicsBody *body_a = fnode_a->body_node;
	Box2DPhysicsBody *body_b = fnode_b->body_node;

	const bool monitoringA = fnode_a->body_node->is_contact_monitor_enabled();
	const bool monitoringB = fnode_b->body_node->is_contact_monitor_enabled();

	// Deliver signals to bodies with contact monitoring enabled
	if (monitoringA) {
		int *body_count_ptr = body_a->contact_monitor->entered_objects.getptr(body_b->get_instance_id());
		--(*body_count_ptr);

		if ((*body_count_ptr) == 0) {
			body_a->contact_monitor->entered_objects.erase(body_b->get_instance_id());
			collision_callback_queue.push_back(GodotSignalCaller("body_exited", body_a, body_b, nullptr));
		}

		int *fix_count_ptr = body_a->contact_monitor->entered_objects.getptr(fnode_b->get_instance_id());
		--(*fix_count_ptr);

		if ((*fix_count_ptr) == 0) {
			body_a->contact_monitor->entered_objects.erase(fnode_b->get_instance_id());
			collision_callback_queue.push_back(GodotSignalCaller("body_fixture_exited", body_a, fnode_b, fnode_a));
		}
	}
	if (monitoringB) {
		int *body_count_ptr = body_b->contact_monitor->entered_objects.getptr(body_a->get_instance_id());
		--(*body_count_ptr);

		if ((*body_count_ptr) == 0) {
			body_b->contact_monitor->entered_objects.erase(body_a->get_instance_id());
			collision_callback_queue.push_back(GodotSignalCaller("body_exited", body_b, body_a, nullptr));
		}

		int *fix_count_ptr = body_b->contact_monitor->entered_objects.getptr(fnode_a->get_instance_id());
		--(*fix_count_ptr);

		if ((*fix_count_ptr) == 0) {
			body_b->contact_monitor->entered_objects.erase(fnode_a->get_instance_id());
			collision_callback_queue.push_back(GodotSignalCaller("body_fixture_exited", body_b, fnode_a, fnode_b));
		}
	}

	// Clean up all buffered contacts in the manifold
	ContactBufferManifold *buffer_manifold = contact_buffer.getptr(reinterpret_cast<uint64_t>(contact));

	if (buffer_manifold) {
		for (int i = 0; i < buffer_manifold->count; ++i) {
			Box2DContactPoint *c_ptr = &buffer_manifold->points[i];

			if (c_ptr->fixture_a->body_node->is_contact_monitor_enabled()) {
				// TODO lock/unlock
				c_ptr->fixture_a->body_node->contact_monitor->contacts.erase(*c_ptr);
			}
			if (c_ptr->fixture_b->body_node->is_contact_monitor_enabled()) {
				// TODO lock/unlock
				c_ptr->fixture_b->body_node->contact_monitor->contacts.erase(*c_ptr);
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

				if (c_ptr->fixture_a->body_node->is_contact_monitor_enabled()) {
					// TODO lock/unlock
					c_ptr->fixture_a->body_node->contact_monitor->contacts.erase(*c_ptr);
				}
				if (c_ptr->fixture_b->body_node->is_contact_monitor_enabled()) {
					// TODO lock/unlock
					c_ptr->fixture_b->body_node->contact_monitor->contacts.erase(*c_ptr);
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

	const bool monitoringA = fnode_a->body_node->is_contact_monitor_enabled();
	const bool monitoringB = fnode_b->body_node->is_contact_monitor_enabled();
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
					auto contacts = &fnode_a->body_node->contact_monitor->contacts;
					int idx = contacts->find(*c_ptr);
					if (idx >= 0)
						(*contacts)[idx] = (*c_ptr);
					//fnode_a->body_node->contact_monitor.locked = false;
				}
				if (monitoringB) {
					// Invert contact so A is always owned by the monitor
					Box2DContactPoint cB = c_ptr->flipped_a_b();

					//fnode_b->body_node->contact_monitor.locked = true; TODO
					auto contacts = &fnode_b->body_node->contact_monitor->contacts;
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

		Set<Box2DPhysicsBody *>::Element *body = bodies.front();
		while (body) {
			body->get()->on_parent_created(this);
			body = body->next();
		}
		Set<Box2DJoint *>::Element *joint = joints.front();
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

	ClassDB::bind_method(D_METHOD("query_aabb", "aabb", "callback"), &Box2DWorld::query_aabb);
	ClassDB::bind_method(D_METHOD("raycast", "from", "to", "callback"), &Box2DWorld::raycast);

	ClassDB::bind_method(D_METHOD("step", "delta"), &Box2DWorld::step);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "gravity"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_step"), "set_auto_step", "get_auto_step");
}

void Box2DWorld::step(float p_step) {
	//print_line(("step: " + std::to_string(p_step)
	//		+ ", gravity: ("
	//		+ std::to_string(world->GetGravity().x) + ", "
	//		+ std::to_string(world->GetGravity().y) + ")")
	//	.c_str());

	// Reset contact "solves" counter to 0
	const uint64_t *k = NULL;
	while ((k = contact_buffer.next(k))) {
		ContactBufferManifold *buffer_manifold = contact_buffer.getptr(*k);
		for (int i = 0; i < buffer_manifold->count; ++i) {
			buffer_manifold->points[i].reset_accum();
		}
	}

	world->Step(p_step, 8, 8);
	flag_rescan_contacts_monitored = false;

	// Pump callbacks
	while(!collision_callback_queue.empty()) {
		GodotSignalCaller sig = collision_callback_queue.front();
		if(sig.obj_b) {
			sig.obj_emitter->emit_signal(sig.signal_name, sig.obj_a, sig.obj_b);
		}
		else {
			sig.obj_emitter->emit_signal(sig.signal_name, sig.obj_a);
		}
		collision_callback_queue.pop_front();
	}

	// Notify our bodies in this world
	propagate_notification(NOTIFICATION_WORLD_STEPPED);
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

Array Box2DWorld::intersect_point(const Vector2 &p_point, int p_max_results, const Vector<int64_t> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_sensors, uint32_t p_collision_layer, int32_t p_group_index) {
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
		d["body"] = fixture->body_node;
		d["fixture"] = fixture;
		// TODO do we really need to return a dict, or can we just return an
		//      array of Box2DFixture objects and let the user get data from just that?

		arr[i] = d;
		++i;
	}

	return arr;
}

Dictionary Box2DWorld::intersect_ray(const Vector2 &p_from, const Vector2 &p_to, const Vector<int64_t> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_sensors, uint32_t p_collision_layer, int32_t p_group_index) {
	// This function uses queries in Box2DWorld-local space, not global space

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
				b2shape->ComputeAABB(&child_aabb, gd_to_b2(p_query->transform), j);
				aabb.Combine(child_aabb);
			}
		}
	} else {
		const b2Shape *b2shape = shape->get_shape();
		for (int i = 0; i < b2shape->GetChildCount(); ++i) {
			b2AABB child_aabb;
			b2shape->ComputeAABB(&child_aabb, gd_to_b2(p_query->transform), i);
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
		d["body"] = fixture->body_node;
		d["fixture"] = fixture;

		arr[i] = d;
		++i;
	}

	return arr;
}

// Are file-scoped inline functions (for duplicate code) good practice for code cleanliness?
inline bool _query_should_ignore_fixture(b2Fixture *fixture, const bool collide_with_sensors, const bool collide_with_bodies, const b2Filter &filter, const Set<Box2DPhysicsBody *> &exclude) {
	// Check sensor flags
	if (!(collide_with_sensors && fixture->IsSensor()) && !(collide_with_bodies && !fixture->IsSensor()))
		return true;

	// Check filter
	// TODO move this to a separate function for maintainability. This logic is duplicated from Box2DWorld::ShouldCollide
	bool filtered = false;
	const b2Filter filterB = fixture->GetFilterData();
	if (filterB.groupIndex == filter.groupIndex && filterB.groupIndex != 0) {
		filtered = filter.groupIndex < 0;
	}

	filtered |= (filterB.categoryBits & filter.maskBits) == 0 && (filter.categoryBits & filterB.maskBits) == 0;

	if (filtered)
		return true;

	// Check exclusion
	if (exclude.find(fixture->GetBody()->GetUserData().owner) > 0)
		return true;

	// This fixture should not be filtered
	return false;
}

struct CastQueryWrapper {
	Ref<Box2DShapeQueryParameters> params;

	Vector<b2FixtureProxy *> results;

	bool QueryCallback(int32 proxyId) {
		b2FixtureProxy *proxy = (b2FixtureProxy *)broadPhase->GetUserData(proxyId);

		if (_query_should_ignore_fixture(proxy->fixture, params->is_collide_with_sensors_enabled(), params->is_collide_with_bodies_enabled(), params->_get_filter(), params->_get_exclude()))
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

		results.append(proxy);

		return true;
	}

	const b2BroadPhase *broadPhase;
};

Array Box2DWorld::cast_motion(const Ref<Box2DShapeQueryParameters> &p_query) {
	// This function uses queries in Box2DWorld-local space, not global space

	// Godot cast_motion does a binary search collision test
	// It returns two values that it defines as:
	//     the point before collision is triggered
	//     the point when collision occurs
	// These values are just the low/high values of the binary search
	// This also does nothing to avoid tunnelling.

	// This function differs from the Godot API because we don't need to do
	// nonsense like that. We can find the exact point of collision using TOI.
	// To keep our API swappable, we return the motion fraction at the point
	// of collision and a motion fraction a safe distance away (b2_linearSlop).

	b2Transform xform_t0 = b2Transform(gd_to_b2(p_query->transform.get_origin()), b2Rot(p_query->transform.get_rotation()));
	b2Transform xform_t1 = b2Transform(xform_t0.p + gd_to_b2(p_query->motion), b2Rot(p_query->transform.get_rotation() + p_query->rotation));

	// Calc AABB with motion
	b2AABB query_aabb;
	const Vector<const b2Shape *> cast_b2shapes = p_query->shape_ref->get_shapes();
	for (int i = 0; i < cast_b2shapes.size(); ++i) {
		const b2Shape *cast_b2shape = cast_b2shapes[i];

		for (int child_index = 0; child_index < cast_b2shape->GetChildCount(); ++child_index) {
			b2AABB aabb_t0, aabb_t1;
			cast_b2shape->ComputeAABB(&aabb_t0, xform_t0, child_index);
			cast_b2shape->ComputeAABB(&aabb_t1, xform_t1, child_index);

			query_aabb.Combine(aabb_t0);
			query_aabb.Combine(aabb_t1);
		}
	}

	// Query broadphase
	CastQueryWrapper wrapper;
	wrapper.broadPhase = &world->GetContactManager().m_broadPhase;
	wrapper.params = p_query;

	world->GetContactManager().m_broadPhase.Query(&wrapper, query_aabb);

	// Calculate TOIs and select the closest one
	b2TOIInput input;
	// if predict_other_body_motion:
	//     input.tMax = motion_timedelta_for_prediction (i think)
	// else:
	input.tMax = 1.0f;
	input.sweepA.localCenter = gd_to_b2(p_query->local_center);
	input.sweepA.c0 = xform_t0.p;
	input.sweepA.c = xform_t1.p;
	input.sweepA.a0 = xform_t0.q.GetAngle();
	input.sweepA.a = xform_t1.q.GetAngle();
	input.sweepA.alpha0 = 0.0f;

	b2TOIOutput min_output;
	min_output.state = b2TOIOutput::e_unknown;
	min_output.t = input.tMax;

	b2TOIOutput output;

	// TODO optimize this
	// With longer motion casts, especially in the diagonal direction, there will
	// be many irrelevant shapes being tested using TOI.
	for (int i = 0; i < cast_b2shapes.size(); ++i) {
		const b2Shape *cast_b2shape = cast_b2shapes[i];
		for (int child_index_A = 0; child_index_A < cast_b2shape->GetChildCount(); ++child_index_A) {

			input.proxyA.Set(cast_b2shape, child_index_A);

			for (int i = 0; i < wrapper.results.size(); ++i) {
				const b2FixtureProxy *proxy = wrapper.results[i];
				const b2Body *body = proxy->fixture->GetBody();
				const b2Shape *b2shape = proxy->fixture->GetShape();
				for (int child_index_B = 0; child_index_B < b2shape->GetChildCount(); ++child_index_B) {

					input.proxyB.Set(b2shape, child_index_B);
					input.sweepB.localCenter = body->GetLocalCenter();
					input.sweepB.alpha0 = 0.0f;
					// if predict_other_body_motion:
					//     calculate sweepB from body
					// else:
					input.sweepB.c = body->GetWorldCenter();
					input.sweepB.c0 = input.sweepB.c;
					input.sweepB.a = body->GetAngle();
					input.sweepB.a0 = input.sweepB.a;
					

					b2TimeOfImpact(&output, &input);

					switch (output.state) {
						case b2TOIOutput::State::e_failed: // failed still gives a result, it just doesn't guarantee accuracy
						case b2TOIOutput::State::e_overlapped:
						case b2TOIOutput::State::e_touching: {
							if (output.t < min_output.t) {
								min_output = output;
							}
						} break;
					}
				}
			}
		}
	}

	const float t_norm = min_output.t / input.tMax;
	const real_t motion_len = p_query->motion.length();
	const float t_safe = MAX(0, t_norm - (b2_linearSlop * B2_TO_GD / motion_len));

	Array ret;
	ret.append(t_norm);
	ret.append(t_safe);
	return ret;
}

void Box2DWorld::query_aabb(const Rect2 &p_bounds, const Callable &p_callback) {
	// This function uses queries in Box2DWorld-local space, not global space
	user_query_callback.handled_fixtures.clear();
	user_query_callback.callback = &p_callback;
	world->QueryAABB(&user_query_callback, gd_to_b2(p_bounds));
}

void Box2DWorld::raycast(const Vector2 &p_from, const Vector2 &p_to, const Callable &p_callback) {
	// This function uses queries in Box2DWorld-local space, not global space
	user_raycast_callback.handled_fixtures.clear();
	user_raycast_callback.callback = &p_callback;
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
	if (_query_should_ignore_fixture(fixture, collide_with_sensors, collide_with_bodies, filter, exclude))
		return true;

	// Check intersection
	if (!fixture->TestPoint(point))
		return true;

	// Add to results
	results.insert(fixture->GetUserData().owner);
	return results.size() < max_results;
}

float Box2DWorld::RaycastQueryCallback::ReportFixture(b2Fixture *fixture, const b2Vec2 &point, const b2Vec2 &normal, float fraction) {
	if (_query_should_ignore_fixture(fixture, collide_with_sensors, collide_with_bodies, filter, exclude))
		return -1;

	// Record result
	result.fixture = fixture;
	result.point = point;
	result.normal = normal;
	//result.fraction = fraction;

	// Keep clipping ray until we get the closest fixture
	return fraction;
}

bool Box2DWorld::ShapeQueryCallback::ReportFixture(b2Fixture *fixture) {
	if (_query_should_ignore_fixture(fixture, params->collide_with_sensors, params->collide_with_bodies, params->filter, params->exclude))
		return true;

	// Check intersection
	bool overlaps = false;

	Vector<const b2Shape *> query_b2shapes = params->shape_ref->get_shapes();

	const b2Shape *fixture_b2shape = fixture->GetShape();
	for (int index_A = 0; index_A < fixture_b2shape->GetChildCount(); ++index_A) {

		for (int i = 0; i < query_b2shapes.size(); ++i) {

			const b2Shape *query_b2shape = query_b2shapes[i];
			for (int index_B = 0; index_B < query_b2shape->GetChildCount(); ++index_B) {

				if (b2TestOverlap(fixture_b2shape, index_A, query_b2shape, index_B, fixture->GetBody()->GetTransform(), gd_to_b2(params->transform))) {
					overlaps = true;
					goto endloop;
				}
			}
		}
	}
endloop:

	if (!overlaps)
		return true;

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
	Callable::CallError ce;
	callback->call((const Variant **)&args, argcount, ret, ce);

	if (ce.error != Callable::CallError::CALL_OK) {
		String err = Variant::get_callable_error_text(*callback, (const Variant **)&args, argcount, ce);
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
	Callable::CallError ce;
	callback->call((const Variant **)&args, argcount, ret, ce);

	if (ce.error != Callable::CallError::CALL_OK) {
		String err = Variant::get_callable_error_text(*callback, (const Variant **)&args, argcount, ce);
		ERR_PRINT("Error calling function from raycast: " + err);
		return false;
	}

	if (ret.get_type() == Variant::Type::FLOAT) {
		return float(ret);
	} else {
		ERR_PRINT("Error returning from raycast callback: Wrong return type. Was expecting float but instead got " + ret.get_type_name(ret.get_type()) + ".");
		return false;
	}
}
