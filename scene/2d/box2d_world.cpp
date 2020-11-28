#include "box2d_world.h"

#include <core/engine.h>
#include <core/os/os.h>

#include <box2d/b2_collision.h>

#include "box2d_fixtures.h"
#include "box2d_joints.h"

#include <chrono>
#include <string>

/**
* @author Brian Semrau
*/

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

	if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0) {
		return filterA.groupIndex > 0;
	}

	bool collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;

	// Custom contact filtering

	if (!collide) {
		return false;
	}

	// Check for fixture exclusions
	Box2DFixture *const &ownerA = fixtureA->GetUserData().owner;
	Box2DFixture *const &ownerB = fixtureB->GetUserData().owner;
	if (ownerA->filtered.has(ownerB) || ownerB->filtered.has(ownerA)) {
		return false;
	} else {
		// Check for body exclusions
		Box2DPhysicsBody *const &bodyA = ownerA->body_node;
		Box2DPhysicsBody *const &bodyB = ownerB->body_node;
		if ((ownerA->accept_body_collision_exceptions && bodyA->filtered.has(bodyB)) || (ownerB->accept_body_collision_exceptions && bodyB->filtered.has(bodyA))) {
			return false;
		} else {
			// TODO should we bother to let bodies exclude fixtures?
			return true;
		}
	}
}

void Box2DWorld::BeginContact(b2Contact *contact) {
	// TODO body enter signal

	Box2DFixture *fnode_a = contact->GetFixtureA()->GetUserData().owner;
	Box2DFixture *fnode_b = contact->GetFixtureB()->GetUserData().owner;

	// Only buffer contacts that are being monitored
	const bool monitoringA = fnode_a->body_node->is_contact_monitor_enabled();
	const bool monitoringB = fnode_b->body_node->is_contact_monitor_enabled();

	if (monitoringA || monitoringB) {
		// Init contact
		Box2DContact c;
		c.id = next_contact_id++;
		c.fixture_a = fnode_a;
		c.fixture_b = fnode_b;
		// The contact pointer address seems to be the only way to create a unique key. Please prove me wrong.
		contact_buffer.set(reinterpret_cast<int64_t>(contact), c);
		// Buffer again into monitoring node
		if (monitoringA) {
			auto contacts = &fnode_a->body_node->contact_monitor->contacts;
			contacts->insert(c);
		}
		if (monitoringB) {
			auto contacts = &fnode_b->body_node->contact_monitor->contacts;
			contacts->insert(c);
		}
	}
}

void Box2DWorld::EndContact(b2Contact *contact) {
	// TODO body exit signal

	Box2DContact *c_ptr = contact_buffer.getptr(reinterpret_cast<int64_t>(contact));
	if (c_ptr) {
		if (c_ptr->fixture_a->body_node->is_contact_monitor_enabled()) {
			// TODO lock/unlock
			c_ptr->fixture_a->body_node->contact_monitor->contacts.erase(*c_ptr);
		}
		if (c_ptr->fixture_b->body_node->is_contact_monitor_enabled()) {
			// TODO lock/unlock
			c_ptr->fixture_b->body_node->contact_monitor->contacts.erase(*c_ptr);
		}
		ERR_FAIL_COND(!contact_buffer.erase(reinterpret_cast<int64_t>(contact)));
	}
}

void Box2DWorld::PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {
	// Box2D point states are confusing... need to inspect how they work further
	//b2PointState state1[2], state2[2];
	//b2GetPointStates(state1, state2, oldManifold, contact->GetManifold());

	// TODO using point state transitions, create/destroy Box2DContacts

	Box2DFixture *fnode_a = contact->GetFixtureA()->GetUserData().owner;
	Box2DFixture *fnode_b = contact->GetFixtureB()->GetUserData().owner;

	Box2DContact *c_ptr = contact_buffer.getptr(reinterpret_cast<int64_t>(contact));

	if (unlikely(flag_rescan_contacts_monitored) && !c_ptr) {
		// Buffer a contact that only started being monitored after BeginContact
		const bool monitoringA = fnode_a->body_node->is_contact_monitor_enabled();
		const bool monitoringB = fnode_b->body_node->is_contact_monitor_enabled();
		if (monitoringA || monitoringB) {
			// Init contact
			Box2DContact c;
			c.id = next_contact_id++;
			c.fixture_a = fnode_a;
			c.fixture_b = fnode_b;
			contact_buffer.set(reinterpret_cast<int64_t>(contact), c);
			// Buffer again into monitoring node
			if (monitoringA) {
				auto contacts = &fnode_a->body_node->contact_monitor->contacts;
				contacts->insert(c);
			}
			if (monitoringB) {
				auto contacts = &fnode_b->body_node->contact_monitor->contacts;
				contacts->insert(c);
			}
			c_ptr = contact_buffer.getptr(reinterpret_cast<int64_t>(contact));
		}
	}

	// Only handle the first PreSolve for this contact this step (don't overwrite first impact_velocity)
	if (c_ptr && c_ptr->solves == 0) {
		c_ptr->solves += 1;

		b2WorldManifold worldManifold;
		contact->GetWorldManifold(&worldManifold);

		Vector2 manifold_norm = Vector2(worldManifold.normal.x, worldManifold.normal.y);
		c_ptr->normal = manifold_norm;
		c_ptr->normal_impulse = 0.0f;
		c_ptr->tangent_impulse = Vector2();
		c_ptr->world_pos = Vector2();
		const int manPtCount = contact->GetManifold()->pointCount;
		b2Vec2 manifold_center = b2Vec2_zero; // TODO remove when making separate contacts for each point
		for (int i = 0; i < manPtCount; ++i) {
			manifold_center += worldManifold.points[i];
		}
		manifold_center *= 1.0f / manPtCount;
		c_ptr->world_pos = b2_to_gd(manifold_center);

		b2Vec2 point = manifold_center; //worldManifold.points[0];
		b2Vec2 relV = contact->GetFixtureB()->GetBody()->GetLinearVelocityFromWorldPoint(point);
		relV -= contact->GetFixtureA()->GetBody()->GetLinearVelocityFromWorldPoint(point);

		c_ptr->impact_velocity = b2_to_gd(relV);
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

		// TODO report bug to Box2D: polygon/edge collision creates manifold with pt count = 2 but impulse with count 1
		//    * needs testing to confirm
		// actually this is probably because the second point is in the removed state
		//if (contact->GetManifold()->pointCount != impulse->count) {
		//	WARN_PRINT(("Point counts not equal! manifold: " + std::to_string(contact->GetManifold()->pointCount) + ", impulse: " + std::to_string(impulse->count)).c_str());
		//}

		Box2DContact *c_ptr = contact_buffer.getptr(reinterpret_cast<int64_t>(contact));

		Vector2 manifold_tan = c_ptr->normal.rotated(Math_PI * 0.5);
		// TODO test: should impulse be accumulated (relevant to TOI solve), or does Box2D accumulate them itself?
		int pointCount = impulse->count;
		for (int i = 0; i < pointCount; ++i) {
			c_ptr->normal_impulse += impulse->normalImpulses[i];
			c_ptr->tangent_impulse += manifold_tan * impulse->tangentImpulses[i];
		}

		if (monitoringA) {
			//fnode_a->body_node->contact_monitor.locked = true; TODO
			auto contacts = &fnode_a->body_node->contact_monitor->contacts;
			int idx = contacts->find(*c_ptr);
			(*contacts)[idx] = (*c_ptr);
			//fnode_a->body_node->contact_monitor.locked = false;
		}
		if (monitoringB) {
			// Invert contact so A is always owned by the monitor
			Box2DContact cB = c_ptr->flipped_a_b();

			//fnode_b->body_node->contact_monitor.locked = true; TODO
			auto contacts = &fnode_b->body_node->contact_monitor->contacts;
			int idx = contacts->find(cB);
			(*contacts)[idx] = (cB);
			//fnode_b->body_node->contact_monitor.locked = false;
		}
	}
}

void Box2DWorld::create_b2World() {
	if (!world) {
		world = memnew(b2World(gd_to_b2(gravity)));

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

		world->SetDestructionListener(this);
		world->SetContactFilter(this);
		world->SetContactListener(this);

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
			float time = get_physics_process_delta_time();
			step(time);
		} break;
	}
}

void Box2DWorld::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &Box2DWorld::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &Box2DWorld::get_gravity);

	//ClassDB::bind_method(D_METHOD("query_aabb", "bounds"), &Box2DWorld::query_aabb);
	ClassDB::bind_method(D_METHOD("intersect_point", "point"), &Box2DWorld::intersect_point, DEFVAL(32));
	//ClassDB::bind_method(D_METHOD("intersect_shape", "TODO"), &Box2DWorld::intersect_shape);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "gravity"), "set_gravity", "get_gravity");
}

void Box2DWorld::step(real_t p_step) {
	//print_line(("step: " + std::to_string(p_step)
	//		+ ", gravity: ("
	//		+ std::to_string(world->GetGravity().x) + ", "
	//		+ std::to_string(world->GetGravity().y) + ")")
	//	.c_str());

	// Reset contact "solves" counter to 0
	const int64_t *k = NULL;
	while ((k = contact_buffer.next(k))) {
		contact_buffer.getptr(*k)->reset_accum();
	}

	world->Step(p_step, 8, 8);
	flag_rescan_contacts_monitored = false;
}

void Box2DWorld::set_gravity(const Vector2 &p_gravity) {
	if (world)
		world->SetGravity(gd_to_b2(p_gravity));
	gravity = p_gravity;
}

Vector2 Box2DWorld::get_gravity() const {
	return gravity;
}

Array Box2DWorld::intersect_point(const Vector2 &p_point, int p_max_results) { //, const Vector<Ref<Box2DPhysicsBody> > &p_exclude/*, uint32_t p_layers*/) {
	pointCallback.results.clear();
	pointCallback.point = gd_to_b2(p_point);
	//pointCallback.exclude.clear();
	//for (int i = 0; i < p_exclude.size(); i++)
	//	pointCallback.exclude.insert(p_exclude[i]);
	world->QueryAABB(&pointCallback, gd_to_b2(Rect2(p_point, Size2(0, 0))));

	int n = pointCallback.results.size();
	Array arr;
	arr.resize(n);
	for (int i = 0; i < n; i++) {
		b2Fixture *fixture = pointCallback.results.get(i);

		Dictionary d;
		d["body"] = fixture->GetBody()->GetUserData().owner;
		d["fixture"] = fixture->GetUserData().owner;
		// TODO do we really need to return a dict, or can we just return an
		//      array of Box2DFixture objects and let the user get data from just that?

		arr[i] = d;
	}

	return arr;
}

//Array Box2DWorld::query_aabb(const Rect2 &p_bounds) {
//	aabbCallback.results.clear();
//	world->QueryAABB(&aabbCallback, gd_to_b2(p_bounds));
//
//	int n = aabbCallback.results.size();
//	Array arr;
//	arr.resize(n);
//	for (int i = 0; i < n; i++) {
//		b2Fixture *fixture = aabbCallback.results.get(i);
//
//		Dictionary d;
//		d["body"] = fixture->GetBody()->GetUserData().owner;
//		d["fixture"] = fixture->GetUserData().owner;
//		// TODO do we really need to return a dict, or can we just return an
//		//      array of Box2DFixture objects and let the user get data from just that?
//
//		arr[i] = d;
//	}
//
//	return arr;
//}

Box2DWorld::Box2DWorld() :
		world(NULL) {
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

bool Box2DWorld::QueryCallback::ReportFixture(b2Fixture *fixture) {
	results.push_back(fixture);
	return true;
}

bool Box2DWorld::IntersectPointCallback::ReportFixture(b2Fixture *fixture) {
	if (fixture->TestPoint(point)) // && exclude.find(fixture->GetBody()->GetUserData().owner) > 0)
		results.push_back(fixture);
	return results.size() < max_results;
}
