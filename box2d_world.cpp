#include "box2d_world.h"

#include <core/engine.h>

#include "box2d_fixtures.h"
#include "box2d_joints.h"

#include <string>

/**
* @author Brian Semrau
*/

Box2DWorld::Box2DWorld() :
		world(NULL) {
	gravity = GLOBAL_GET("physics/2d/default_gravity_vector");
	gravity *= real_t(GLOBAL_GET("physics/2d/default_gravity"));
}

Box2DWorld::~Box2DWorld() {
	// Make sure Box2D memory is cleaned up
	memdelete_notnull(world);
}

void Box2DWorld::SayGoodbye(b2Joint *joint) {
	joint->GetUserData().owner->on_b2Joint_destroyed();
}

void Box2DWorld::SayGoodbye(b2Fixture *fixture) {
	fixture->GetUserData().owner->on_b2Fixture_destroyed();
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

void Box2DWorld::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {

			if (!Engine::get_singleton()->is_editor_hint()) {
				// Create world
				if (!world) {
					world = memnew(b2World(gd_to_b2(gravity)));

					auto child = box2d_children.front();
					while (child) {
						child->get()->on_parent_created(this);
						child = child->next();
					}
					set_physics_process_internal(true);

					world->SetDestructionListener(this);
					world->SetContactFilter(this);
				}
			}

		} break;
		case NOTIFICATION_EXIT_TREE: {

			// Don't destroy world. It could be exiting/entering.
			// World should be destroyed in destructor if node is being freed.

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
	world->Step(p_step, 8, 8);
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

bool Box2DWorld::QueryCallback::ReportFixture(b2Fixture *fixture) {
	results.push_back(fixture);
	return true;
}

bool Box2DWorld::IntersectPointCallback::ReportFixture(b2Fixture *fixture) {
	if (fixture->TestPoint(point)) // && exclude.find(fixture->GetBody()->GetUserData().owner) > 0)
		results.push_back(fixture);
	return results.size() < max_results;
}
