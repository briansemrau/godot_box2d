#include "box2d_world.h"

#include <core/engine.h>

#include "box2d_fixtures.h"
#include "box2d_joints.h"

#include <string>

/**
* @author Brian Semrau
*/

/*void ContactListener::BeginContact(b2Contact *contact) {
	// intentionally blank, may remove
}

void ContactListener::EndContact(b2Contact *contact) {
	// intentionally blank, may remove
}

void ContactListener::PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {
	if (contact->GetFixtureA()->GetBody()->GetUserData().exceptions.has(contact->GetFixtureB()->GetBody()->GetUserData().id)) {
		contact->SetEnabled(false);
	}
}

void ContactListener::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {
	// intentionally blank, may remove
	// Note: this impulse callback would be very useful if Godot 2D physics added impulse data to contact reporting
}

void Box2DWorld::body_add_collision_exception(RID p_body, RID p_body_b) {
	b2Body *body = body_owner.get(p_body)->get_obj();
	ERR_FAIL_COND(!body);

	body->GetUserData().exceptions.insert(p_body_b);
	body->SetAwake(true);
}

void Box2DWorld::body_remove_collision_exception(RID p_body, RID p_body_b) {
	b2Body *body = body_owner.get(p_body)->get_obj();
	ERR_FAIL_COND(!body);

	body->GetUserData().exceptions.erase(p_body_b);
	body->SetAwake(true);
}

void Box2DWorld::body_get_collision_exceptions(RID p_body, List<RID> *p_exceptions) {
	b2Body *body = body_owner.get(p_body)->get_obj();
	ERR_FAIL_COND(!body);

	for (int i = 0; i < body->GetUserData().exceptions.size(); i++) {
		p_exceptions->push_back(body->GetUserData().exceptions[i]);
	}
}*/

Box2DWorld::Box2DWorld() :
		world(NULL) {
	gravity = GLOBAL_GET("physics/2d/default_gravity_vector");
	gravity *= real_t(GLOBAL_GET("physics/2d/default_gravity"));

	//contact_listener = new ContactListener();
	//world->SetContactListener(contact_listener);
	//world = memnew(b2World(b2Vec2_zero));
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

Array Box2DWorld::intersect_point(const Vector2 &p_point, int p_max_results) {//, const Vector<Ref<Box2DPhysicsBody> > &p_exclude/*, uint32_t p_layers*/) {
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
	if (fixture->TestPoint(point))// && exclude.find(fixture->GetBody()->GetUserData().owner) > 0)
		results.push_back(fixture);
	return results.size() < max_results;
}
