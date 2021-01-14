#include "box2d_physics_body.h"

#include <core/config/engine.h>
#include <scene/scene_string_names.h>

#include "box2d_area.h"
#include "box2d_fixtures.h"
#include "box2d_joints.h"

#include <vector>

/**
* @author Brian Semrau
*/

void Box2DKinematicCollision::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_position"), &Box2DKinematicCollision::get_position);
	ClassDB::bind_method(D_METHOD("get_normal"), &Box2DKinematicCollision::get_normal);
	ClassDB::bind_method(D_METHOD("get_travel"), &Box2DKinematicCollision::get_travel);
	ClassDB::bind_method(D_METHOD("get_remainder"), &Box2DKinematicCollision::get_remainder);
	ClassDB::bind_method(D_METHOD("get_local_fixture"), &Box2DKinematicCollision::get_local_fixture);
	ClassDB::bind_method(D_METHOD("get_collider"), &Box2DKinematicCollision::get_collider);
	ClassDB::bind_method(D_METHOD("get_collider_id"), &Box2DKinematicCollision::get_collider_id);
	ClassDB::bind_method(D_METHOD("get_collider_fixture"), &Box2DKinematicCollision::get_collider_fixture);
	ClassDB::bind_method(D_METHOD("get_collider_fixture_id"), &Box2DKinematicCollision::get_collider_fixture_id);
	ClassDB::bind_method(D_METHOD("get_collider_velocity"), &Box2DKinematicCollision::get_collider_velocity);
	//ClassDB::bind_method(D_METHOD("get_collider_metadata"), &Box2DKinematicCollision::get_collider_metadata);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "position"), "", "get_position");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "normal"), "", "get_normal");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "travel"), "", "get_travel");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "remainder"), "", "get_remainder");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "local_fixture"), "", "get_local_fixture");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "collider"), "", "get_collider");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collider_id"), "", "get_collider_id");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "collider_fixture"), "", "get_collider_fixture");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collider_fixture_id"), "", "get_collider_fixture_id");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "collider_velocity"), "", "get_collider_velocity");
	//ADD_PROPERTY(PropertyInfo(Variant::NIL, "collider_metadata", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NIL_IS_VARIANT), "", "get_collider_metadata");
}

Vector2 Box2DKinematicCollision::get_position() const {
	return collision.collision_point;
}

Vector2 Box2DKinematicCollision::get_normal() const {
	return collision.normal;
}

Vector2 Box2DKinematicCollision::get_travel() const {
	return collision.travel;
}

Vector2 Box2DKinematicCollision::get_remainder() const {
	return collision.remainder;
}

Object *Box2DKinematicCollision::get_local_fixture() const {
	return collision.local_fixture;
}

Object *Box2DKinematicCollision::get_collider() const {
	if (collision.collider_fixture != nullptr)
		return collision.collider_fixture->get_owner();
	return nullptr;
}

ObjectID Box2DKinematicCollision::get_collider_id() const {
	if (collision.collider_fixture != nullptr)
		return collision.collider_fixture->get_owner()->get_instance_id();
	return ObjectID();
}

Object *Box2DKinematicCollision::get_collider_fixture() const {
	return collision.collider_fixture;
}

ObjectID Box2DKinematicCollision::get_collider_fixture_id() const {
	if (collision.collider_fixture != nullptr)
		return collision.collider_fixture->get_instance_id();
	return ObjectID();
}

Vector2 Box2DKinematicCollision::get_collider_velocity() const {
	return collision.collider_vel;
}

void Box2DPhysicsBody::update_mass(bool p_calc_reset) {
	if (_get_b2Body()) {
		if (use_custom_massdata) {
			_get_b2Body()->SetMassData(&massDataDef);
		} else if (p_calc_reset) {
			_get_b2Body()->ResetMassData();
		}
	}
}

void Box2DPhysicsBody::on_b2Body_created() {
	update_mass(false);

	// Notify joints
	auto joint = joints.front();
	while (joint) {
		joint->get()->on_parent_created(this);
		joint = joint->next();
	}
}

void Box2DPhysicsBody::pre_step(float p_delta) {
	if (get_type() == Mode::MODE_KINEMATIC) {
		if (!kinematic_integrate_velocity) {
			Transform2D motion = old_xform.affine_inverse() * get_box2dworld_transform();
			// TODO there is a bug here. See this issue: https://github.com/godotengine/godot/issues/34869
			// Maybe velocities should be differentiated in NOTIF_TRANSFORM_CHANGED(?) but then
			// we don't have access to the time step... maybe maintain a member var for motion
			_set_linear_velocity_no_check(motion.get_origin() / p_delta);
			_set_angular_velocity_no_check(motion.get_rotation() / p_delta);
		}
		old_xform = get_box2dworld_transform();
	}

	_update_area_effects();
}

void Box2DPhysicsBody::sync_state() {
	set_block_transform_notify(true);
	set_box2dworld_transform(b2_to_gd(_get_b2Body()->GetTransform()));
	set_block_transform_notify(false);

	// At the moment, code meant for "_integrate_forces" goes in _physics_process
	//if (get_script_instance())
	//	get_script_instance()->call("_integrate_forces");
}

void Box2DPhysicsBody::teleport(const Transform2D &p_transform) {
	bodyDef.position = gd_to_b2(p_transform.get_origin());
	bodyDef.angle = p_transform.get_rotation();

	b2Body *body = _get_b2Body();
	if (body) {
		const b2Vec2 new_b2pos = gd_to_b2(p_transform.get_origin());
		const float new_b2ang = p_transform.get_rotation();
		if (body->GetTransform().p != new_b2pos || body->GetTransform().q.GetAngle() != new_b2ang) {
			body->SetTransform(new_b2pos, new_b2ang);
		}
	}

	old_xform = p_transform;
}

void Box2DPhysicsBody::_compute_area_effects(const Box2DArea *p_area, b2Vec2 &r_gravity, float &r_lin_damp, float &r_ang_damp) {
	b2Body *body = _get_b2Body();

	if (p_area->is_gravity_a_point()) {
		if (p_area->get_gravity_distance_scale() > 0) {
			Vector2 v = p_area->get_transform().xform(p_area->get_gravity_vector()) - get_transform().get_origin();
			r_gravity += gd_to_b2(v.normalized() * (p_area->get_gravity() / Math::pow(v.length() * p_area->get_gravity_distance_scale() + 1, 2)));
		} else {
			r_gravity += gd_to_b2((p_area->get_transform().xform(p_area->get_gravity_vector()) - get_transform().get_origin()).normalized() * p_area->get_gravity());
		}
	} else {
		r_gravity += gd_to_b2(p_area->get_gravity_vector() * p_area->get_gravity());
	}

	r_lin_damp += p_area->get_linear_damp();
	r_ang_damp += p_area->get_angular_damp();
}

void Box2DPhysicsBody::_update_area_effects() {
	if (get_type() != Box2DPhysicsBody::Mode::MODE_RIGID)
		return;

	b2Body *body = _get_b2Body();

	if (body == nullptr)
		return;

	body->SetGravityScale(0);

	b2Vec2 gravity = b2Vec2_zero;
	float linear_damp = 0.0f;
	float angular_damp = 0.0f;

	const int area_count = colliding_areas.size();
	bool stopped = false;

	if (area_count) {
		colliding_areas.sort();
		const Box2DAreaItem *area_arr = &colliding_areas[0];

		for (int i = area_count - 1; i >= 0 && !stopped; --i) {
			const Box2DArea *area = area_arr[i].area;

			Box2DArea::SpaceOverride mode = area->get_space_override_mode();
			switch (mode) {
				case Box2DArea::SpaceOverride::SPACE_OVERRIDE_COMBINE_REPLACE: {
					_compute_area_effects(area, gravity, linear_damp, angular_damp);
					stopped = true;
				} break;

				case Box2DArea::SpaceOverride::SPACE_OVERRIDE_COMBINE: {
					_compute_area_effects(area, gravity, linear_damp, angular_damp);
				} break;

				case Box2DArea::SpaceOverride::SPACE_OVERRIDE_REPLACE: {
					gravity = b2Vec2_zero;
					angular_damp = 0;
					linear_damp = 0;
					_compute_area_effects(area, gravity, linear_damp, angular_damp);
					stopped = true;
				} break;

				case Box2DArea::SpaceOverride::SPACE_OVERRIDE_REPLACE_COMBINE: {
					gravity = b2Vec2_zero;
					angular_damp = 0;
					linear_damp = 0;
					_compute_area_effects(area, gravity, linear_damp, angular_damp);
				} break;

				default: {
					// area override has been disabled after colliding
				} break;
			}
		}
	}
	if (!stopped) {
		// Apply default area effects
		//_compute_area_effects(def_area, gravity, linear_damp, angular_damp);
		gravity += body->GetWorld()->GetGravity();
		linear_damp += body->GetLinearDamping();
		angular_damp += body->GetAngularDamping();
	}

	gravity *= get_gravity_scale() * body->GetMass();

	body->ApplyForceToCenter(gravity, false);
	if (linear_damp >= 0) {
		body->SetLinearDamping(linear_damp);
	}
	if (angular_damp >= 0) {
		body->SetLinearDamping(angular_damp);
	}
}

void Box2DPhysicsBody::_on_object_entered(Box2DCollisionObject *p_object) {
	const Box2DPhysicsBody *body = Object::cast_to<const Box2DPhysicsBody>(p_object);
	if (body) {
		emit_signal(SceneStringNames::get_singleton()->body_entered, body);
	}
	// ignore areas
}

void Box2DPhysicsBody::_on_object_exited(Box2DCollisionObject *p_object) {
	const Box2DPhysicsBody *body = Object::cast_to<const Box2DPhysicsBody>(p_object);
	if (body) {
		emit_signal(SceneStringNames::get_singleton()->body_exited, body);
	}
	// ignore areas
}

void Box2DPhysicsBody::_on_fixture_entered(Box2DFixture *p_fixture) {
	const Box2DPhysicsBody *body = Object::cast_to<const Box2DPhysicsBody>(p_fixture->_get_owner_node());
	if (body) {
		emit_signal("body_fixture_entered", p_fixture);
	}
	// ignore area fixtures
}

void Box2DPhysicsBody::_on_fixture_exited(Box2DFixture *p_fixture) {
	const Box2DPhysicsBody *body = Object::cast_to<const Box2DPhysicsBody>(p_fixture->_get_owner_node());
	if (body) {
		emit_signal("body_fixture_exited", p_fixture);
	}
	// ignore area fixtures
}

Ref<Box2DKinematicCollision> Box2DPhysicsBody::_move_and_collide_binding(const Vector2 &p_motion, const float p_rotation, const bool p_infinite_inertia, const bool p_exclude_raycast_shapes, const bool p_test_only) {
	KinematicCollision col;

	if (move_and_collide(p_motion, p_rotation, p_infinite_inertia, col, p_exclude_raycast_shapes, p_test_only)) {
		if (motion_cache.is_null()) {
			motion_cache.instance();
			motion_cache->owner = this;
		}

		motion_cache->collision = col;

		return motion_cache;
	}

	return Ref<Box2DKinematicCollision>();
}

Ref<Box2DKinematicCollision> Box2DPhysicsBody::_get_slide_collision_binding(int p_bounce) {
	ERR_FAIL_INDEX_V(p_bounce, kinematic_colliders.size(), Ref<Box2DKinematicCollision>());
	if (p_bounce >= kinematic_colliders_refcache.size()) {
		kinematic_colliders_refcache.resize(p_bounce + 1);
	}

	if (kinematic_colliders_refcache[p_bounce].is_null()) {
		kinematic_colliders_refcache.write[p_bounce].instance();
		kinematic_colliders_refcache.write[p_bounce]->owner = this;
	}

	kinematic_colliders_refcache.write[p_bounce]->collision = kinematic_colliders[p_bounce];
	return kinematic_colliders_refcache[p_bounce];
}

void Box2DPhysicsBody::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PREDELETE: {
			// Inform joints that this node is no more
			auto joint = joints.front();
			while (joint) {
				joint->get()->on_node_predelete(this);
				joint = joint->next();
			}

			// Inform filterers that this node has gone to a farm far away where it can run around in fields much bigger than we have at home
			for (int i = 0; i < filtering_me.size(); i++) {
				filtering_me[i]->filtered.erase(this);
			}
		} break;

		case NOTIFICATION_ENTER_TREE: {
			//last_valid_xform = get_box2dworld_transform();
			old_xform = get_box2dworld_transform();

			// Reset move_and_slide data
			on_floor = false;
			on_floor_body = ObjectID();
			on_ceiling = false;
			on_wall = false;
			kinematic_colliders.clear();
			floor_velocity = Vector2();

			// for debug drawing
			if (Engine::get_singleton()->is_editor_hint() || get_tree()->is_debugging_collisions_hint()) {
				set_process_internal(true);
			}
		} break;

		case NOTIFICATION_EXIT_TREE: {
			// Don't destroy body. It could be exiting/entering.
			// Body should be destroyed in destructor if node is being freed.

			// TODO What do we do if it exits the tree, the ref is kept (in a script), and it's never destroyed?
			//      Exiting w/o reentering should destroy body.
			//      This applies to Box2DFixture and Box2DJoint as well.
			//
			// Perhaps body creation/destruction should be queued
			// - body created: immediate CreateBody request
			// - body destroyed: queue DestroyBody
			//     - recreated before pumping -> remove from queue
			// - world pre-step: pump queue

			set_process_internal(false);
		} break;

		case NOTIFICATION_LOCAL_TRANSFORM_CHANGED: {
			if (get_type() != Mode::MODE_KINEMATIC || kinematic_integrate_velocity) {
				// Send new transform to physics
				Transform2D new_xform = get_box2dworld_transform();
				teleport(new_xform);
			}

			// Inform joints in editor that we moved
			if (Engine::get_singleton()->is_editor_hint()) {
				auto joint = joints.front();
				while (joint) {
					joint->get()->on_editor_transforms_changed();
					joint = joint->next();
				}
			}
		} break;

		case Box2DWorld::NOTIFICATION_WORLD_STEPPED: {
			if (_get_b2Body()) {
				const bool awake = _get_b2Body()->IsAwake();
				if (awake != prev_sleeping_state) {
					emit_signal("sleeping_state_changed");
					prev_sleeping_state = awake;
				}

				if (get_type() == Mode::MODE_RIGID || (get_type() == Mode::MODE_KINEMATIC && kinematic_integrate_velocity)) {
					sync_state();
				}
			}
		} break;

		case NOTIFICATION_INTERNAL_PROCESS: {
			// Do nothing
		} break;

		case NOTIFICATION_DRAW: {
			if (!Engine::get_singleton()->is_editor_hint() && !get_tree()->is_debugging_collisions_hint()) {
				break;
			}

			// TODO discuss how to put back in.  But does this just show contact points during simulation?
			
			/*
			if (body) {
				b2ContactEdge *ce = body->GetContactList();
				while (ce) {
					int count = ce->contact->GetManifold()->pointCount;

					b2WorldManifold worldManifold;
					ce->contact->GetWorldManifold(&worldManifold);
					for (int i = 0; i < count; i++) {
						draw_circle(get_box2dworld_transform().xform_inv(b2_to_gd(worldManifold.points[i])), 1.0f, Color(1.0f, 1.0f, 0.0f));
					}

					ce = ce->next;
				}
			}
			*/
		}
	}
}

void Box2DPhysicsBody::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_linear_velocity", "linear_velocity"), &Box2DPhysicsBody::set_linear_velocity);
	ClassDB::bind_method(D_METHOD("get_linear_velocity"), &Box2DPhysicsBody::get_linear_velocity);
	ClassDB::bind_method(D_METHOD("set_angular_velocity", "angular_velocity"), &Box2DPhysicsBody::set_angular_velocity);
	ClassDB::bind_method(D_METHOD("get_angular_velocity"), &Box2DPhysicsBody::get_angular_velocity);
	ClassDB::bind_method(D_METHOD("set_use_custom_massdata", "use_custom_massdata"), &Box2DPhysicsBody::set_use_custom_massdata);
	ClassDB::bind_method(D_METHOD("get_use_custom_massdata"), &Box2DPhysicsBody::get_use_custom_massdata);
	ClassDB::bind_method(D_METHOD("set_custom_mass", "custom_mass"), &Box2DPhysicsBody::set_custom_mass);
	ClassDB::bind_method(D_METHOD("get_custom_mass"), &Box2DPhysicsBody::get_custom_mass);
	ClassDB::bind_method(D_METHOD("set_custom_inertia", "custom_inertia"), &Box2DPhysicsBody::set_custom_inertia);
	ClassDB::bind_method(D_METHOD("get_custom_inertia"), &Box2DPhysicsBody::get_custom_inertia);
	ClassDB::bind_method(D_METHOD("set_custom_center_of_mass", "custom_center_of_mass"), &Box2DPhysicsBody::set_custom_center_of_mass);
	ClassDB::bind_method(D_METHOD("get_custom_center_of_mass"), &Box2DPhysicsBody::get_custom_center_of_mass);
	ClassDB::bind_method(D_METHOD("set_custom_mass_data", "mass", "inertia", "center_of_mass"), &Box2DPhysicsBody::set_custom_mass_data);
	ClassDB::bind_method(D_METHOD("get_mass"), &Box2DPhysicsBody::get_mass);
	ClassDB::bind_method(D_METHOD("get_inertia"), &Box2DPhysicsBody::get_inertia);
	ClassDB::bind_method(D_METHOD("get_center_of_mass"), &Box2DPhysicsBody::get_center_of_mass);
	ClassDB::bind_method(D_METHOD("set_linear_damping", "linear_damping"), &Box2DPhysicsBody::set_linear_damping);
	ClassDB::bind_method(D_METHOD("get_linear_damping"), &Box2DPhysicsBody::get_linear_damping);
	ClassDB::bind_method(D_METHOD("set_angular_damping", "angular_damping"), &Box2DPhysicsBody::set_angular_damping);
	ClassDB::bind_method(D_METHOD("get_angular_damping"), &Box2DPhysicsBody::get_angular_damping);
	ClassDB::bind_method(D_METHOD("set_gravity_scale", "gravity_scale"), &Box2DPhysicsBody::set_gravity_scale);
	ClassDB::bind_method(D_METHOD("get_gravity_scale"), &Box2DPhysicsBody::get_gravity_scale);
	ClassDB::bind_method(D_METHOD("set_type", "type"), &Box2DPhysicsBody::set_type);
	ClassDB::bind_method(D_METHOD("get_type"), &Box2DPhysicsBody::get_type);
	ClassDB::bind_method(D_METHOD("set_bullet", "bullet"), &Box2DPhysicsBody::set_bullet);
	ClassDB::bind_method(D_METHOD("is_bullet"), &Box2DPhysicsBody::is_bullet);
	ClassDB::bind_method(D_METHOD("set_awake", "awake"), &Box2DPhysicsBody::set_awake);
	ClassDB::bind_method(D_METHOD("is_awake"), &Box2DPhysicsBody::is_awake);
	ClassDB::bind_method(D_METHOD("set_can_sleep", "can_sleep"), &Box2DPhysicsBody::set_can_sleep);
	ClassDB::bind_method(D_METHOD("get_can_sleep"), &Box2DPhysicsBody::get_can_sleep);
	ClassDB::bind_method(D_METHOD("set_fixed_rotation", "fixed_rotation"), &Box2DPhysicsBody::set_fixed_rotation);
	ClassDB::bind_method(D_METHOD("is_fixed_rotation"), &Box2DPhysicsBody::is_fixed_rotation);

	ClassDB::bind_method(D_METHOD("get_collision_exceptions"), &Box2DPhysicsBody::get_collision_exceptions);
	ClassDB::bind_method(D_METHOD("add_collision_exception_with", "body"), &Box2DPhysicsBody::add_collision_exception_with);
	ClassDB::bind_method(D_METHOD("remove_collision_exception_with", "body"), &Box2DPhysicsBody::remove_collision_exception_with);

	ClassDB::bind_method(D_METHOD("set_contact_monitor", "enabled"), &Box2DPhysicsBody::set_contact_monitor);
	ClassDB::bind_method(D_METHOD("is_contact_monitor_enabled"), &Box2DPhysicsBody::is_contact_monitor_enabled);
	ClassDB::bind_method(D_METHOD("set_max_contacts_reported", "amount"), &Box2DPhysicsBody::set_max_contacts_reported);
	ClassDB::bind_method(D_METHOD("get_max_contacts_reported"), &Box2DPhysicsBody::get_max_contacts_reported);

	ClassDB::bind_method(D_METHOD("get_colliding_bodies"), &Box2DPhysicsBody::get_colliding_bodies);

	ClassDB::bind_method(D_METHOD("get_contact_count"), &Box2DPhysicsBody::get_contact_count);
	ClassDB::bind_method(D_METHOD("get_contact_fixture_a", "idx"), &Box2DPhysicsBody::get_contact_fixture_a);
	ClassDB::bind_method(D_METHOD("get_contact_fixture_b", "idx"), &Box2DPhysicsBody::get_contact_fixture_b);
	ClassDB::bind_method(D_METHOD("get_contact_world_pos", "idx"), &Box2DPhysicsBody::get_contact_world_pos);
	ClassDB::bind_method(D_METHOD("get_contact_impact_velocity", "idx"), &Box2DPhysicsBody::get_contact_impact_velocity);
	ClassDB::bind_method(D_METHOD("get_contact_normal", "idx"), &Box2DPhysicsBody::get_contact_normal);
	ClassDB::bind_method(D_METHOD("get_contact_normal_impulse", "idx"), &Box2DPhysicsBody::get_contact_normal_impulse);
	ClassDB::bind_method(D_METHOD("get_contact_tangent_impulse", "idx"), &Box2DPhysicsBody::get_contact_tangent_impulse);

	ClassDB::bind_method(D_METHOD("apply_force", "force", "point"), &Box2DPhysicsBody::apply_force, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("apply_central_force", "force"), &Box2DPhysicsBody::apply_central_force, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("apply_torque", "torque"), &Box2DPhysicsBody::apply_torque, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("apply_linear_impulse", "impulse", "point"), &Box2DPhysicsBody::apply_linear_impulse, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("apply_central_linear_impulse", "impulse"), &Box2DPhysicsBody::apply_central_linear_impulse, DEFVAL(true));
	ClassDB::bind_method(D_METHOD("apply_torque_impulse", "impulse"), &Box2DPhysicsBody::apply_torque_impulse, DEFVAL(true));

	ClassDB::bind_method(D_METHOD("set_kinematic_integrate_velocity", "enabled"), &Box2DPhysicsBody::set_kinematic_integrate_velocity);
	ClassDB::bind_method(D_METHOD("is_kinematic_integrating_velocity"), &Box2DPhysicsBody::is_kinematic_integrating_velocity);

	ClassDB::bind_method(D_METHOD("move_and_collide", "velocity", "rotation", "infinite_inertia", "exclude_raycast_shapes", "test_only"), &Box2DPhysicsBody::_move_and_collide_binding, DEFVAL(true), DEFVAL(true), DEFVAL(false));
	ClassDB::bind_method(D_METHOD("move_and_slide", "linear_velocity", "up_direction", "stop_on_slope", "max_slides", "floor_max_angle", "infinite_inertia"), &Box2DPhysicsBody::move_and_slide, DEFVAL(Vector2(0, 0)), DEFVAL(false), DEFVAL(4), DEFVAL(Math::deg2rad(45.0f)), DEFVAL(true));
	ClassDB::bind_method(D_METHOD("move_and_slide_with_snap", "linear_velocity", "snap", "up_direction", "stop_on_slope", "max_slides", "floor_max_angle", "infinite_inertia"), &Box2DPhysicsBody::move_and_slide_with_snap, DEFVAL(Vector2(0, 0)), DEFVAL(false), DEFVAL(4), DEFVAL(Math::deg2rad(45.0f)), DEFVAL(true));

	ClassDB::bind_method(D_METHOD("test_move", "from", "rel_vec", "infinite_inertia"), &Box2DPhysicsBody::test_move, DEFVAL(true));

	ClassDB::bind_method(D_METHOD("is_on_floor"), &Box2DPhysicsBody::is_on_floor);
	ClassDB::bind_method(D_METHOD("is_on_ceiling"), &Box2DPhysicsBody::is_on_ceiling);
	ClassDB::bind_method(D_METHOD("is_on_wall"), &Box2DPhysicsBody::is_on_wall);
	ClassDB::bind_method(D_METHOD("get_floor_normal"), &Box2DPhysicsBody::get_floor_normal);
	ClassDB::bind_method(D_METHOD("get_floor_velocity"), &Box2DPhysicsBody::get_floor_velocity);

	ClassDB::bind_method(D_METHOD("get_slide_count"), &Box2DPhysicsBody::get_slide_count);
	ClassDB::bind_method(D_METHOD("get_slide_collision", "slide_idx"), &Box2DPhysicsBody::_get_slide_collision_binding);

	//ClassDB::bind_method(D_METHOD("set_sync_to_physics", "enable"), &Box2DPhysicsBody::set_sync_to_physics);
	//ClassDB::bind_method(D_METHOD("is_sync_to_physics_enabled"), &Box2DPhysicsBody::is_sync_to_physics_enabled);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity_scale", PROPERTY_HINT_RANGE, "-128,128,0.01"), "set_gravity_scale", "get_gravity_scale");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "type", PROPERTY_HINT_ENUM, "Static,Kinematic,Rigid"), "set_type", "get_type");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "bullet"), "set_bullet", "is_bullet");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "fixed_rotation"), "set_fixed_rotation", "is_fixed_rotation");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "contacts_reported", PROPERTY_HINT_RANGE, "0,64,1,or_greater"), "set_max_contacts_reported", "get_max_contacts_reported");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "contact_monitor"), "set_contact_monitor", "is_contact_monitor_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "awake"), "set_awake", "is_awake"); // TODO rename to sleeping, or keep and add sleeping property
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "can_sleep"), "set_can_sleep", "get_can_sleep");
	ADD_GROUP("Kinematic", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "kinematic_integrate_velocity"), "set_kinematic_integrate_velocity", "is_kinematic_integrating_velocity");
	//ADD_PROPERTY(PropertyInfo(Variant::BOOL, "sync_to_physics"), "set_sync_to_physics", "is_sync_to_physics_enabled");
	ADD_GROUP("Linear", "linear_");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "linear_velocity"), "set_linear_velocity", "get_linear_velocity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "linear_damping", PROPERTY_HINT_RANGE, "-1,1,0.001"), "set_linear_damping", "get_linear_damping");
	ADD_GROUP("Angular", "angular_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_velocity"), "set_angular_velocity", "get_angular_velocity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_damping"), "set_angular_damping", "get_angular_damping");
	ADD_GROUP("", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_custom_massdata"), "set_use_custom_massdata", "get_use_custom_massdata");
	ADD_GROUP("Custom Mass Data", "custom_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "custom_mass", PROPERTY_HINT_EXP_RANGE, "0.01,65535,0.01"), "set_custom_mass", "get_custom_mass");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "custom_inertia", PROPERTY_HINT_EXP_RANGE, "0.01,65535,0.01"), "set_custom_inertia", "get_custom_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "custom_center_of_mass"), "set_custom_center_of_mass", "get_custom_center_of_mass");

	ADD_SIGNAL(MethodInfo("body_fixture_entered", PropertyInfo(Variant::OBJECT, "fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture"), PropertyInfo(Variant::OBJECT, "local_fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture")));
	ADD_SIGNAL(MethodInfo("body_fixture_exited", PropertyInfo(Variant::OBJECT, "fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture"), PropertyInfo(Variant::OBJECT, "local_fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture")));
	ADD_SIGNAL(MethodInfo("body_entered", PropertyInfo(Variant::OBJECT, "body", PROPERTY_HINT_RESOURCE_TYPE, "Box2DPhysicsBody")));
	ADD_SIGNAL(MethodInfo("body_exited", PropertyInfo(Variant::OBJECT, "body", PROPERTY_HINT_RESOURCE_TYPE, "Box2DPhysicsBody")));
	ADD_SIGNAL(MethodInfo("sleeping_state_changed"));

	BIND_ENUM_CONSTANT(MODE_RIGID);
	BIND_ENUM_CONSTANT(MODE_STATIC);
	BIND_ENUM_CONSTANT(MODE_KINEMATIC);
}

void Box2DPhysicsBody::_add_area(Box2DArea *p_area) {
	Box2DAreaItem item = Box2DAreaItem(p_area);
	const int index = colliding_areas.find(item);
	if (index == -1) {
		colliding_areas.ordered_insert(item);
		Vector<Variant> binds;
		binds.push_back(Variant(p_area));
		p_area->connect(SceneStringNames::get_singleton()->tree_exited, callable_mp(this, &Box2DPhysicsBody::_remove_area_variant), binds, Object::ConnectFlags::CONNECT_ONESHOT);
	}
}

void Box2DPhysicsBody::_remove_area(Box2DArea *p_area) {
	const int index = colliding_areas.find(Box2DAreaItem(p_area));
	if (index > -1) {
		colliding_areas.remove(index);
		p_area->disconnect(SceneStringNames::get_singleton()->tree_exited, callable_mp(this, &Box2DPhysicsBody::_remove_area_variant));
	}
}

void Box2DPhysicsBody::_remove_area_variant(const Variant &p_area) {
	_remove_area(Object::cast_to<Box2DArea>(static_cast<Object *>(p_area)));
}

String Box2DPhysicsBody::get_configuration_warning() const {
	String warning = Node2D::get_configuration_warning();

	Node *_ancestor = get_parent();
	Box2DWorld *new_world = NULL;
	while (_ancestor && !new_world) {
		new_world = Object::cast_to<Box2DWorld>(_ancestor);
		_ancestor = _ancestor->get_parent();
	}

	if (!new_world) {
		if (warning != String()) {
			warning += "\n\n";
		}
		warning += TTR("Box2DPhysicsBody only serves to provide bodies to a Box2DWorld node. Please only use it under the hierarchy of Box2DWorld.");
	}

	bool has_fixture_child = false;
	for (int i = 0; i < get_child_count(); i++) {
		if (Object::cast_to<Box2DFixture>(get_child(i))) {
			has_fixture_child = true;
			break;
		}
	}
	if (!has_fixture_child) {
		if (warning != String()) {
			warning += "\n\n";
		}
		warning += TTR("This node has no fixture, so it can't collide or interact with other objects.\nConsider adding a Box2DFixture subtype as a child to define its shape.");
	}

	return warning;
}

void Box2DPhysicsBody::_set_linear_velocity_no_check(const Vector2 &p_vel) {
	if (_get_b2Body()) {
		_get_b2Body()->SetLinearVelocity(gd_to_b2(p_vel));
	}
	bodyDef.linearVelocity = gd_to_b2(p_vel);
}

void Box2DPhysicsBody::set_linear_velocity(const Vector2 &p_vel) {
	ERR_FAIL_COND_MSG(get_type() == Mode::MODE_KINEMATIC && !kinematic_integrate_velocity, "Kinematic type bodies do not support setting linear_velocity without enabling `kinematic_integrate_velocity`.");
	_set_linear_velocity_no_check(p_vel);
}

Vector2 Box2DPhysicsBody::get_linear_velocity() const {
	if (_get_b2Body()) {
		return b2_to_gd(_get_b2Body()->GetLinearVelocity());
	}
	return b2_to_gd(bodyDef.linearVelocity);
}

void Box2DPhysicsBody::_set_angular_velocity_no_check(const real_t p_omega) {
	if (_get_b2Body()) {
		_get_b2Body()->SetAngularVelocity(p_omega);
	}
	bodyDef.angularVelocity = p_omega;
}

void Box2DPhysicsBody::set_angular_velocity(const real_t p_omega) {
	ERR_FAIL_COND_MSG(get_type() == Mode::MODE_KINEMATIC && !kinematic_integrate_velocity, "Kinematic type bodies do not support setting linear_velocity without enabling `kinematic_integrate_velocity`.");
	_set_angular_velocity_no_check(p_omega);
}

real_t Box2DPhysicsBody::get_angular_velocity() const {
	if (_get_b2Body()) {
		return _get_b2Body()->GetAngularVelocity();
	}
	return bodyDef.angularVelocity;
}

void Box2DPhysicsBody::set_use_custom_massdata(bool p_use_custom) {
	use_custom_massdata = p_use_custom;
	update_mass();
}

bool Box2DPhysicsBody::get_use_custom_massdata() const {
	return use_custom_massdata;
}

void Box2DPhysicsBody::set_custom_mass(const real_t p_mass) {
	if (!use_custom_massdata && p_mass != 1.0f) // don't print warn when setting default value
		WARN_PRINT("Changing mass related data without setting use_custom_massdata=true has no effect on the body.");
	massDataDef.mass = p_mass;
	update_mass(false);
}

real_t Box2DPhysicsBody::get_custom_mass() const {
	return massDataDef.mass;
}

void Box2DPhysicsBody::set_custom_inertia(const real_t p_inertia) {
	if (!use_custom_massdata && p_inertia != 0.5f) // don't print warn when setting default value
		WARN_PRINT("Changing mass related data without setting use_custom_massdata=true has no effect on the body.");
	massDataDef.I = p_inertia;
	update_mass(false);
}

real_t Box2DPhysicsBody::get_custom_inertia() const {
	return massDataDef.I;
}

void Box2DPhysicsBody::set_custom_center_of_mass(const Vector2 &p_center) {
	if (!use_custom_massdata && p_center != Vector2()) // don't print warn when setting default value
		WARN_PRINT("Changing mass related data without setting use_custom_massdata=true has no effect on the body.");
	massDataDef.center = gd_to_b2(p_center);
	update_mass(false);
}

Vector2 Box2DPhysicsBody::get_custom_center_of_mass() const {
	return b2_to_gd(massDataDef.center);
}

void Box2DPhysicsBody::set_custom_mass_data(const real_t p_mass, const real_t p_inertia, const Vector2 &p_center) {
	if (!use_custom_massdata)
		WARN_PRINT("Changing mass related data without setting use_custom_massdata=true has no effect on the body.");
	massDataDef.mass = p_mass;
	massDataDef.I = p_inertia;
	massDataDef.center = gd_to_b2(p_center);
	update_mass(false);
}

real_t Box2DPhysicsBody::get_mass() const {
	if (_get_b2Body()) {
		return _get_b2Body()->GetMass();
	}
	return 1.0f; // if there is no body, we can safely return a default mass
}

real_t Box2DPhysicsBody::get_inertia() const {
	if (_get_b2Body()) {
		return _get_b2Body()->GetInertia();
	}
	return 1.0f;  // if there is no body, we can safely return a default mass
}

Vector2 Box2DPhysicsBody::get_center_of_mass() const {
	if (_get_b2Body()) {
		return b2_to_gd(_get_b2Body()->GetLocalCenter());
	}
	
	return Vector2(0,0);
}

void Box2DPhysicsBody::set_linear_damping(real_t p_damping) {
	linear_damping = p_damping;
	if (p_damping < 0) {
		linear_damping = -1;
		p_damping = GLOBAL_GET("physics/2d/default_linear_damp");
	}
	if (_get_b2Body())
		_get_b2Body()->SetLinearDamping(p_damping);
	bodyDef.linearDamping = p_damping;
}

real_t Box2DPhysicsBody::get_linear_damping() const {
	return linear_damping;
}

void Box2DPhysicsBody::set_angular_damping(real_t p_damping) {
	angular_damping = p_damping;
	if (p_damping < 0) {
		angular_damping = -1;
		p_damping = GLOBAL_GET("physics/2d/default_angular_damp");
	}
	if (_get_b2Body())
		_get_b2Body()->SetAngularDamping(p_damping);
	bodyDef.angularDamping = p_damping;
}

real_t Box2DPhysicsBody::get_angular_damping() const {
	return angular_damping;
}

void Box2DPhysicsBody::set_gravity_scale(real_t p_scale) {
	if (_get_b2Body())
		_get_b2Body()->SetGravityScale(p_scale);
	bodyDef.gravityScale = p_scale;
}

real_t Box2DPhysicsBody::get_gravity_scale() const {
	return bodyDef.gravityScale;
}

void Box2DPhysicsBody::set_type(Mode p_type) {
	if (_get_b2Body())
		_get_b2Body()->SetType(static_cast<b2BodyType>(p_type));
	bodyDef.type = static_cast<b2BodyType>(p_type);
}

Box2DPhysicsBody::Mode Box2DPhysicsBody::get_type() const {
	return static_cast<Mode>(bodyDef.type);
}

void Box2DPhysicsBody::set_bullet(bool p_ccd) {
	if (_get_b2Body())
		_get_b2Body()->SetBullet(p_ccd);
	bodyDef.bullet = p_ccd;
}

bool Box2DPhysicsBody::is_bullet() const {
	return bodyDef.bullet;
}

void Box2DPhysicsBody::set_awake(bool p_awake) {
	if (_get_b2Body())
		_get_b2Body()->SetAwake(p_awake);
	bodyDef.awake = p_awake;
	prev_sleeping_state = p_awake;
}

bool Box2DPhysicsBody::is_awake() const {
	if (_get_b2Body())
		return _get_b2Body()->IsAwake();
	return bodyDef.awake;
}

void Box2DPhysicsBody::set_can_sleep(bool p_can_sleep) {
	if (_get_b2Body())
		_get_b2Body()->SetSleepingAllowed(p_can_sleep);
	bodyDef.allowSleep = p_can_sleep;
}

bool Box2DPhysicsBody::get_can_sleep() const {
	return bodyDef.allowSleep;
}

void Box2DPhysicsBody::set_fixed_rotation(bool p_fixed) {
	if (_get_b2Body())
		_get_b2Body()->SetFixedRotation(p_fixed);
	bodyDef.fixedRotation = p_fixed;
}

bool Box2DPhysicsBody::is_fixed_rotation() const {
	return bodyDef.fixedRotation;
}

Array Box2DPhysicsBody::get_collision_exceptions() {
	Array ret;
	for (int i = 0; i < filtered.size(); i++) {
		ret.append(filtered[i]);
	}
	return ret;
}

void Box2DPhysicsBody::add_collision_exception_with(Node *p_node) {
	ERR_FAIL_NULL(p_node);
	Box2DPhysicsBody *body = Object::cast_to<Box2DPhysicsBody>(p_node);
	ERR_FAIL_COND_MSG(!body, "Body collision exceptions only work with other bodies. Submit an issue if you need this.");
	filtered.insert(body);
	body->filtering_me.insert(this);
}

void Box2DPhysicsBody::remove_collision_exception_with(Node *p_node) {
	ERR_FAIL_NULL(p_node);
	Box2DPhysicsBody *body = Object::cast_to<Box2DPhysicsBody>(p_node);
	ERR_FAIL_COND_MSG(!body, "Body collision exceptions only work with other bodies. Submit an issue if you need this.");
	filtered.erase(body);
	body->filtering_me.erase(this);
}

void Box2DPhysicsBody::set_contact_monitor(bool p_enabled) {
	_set_contact_monitor(p_enabled);
}

bool Box2DPhysicsBody::is_contact_monitor_enabled() const {
	return _is_contact_monitor_enabled();
}

void Box2DPhysicsBody::set_max_contacts_reported(int p_amount) {
	max_contacts_reported = p_amount;
}

int Box2DPhysicsBody::get_max_contacts_reported() const {
	return max_contacts_reported;
}

Array Box2DPhysicsBody::get_colliding_bodies() const {
	ERR_FAIL_COND_V(!contact_monitor, Array());
	Array ret;

	List<ObjectID> keys;
	contact_monitor->entered_objects.get_key_list(&keys);
	for (int i = 0; i < keys.size(); i++) {
		Object *node = ObjectDB::get_instance(keys[i]);
		if (node && Object::cast_to<Box2DPhysicsBody>(node)) {
			ret.append(node);
		}
	}

	return ret;
}

int Box2DPhysicsBody::get_contact_count() const {
	ERR_FAIL_COND_V_MSG(!contact_monitor, int(), "Contact monitoring is disabled.");
	// TODO locking
	return contact_monitor->contacts.size();
}

Box2DFixture *Box2DPhysicsBody::get_contact_fixture_a(int p_idx) const {
	ERR_FAIL_COND_V_MSG(!contact_monitor, NULL, "Contact monitoring is disabled.");
	return contact_monitor->contacts[p_idx].fixture_a;
}

Box2DFixture *Box2DPhysicsBody::get_contact_fixture_b(int p_idx) const {
	ERR_FAIL_COND_V_MSG(!contact_monitor, NULL, "Contact monitoring is disabled.");
	return contact_monitor->contacts[p_idx].fixture_b;
}

Vector2 Box2DPhysicsBody::get_contact_world_pos(int p_idx) const {
	ERR_FAIL_COND_V_MSG(!contact_monitor, Vector2(), "Contact monitoring is disabled.");
	return contact_monitor->contacts[p_idx].world_pos;
}

Vector2 Box2DPhysicsBody::get_contact_impact_velocity(int p_idx) const {
	ERR_FAIL_COND_V_MSG(!contact_monitor, Vector2(), "Contact monitoring is disabled.");
	return contact_monitor->contacts[p_idx].impact_velocity;
}

Vector2 Box2DPhysicsBody::get_contact_normal(int p_idx) const {
	ERR_FAIL_COND_V_MSG(!contact_monitor, Vector2(), "Contact monitoring is disabled.");
	return contact_monitor->contacts[p_idx].normal;
}

float Box2DPhysicsBody::get_contact_normal_impulse(int p_idx) const {
	ERR_FAIL_COND_V_MSG(!contact_monitor, float(), "Contact monitoring is disabled.");
	return contact_monitor->contacts[p_idx].normal_impulse;
}

Vector2 Box2DPhysicsBody::get_contact_tangent_impulse(int p_idx) const {
	ERR_FAIL_COND_V_MSG(!contact_monitor, Vector2(), "Contact monitoring is disabled.");
	return contact_monitor->contacts[p_idx].tangent_impulse;
}

void Box2DPhysicsBody::apply_force(const Vector2 &force, const Vector2 &point, bool wake) {
	ERR_FAIL_COND_MSG(!_get_b2Body(), "b2Body is null.");
	_get_b2Body()->ApplyForce(gd_to_b2(force), gd_to_b2(point), wake);
}

void Box2DPhysicsBody::apply_central_force(const Vector2 &force, bool wake) {
	ERR_FAIL_COND_MSG(!_get_b2Body(), "b2Body is null.");
	_get_b2Body()->ApplyForceToCenter(gd_to_b2(force), wake);
}

void Box2DPhysicsBody::apply_torque(real_t torque, bool wake) {
	ERR_FAIL_COND_MSG(!_get_b2Body(), "b2Body is null.");
	_get_b2Body()->ApplyTorque(torque * GD_TO_B2, wake);
}

void Box2DPhysicsBody::apply_linear_impulse(const Vector2 &impulse, const Vector2 &point, bool wake) {
	ERR_FAIL_COND_MSG(!_get_b2Body(), "b2Body is null.");
	_get_b2Body()->ApplyLinearImpulse(gd_to_b2(impulse), gd_to_b2(point), wake);
}

void Box2DPhysicsBody::apply_central_linear_impulse(const Vector2 &impulse, bool wake) {
	ERR_FAIL_COND_MSG(!_get_b2Body(), "b2Body is null.");
	_get_b2Body()->ApplyLinearImpulseToCenter(gd_to_b2(impulse), wake);
}

void Box2DPhysicsBody::apply_torque_impulse(real_t impulse, bool wake) {
	ERR_FAIL_COND_MSG(!_get_b2Body(), "b2Body is null.");
	_get_b2Body()->ApplyAngularImpulse(impulse * GD_TO_B2, wake);
}

void Box2DPhysicsBody::set_kinematic_integrate_velocity(bool p_integrate_vel) {
	if (kinematic_integrate_velocity == p_integrate_vel) {
		return;
	}
	ERR_FAIL_COND_MSG(get_type() != Mode::MODE_KINEMATIC, "The property kinematic_integrate_velocity has no effect on non-kinematic bodies.");

	kinematic_integrate_velocity = p_integrate_vel;

	if (kinematic_integrate_velocity) {
		// Make sure any transform changes between last step and now are integrated
		teleport(get_box2dworld_transform());
	}
}

bool Box2DPhysicsBody::is_kinematic_integrating_velocity() const {
	return kinematic_integrate_velocity;
}

bool Box2DPhysicsBody::move_and_collide(const Vector2 &p_motion, const float p_rotation, const bool p_infinite_inertia, KinematicCollision &r_collision, const bool p_exclude_raycast_shapes, const bool p_test_only) {
	if (get_type() != Mode::MODE_KINEMATIC) {
		ERR_PRINT_ONCE("This function is only meant to be used with Kinematic body types.");
	}
	ERR_FAIL_COND_V(!_get_world_node(), false);

	// TODO check sync_to_physics property?

	Transform2D gt = get_box2dworld_transform();

	Box2DWorld::MotionResult result;
	bool colliding = _get_world_node()->body_test_motion(this, gt, p_motion, p_infinite_inertia, &result);

	if (colliding) {
		r_collision.collision_point = result.collision_point;
		r_collision.normal = result.collision_normal;
		r_collision.collider_vel = result.collider_velocity;
		r_collision.collider_fixture = result.collider_fixture;
		r_collision.remainder = result.remainder;
		r_collision.travel = result.motion;
		r_collision.collider_fixture = result.collider_fixture;
		r_collision.local_fixture = result.local_fixture;
	}

	if (!p_test_only) {
		gt.elements[2] += result.motion;
		set_block_transform_notify(true);
		set_box2dworld_transform(gt);
		set_block_transform_notify(false);
	}

	return colliding;
}

bool Box2DPhysicsBody::test_move(const Transform2D &p_from, const Vector2 &p_motion, bool p_infinite_inertia) {
	return _get_world_node()->body_test_motion(this, p_from, p_motion, p_infinite_inertia);
}

Vector2 Box2DPhysicsBody::move_and_slide(const Vector2 &p_linear_velocity, const Vector2 &p_up_direction, bool p_stop_on_slope, int p_max_slides, float p_floor_max_angle, bool p_infinite_inertia) {

	static constexpr float floor_angle_threshold = 0.01f;

	Vector2 body_velocity = p_linear_velocity;
	Vector2 body_velocity_normal = body_velocity.normalized();
	Vector2 up_direction = p_up_direction.normalized();

	Vector2 current_floor_velocity = floor_velocity;
	if (on_floor && on_floor_body.is_valid()) {
		const Object *obj = ObjectDB::get_instance(on_floor_body);
		const Box2DPhysicsBody *body = Object::cast_to<Box2DPhysicsBody>(obj);
		if (body) {
			// TODO this could be more accurate by taking into account angular velocity
			current_floor_velocity = body->get_linear_velocity();
		}
	}

	Vector2 motion = (current_floor_velocity + body_velocity) * _get_world_node()->get_last_step_delta();

	on_floor = false;
	on_floor_body = ObjectID();
	on_ceiling = false;
	on_wall = false;
	kinematic_colliders.clear();
	floor_normal = Vector2();
	floor_velocity = Vector2();

	while (p_max_slides) {
		KinematicCollision collision;
		bool collided = false;

		collided = move_and_collide(motion, 0, p_infinite_inertia, collision);
		if (!collided) {
			motion = Vector2(); // motion completed
		}

		if (collided) {
			kinematic_colliders.push_back(collision);
			motion = collision.remainder;

			if (up_direction == Vector2()) {
				// every collision is a wall
				on_wall = true;
			} else {
				if (Math::acos(collision.normal.dot(up_direction)) <= p_floor_max_angle + floor_angle_threshold) {

					on_floor = true;
					floor_normal = collision.normal;
					on_floor_body = collision.collider_fixture->get_instance_id();
					floor_velocity = collision.collider_vel;

					if (p_stop_on_slope) {
						if ((body_velocity_normal + up_direction).length() < 0.01 && collision.travel.length() < 1) {
							Transform2D gt = get_box2dworld_transform();
							gt.elements[2] -= collision.travel.slide(up_direction);
							set_box2dworld_transform(gt);
							return Vector2();
						}
					}
				} else if (Math::acos(collision.normal.dot(-up_direction)) <= p_floor_max_angle + floor_angle_threshold) {
					on_ceiling = true;
				} else {
					on_wall = true;
				}
			}

			motion = motion.slide(collision.normal);
			body_velocity = body_velocity.slide(collision.normal);
		}

		if (!collided || motion == Vector2()) {
			break;
		}

		--p_max_slides;
	}

	return body_velocity;
}

Vector2 Box2DPhysicsBody::move_and_slide_with_snap(const Vector2 &p_linear_velocity, const Vector2 &p_snap, const Vector2 &p_up_direction, bool p_stop_on_slope, int p_max_slides, float p_floor_max_angle, bool p_infinite_inertia) {
	static constexpr float floor_angle_threshold = 0.01f;

	Vector2 up_direction = p_up_direction.normalized();
	bool was_on_floor = on_floor;

	Vector2 ret_vel = move_and_slide(p_linear_velocity, up_direction, p_stop_on_slope, p_max_slides, p_floor_max_angle, p_infinite_inertia);
	if (!was_on_floor || p_snap == Vector2()) {
		return ret_vel;
	}

	KinematicCollision col;
	Transform2D gt = get_global_transform();

	if (move_and_collide(p_snap, 0, p_infinite_inertia, col, false, true)) {
		bool apply = true;
		if (up_direction != Vector2()) {
			if (Math::acos(col.normal.dot(up_direction)) <= p_floor_max_angle + floor_angle_threshold) {
				on_floor = true;
				floor_normal = col.normal;
				on_floor_body = col.collider_fixture->get_instance_id();
				floor_velocity = col.collider_vel;
				if (p_stop_on_slope) {
					// move and collide may stray the object a bit because of pre un-stucking,
					// so only ensure that motion happens on floor direction in this case.
					col.travel = up_direction * up_direction.dot(col.travel);
				}
			} else {
				apply = false;
			}
		}

		if (apply) {
			gt.elements[2] += col.travel;
			set_global_transform(gt);
		}
	}

	return ret_vel;
}

bool Box2DPhysicsBody::is_on_floor() const {
	//ERR_FAIL_COND_V_MSG(get_type() != Mode::MODE_KINEMATIC, false, "This function is only available for Kinematic type bodies.");
	return on_floor;
}

bool Box2DPhysicsBody::is_on_wall() const {
	//ERR_FAIL_COND_V_MSG(get_type() != Mode::MODE_KINEMATIC, false, "This function is only available for Kinematic type bodies.");
	return on_ceiling;
}

bool Box2DPhysicsBody::is_on_ceiling() const {
	//ERR_FAIL_COND_V_MSG(get_type() != Mode::MODE_KINEMATIC, false, "This function is only available for Kinematic type bodies.");
	return on_wall;
}

Vector2 Box2DPhysicsBody::get_floor_normal() const {
	//ERR_FAIL_COND_V_MSG(get_type() != Mode::MODE_KINEMATIC, Vector2(), "This function is only available for Kinematic type bodies.");
	return floor_normal;
}

Vector2 Box2DPhysicsBody::get_floor_velocity() const {
	//ERR_FAIL_COND_V_MSG(get_type() != Mode::MODE_KINEMATIC, Vector2(), "This function is only available for Kinematic type bodies.");
	return floor_velocity;
}

int Box2DPhysicsBody::get_slide_count() const {
	return kinematic_colliders.size();
}

Box2DPhysicsBody::KinematicCollision Box2DPhysicsBody::get_slide_collision(int p_bounce) const {
	ERR_FAIL_INDEX_V(p_bounce, kinematic_colliders.size(), KinematicCollision());
	return kinematic_colliders[p_bounce];
}

Box2DPhysicsBody::Box2DPhysicsBody() {
	set_physics_process_internal(true);
	set_notify_local_transform(true);
}

Box2DPhysicsBody::~Box2DPhysicsBody() {
	if (motion_cache.is_valid()) {
		motion_cache->owner = nullptr;
	}

	for (int i = 0; i < kinematic_colliders_refcache.size(); i++) {
		if (kinematic_colliders_refcache[i].is_valid()) {
			kinematic_colliders_refcache.write[i]->owner = nullptr;
		}
	}

	// B2 destruction handled by Box2DCollisionObject
}
