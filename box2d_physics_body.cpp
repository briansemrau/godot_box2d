#include "box2d_physics_body.h"
#include "box2d_world.h"

bool Box2DPhysicsBody::create_b2Body() {
	if (world_node && !body) {
		ERR_FAIL_COND_V(!world_node->world, false);

		// Create body
		bodyDef.position = gd_to_b2(get_transform().get_origin());
		bodyDef.angle = get_transform().get_rotation();

		body = world_node->world->CreateBody(&bodyDef);
		body->GetUserData().owner = this;

		// Create fixtures
		auto child = fixtures.front();
		while (child) {
			child->get()->on_parent_created(this);
			child = child->next();
		}

		// Notify joints
		child = joints.front();
		while (child) {
			child->get()->on_parent_created(this);
			child = child->next();
		}

		set_physics_process_internal(true);
		return true;
	}
	return false;
}

bool Box2DPhysicsBody::destroy_b2Body() {
	if (body) {
		ERR_FAIL_COND_V(!world_node, false);
		ERR_FAIL_COND_V(!world_node->world, false);

		// Destroy body
		world_node->world->DestroyBody(body);
		print_line("body destroyed");
		body = NULL;

		// Fixture destruction is handled by destruction listener

		set_physics_process_internal(false);
		return true;
	}
	return false;
}

void Box2DPhysicsBody::_notification(int p_what) {
	// TODO finalize implementation to imitate behavior from RigidBody2D and Kinematic (static too?)
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {

			last_valid_xform = get_global_transform();

			// Find the Box2DWorld
			Node *_ancestor = get_parent();
			Box2DWorld *new_world = NULL;
			while (_ancestor && !new_world) {
				new_world = Object::cast_to<Box2DWorld>(_ancestor);
			}

			// If new parent, recreate body
			if (new_world != world_node) {
				// Destroy b2Body
				if (world_node) {
					destroy_b2Body();
					if (world_node) {
						world_node->box2d_children.erase(this);
					}
				}
				world_node = new_world;
				// Create b2Body
				if (world_node) {
					world_node->box2d_children.insert(this);

					if (world_node->world) {
						create_b2Body();
					}
				}
			}

		} break;
		case NOTIFICATION_EXIT_TREE: {

			// Don't destroy body. It could be exiting/entering.
			// Body should be destroyed in destructor if node is being freed.

		} break;
		case NOTIFICATION_LOCAL_TRANSFORM_CHANGED: {

			// Send new transform to physics
			Transform2D new_xform = get_transform();

			bodyDef.position = gd_to_b2(new_xform.get_origin());
			bodyDef.angle = new_xform.get_rotation();

			if (body) {
				body->SetTransform(gd_to_b2(new_xform.get_origin()), new_xform.get_rotation());
				// Revert changes. Node transform shall be updated on physics process.
				if (body->GetType() != b2_staticBody) {
					//set_notify_local_transform(false);
					//set_global_transform(last_valid_xform);
					//set_notify_local_transform(true);
				}
			}

		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {

			// TODO figure out if this can instead be a callback from Box2D.
			//		I don't think it can.
			if (body) { // && body->IsAwake()) {
				set_block_transform_notify(true);
				set_transform(b2_to_gd(body->GetTransform()));
				set_block_transform_notify(false);

				// handle contact monitoring or something? (see RigidBody2D::_direct_state_changed)
			}

		} break;
	}
}

void Box2DPhysicsBody::_bind_methods() {

	ClassDB::bind_method(D_METHOD("set_linear_velocity", "linear_velocity"), &Box2DPhysicsBody::set_linear_velocity);
	ClassDB::bind_method(D_METHOD("get_linear_velocity"), &Box2DPhysicsBody::get_linear_velocity);
	ClassDB::bind_method(D_METHOD("set_angular_velocity", "angular_velocity"), &Box2DPhysicsBody::set_angular_velocity);
	ClassDB::bind_method(D_METHOD("get_angular_velocity"), &Box2DPhysicsBody::get_angular_velocity);
	// TODO mass, inertia, etc
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
	// TODO awake/can sleep
	ClassDB::bind_method(D_METHOD("set_fixed_rotation", "fixed_rotation"), &Box2DPhysicsBody::set_fixed_rotation);
	ClassDB::bind_method(D_METHOD("is_fixed_rotation"), &Box2DPhysicsBody::is_fixed_rotation);

	ClassDB::bind_method(D_METHOD("apply_force", "force", "point"), &Box2DPhysicsBody::apply_force);
	ClassDB::bind_method(D_METHOD("apply_central_force", "force"), &Box2DPhysicsBody::apply_central_force);
	ClassDB::bind_method(D_METHOD("apply_torque", "torque"), &Box2DPhysicsBody::apply_torque);
	ClassDB::bind_method(D_METHOD("apply_linear_impulse", "impulse", "point"), &Box2DPhysicsBody::apply_linear_impulse);
	ClassDB::bind_method(D_METHOD("apply_central_linear_impulse", "impulse"), &Box2DPhysicsBody::apply_central_linear_impulse);
	ClassDB::bind_method(D_METHOD("apply_torque_impulse", "impulse"), &Box2DPhysicsBody::apply_torque_impulse);

	// TODO add property hints (see RigidBody2D::_bind_methods)
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "linear_velocity"), "set_linear_velocity", "get_linear_velocity");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "angular_velocity"), "set_angular_velocity", "get_angular_velocity");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "linear_damping"), "set_linear_damping", "get_linear_damping");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "angular_damping"), "set_angular_damping", "get_angular_damping");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "gravity_scale"), "set_gravity_scale", "get_gravity_scale");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "type", PROPERTY_HINT_ENUM, "Static,Kinematic,Rigid"), "set_type", "get_type");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "bullet"), "set_bullet", "is_bullet");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "fixed_rotation"), "set_fixed_rotation", "is_fixed_rotation");

	BIND_ENUM_CONSTANT(MODE_RIGID);
	BIND_ENUM_CONSTANT(MODE_STATIC);
	BIND_ENUM_CONSTANT(MODE_KINEMATIC);
}

void Box2DPhysicsBody::on_parent_created(Node *) {
	if (create_b2Body()) {
		print_line("body created");
	}
}

String Box2DPhysicsBody::get_configuration_warning() const {
	String warning = Node2D::get_configuration_warning();

	if (!Object::cast_to<Box2DWorld>(get_parent())) {
		if (warning != String()) {
			warning += "\n\n";
		}
		warning += TTR("Box2DPhysicsBody only serves to provide bodies to a Box2DWorld node. Please only use it as a child of Box2DWorld.");
	}

	if (fixtures.size() == 0) {
		if (warning != String()) {
			warning += "\n\n";
		}
		warning += TTR("This node has no fixture, so it can't collide or interact with other objects.\nConsider adding a Box2DFixture subtype as a child to define its shape.");
	}

	return warning;
}

void Box2DPhysicsBody::set_linear_velocity(const Vector2 &p_vel) {
	if (body) {
		body->SetLinearVelocity(gd_to_b2(p_vel));
	}
	bodyDef.linearVelocity = gd_to_b2(p_vel);
}

Vector2 Box2DPhysicsBody::get_linear_velocity() const {
	if (body) {
		return b2_to_gd(body->GetLinearVelocity());
	}
	return b2_to_gd(bodyDef.linearVelocity); // TODO this is stupid...
}

void Box2DPhysicsBody::set_angular_velocity(const real_t p_omega) {
	if (body) {
		body->SetAngularVelocity(p_omega);
	}
	bodyDef.angularVelocity = p_omega;
}

real_t Box2DPhysicsBody::get_angular_velocity() const {
	if (body)
		return body->GetAngularVelocity();
	return bodyDef.angularVelocity;
}

void Box2DPhysicsBody::set_linear_damping(real_t p_damping) {
	if (body)
		body->SetLinearDamping(p_damping);
	bodyDef.linearDamping = p_damping;
}

real_t Box2DPhysicsBody::get_linear_damping() const {
	return bodyDef.linearDamping;
}

void Box2DPhysicsBody::set_angular_damping(real_t p_damping) {
	if (body)
		body->SetAngularDamping(p_damping);
	bodyDef.angularDamping = p_damping;
}

real_t Box2DPhysicsBody::get_angular_damping() const {
	return bodyDef.angularDamping;
}

void Box2DPhysicsBody::set_gravity_scale(real_t p_scale) {
	if (body)
		body->SetGravityScale(p_scale);
	bodyDef.gravityScale = p_scale;
}

real_t Box2DPhysicsBody::get_gravity_scale() const {
	return bodyDef.gravityScale;
}

void Box2DPhysicsBody::set_type(Mode p_type) {
	if (body)
		body->SetType(static_cast<b2BodyType>(p_type));
	bodyDef.type = static_cast<b2BodyType>(p_type);
}

Box2DPhysicsBody::Mode Box2DPhysicsBody::get_type() const {
	return static_cast<Mode>(bodyDef.type);
}

void Box2DPhysicsBody::set_bullet(bool p_ccd) {
	if (body)
		body->SetBullet(p_ccd);
	bodyDef.bullet = p_ccd;
}

bool Box2DPhysicsBody::is_bullet() const {
	return bodyDef.bullet;
}

void Box2DPhysicsBody::set_fixed_rotation(bool p_fixed) {
	if (body)
		body->SetFixedRotation(p_fixed);
	bodyDef.fixedRotation = p_fixed;
}

bool Box2DPhysicsBody::is_fixed_rotation() const {
	return bodyDef.fixedRotation;
}

void Box2DPhysicsBody::apply_force(const Vector2 &force, const Vector2 &point, bool wake) {
	ERR_FAIL_COND_MSG(!body, "b2Body is null.");
	body->ApplyForce(gd_to_b2(force), gd_to_b2(point), wake);
}

void Box2DPhysicsBody::apply_central_force(const Vector2 &force, bool wake) {
	ERR_FAIL_COND_MSG(!body, "b2Body is null.");
	body->ApplyForceToCenter(gd_to_b2(force), wake);
}

void Box2DPhysicsBody::apply_torque(real_t torque, bool wake) {
	ERR_FAIL_COND_MSG(!body, "b2Body is null.");
	body->ApplyTorque(torque, wake);
}

void Box2DPhysicsBody::apply_linear_impulse(const Vector2 &impulse, const Vector2 &point, bool wake) {
	ERR_FAIL_COND_MSG(!body, "b2Body is null.");
	body->ApplyLinearImpulse(gd_to_b2(impulse), gd_to_b2(point), wake);
}

void Box2DPhysicsBody::apply_central_linear_impulse(const Vector2 &impulse, bool wake) {
	ERR_FAIL_COND_MSG(!body, "b2Body is null.");
	body->ApplyLinearImpulseToCenter(gd_to_b2(impulse), wake);
}

void Box2DPhysicsBody::apply_torque_impulse(real_t impulse, bool wake) {
	ERR_FAIL_COND_MSG(!body, "b2Body is null.");
	body->ApplyAngularImpulse(impulse, wake);
}

Box2DPhysicsBody::Box2DPhysicsBody() :
		world_node(NULL) {
	set_notify_local_transform(true);
}

Box2DPhysicsBody::~Box2DPhysicsBody() {
	if (body && world_node) {
		destroy_b2Body();
	} // else Box2D has/will clean up body
}
