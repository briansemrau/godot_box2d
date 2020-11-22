#include "box2d_joints.h"
#include "box2d_world.h"

#include <core/engine.h>

void Box2DJoint::on_b2Joint_destroyed() {
	joint = NULL;
	update();
}

void Box2DJoint::update_joint_bodies() {
	// Clear preexisting state
	destroy_b2Joint();

	if (bodyA_cache) {
		Box2DPhysicsBody *body_a = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyA_cache));
		if (body_a)
			body_a->joints.erase(this);
		bodyA_cache = 0;
	}
	if (bodyB_cache) {
		Box2DPhysicsBody *body_b = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyB_cache));
		if (body_b)
			body_b->joints.erase(this);
		bodyB_cache = 0;
	}

	// If valid, update node cache
	Node *node_a = has_node(get_node_a()) ? get_node(get_node_a()) : (Node *)NULL;
	Node *node_b = has_node(get_node_b()) ? get_node(get_node_b()) : (Node *)NULL;

	if (!node_a || !node_b)
		return;

	Box2DPhysicsBody *body_a = Object::cast_to<Box2DPhysicsBody>(node_a);
	Box2DPhysicsBody *body_b = Object::cast_to<Box2DPhysicsBody>(node_b);

	if (!body_a || !body_b)
		return;

	bodyA_cache = body_a->get_instance_id();
	bodyB_cache = body_b->get_instance_id();
	// Make sure we receive body creation events
	body_a->joints.insert(this);
	body_a->joints.insert(this);

	// Create joint if b2Bodys are already created
	jointDef->bodyA = body_a->body;
	jointDef->bodyB = body_b->body;
	// TODO do we need to regenerate anchor points? or should that be done inside set_node_x?
	//      if in set_node_x, there should be additional parameters to give anchor point options
	if (jointDef->bodyA && jointDef->bodyB) {
		create_b2Joint();
	}
}

bool Box2DJoint::create_b2Joint() {
	if (world_node && !joint) {
		ERR_FAIL_COND_V(!world_node->world, false);
		ERR_FAIL_COND_V_MSG(!jointDef->bodyA, false, "Tried to create joint with invalid bodyA.");
		ERR_FAIL_COND_V_MSG(!jointDef->bodyB, false, "Tried to create joint with invalid bodyB.");

		// Allow subtypes to do final initialization before they're created
		init_b2JointDef();

		joint = world_node->world->CreateJoint(jointDef);
		joint->GetUserData().owner = this;

		print_line("joint created");
		return true;
	}
	return false;
}

bool Box2DJoint::destroy_b2Joint() {
	if (joint) {
		ERR_FAIL_COND_V(!world_node, false);
		ERR_FAIL_COND_V(!world_node->world, false)

		world_node->world->DestroyJoint(joint);
		joint = NULL;

		print_line("joint destroyed");
		return true;
	}
	return false;
}

void Box2DJoint::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {

			// Find the Box2DWorld
			Node *_ancestor = get_parent();
			Box2DWorld *new_world = NULL;
			while (_ancestor && !new_world) {
				new_world = Object::cast_to<Box2DWorld>(_ancestor);
			}

			// If new world, destroy joint.
			if (new_world != world_node) {
				if (world_node) {
					world_node->box2d_children.erase(this);
				}
				destroy_b2Joint();

				world_node = new_world;
			}

			if (Engine::get_singleton()->is_editor_hint() || get_tree()->is_debugging_collisions_hint()) {
				set_process_internal(true);
			}

		} break;
		case NOTIFICATION_EXIT_TREE: {

			set_process_internal(false);

		} break;
		case NOTIFICATION_POST_ENTER_TREE: {

			// After all bodies created in ENTER_TREE, create joint if valid
			update_joint_bodies();

		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {

			if (breaking_enabled && joint) {
				real_t force = get_reaction_force().length();
				real_t torque = get_reaction_torque();

				if ((max_force > 0 && force > max_force) || (max_torque > 0 && torque > max_torque)) {
					//emit_signal("joint_broken"); TODO
					queue_delete();
				}
			}

		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {

			//if (body) { // && body->IsAwake()) {
			//	set_block_transform_notify(true);
			//	set_transform(b2_to_gd(body->GetTransform()));
			//	set_block_transform_notify(false);
			//}
			if (Engine::get_singleton()->is_editor_hint() || get_tree()->is_debugging_collisions_hint()) {
				update();
			}

		} break;
		case NOTIFICATION_DRAW: {

			if (!Engine::get_singleton()->is_editor_hint() && !get_tree()->is_debugging_collisions_hint()) {
				break;
			}

			Color debug_col = Color(0.7, 0.6, 0.0, 0.5);
			if (breaking_enabled && joint) {
				// TODO redo this maybe, meh
				if (max_force > 0) {
					// TODO which max/abs functions should I be using? not that it matters, just for cleanliness
					real_t stress = get_reaction_force().length() / max_force;
					debug_col.r = MAX(1.0, stress * 2.0);
					debug_col.g = MAX(1.0, 1.0 - (stress * 2.0));
					debug_col.b = 0.0;
					if (max_torque > 0) {
						stress = abs(get_reaction_torque() / max_torque);
						debug_col.b = stress;
					}
				} else if (max_torque > 0) {
					real_t stress = abs(get_reaction_torque() / max_torque);
					debug_col.r = MAX(1.0, stress * 2.0);
					debug_col.g = MAX(1.0, (1.0 - stress) * 2.0);
					debug_col.b = 0.0;
				} else {
					// color UNCHANGED
				}
			}
			debug_draw(get_canvas_item(), debug_col);

		} break;
	}
}

void Box2DJoint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_node_a", "node_a"), &Box2DJoint::set_node_a);
	ClassDB::bind_method(D_METHOD("get_node_a"), &Box2DJoint::get_node_a);
	ClassDB::bind_method(D_METHOD("set_node_b", "node_b"), &Box2DJoint::set_node_b);
	ClassDB::bind_method(D_METHOD("get_node_b"), &Box2DJoint::get_node_b);
	ClassDB::bind_method(D_METHOD("set_collide_connected", "collide_connected"), &Box2DJoint::set_collide_connected);
	ClassDB::bind_method(D_METHOD("get_collide_connected"), &Box2DJoint::get_collide_connected);
	ClassDB::bind_method(D_METHOD("set_breaking_enabled", "breaking_enabled"), &Box2DJoint::set_breaking_enabled);
	ClassDB::bind_method(D_METHOD("is_breaking_enabled"), &Box2DJoint::is_breaking_enabled);
	ClassDB::bind_method(D_METHOD("set_max_force", "max_force"), &Box2DJoint::set_max_force);
	ClassDB::bind_method(D_METHOD("get_max_force"), &Box2DJoint::get_max_force);
	ClassDB::bind_method(D_METHOD("set_max_torque", "max_torque"), &Box2DJoint::set_max_torque);
	ClassDB::bind_method(D_METHOD("get_max_torque"), &Box2DJoint::get_max_torque);

	ClassDB::bind_method(D_METHOD("get_reaction_force"), &Box2DJoint::get_reaction_force);
	ClassDB::bind_method(D_METHOD("get_reaction_torque"), &Box2DJoint::get_reaction_torque);

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "node_a"), "set_node_a", "get_node_a");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "node_b"), "set_node_b", "get_node_b");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collide_connected"), "set_collide_connected", "get_collide_connected");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "breaking_enabled"), "set_breaking_enabled", "is_breaking_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_force"), "set_max_force", "get_max_force");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_torque"), "set_max_torque", "get_max_torque");
}

void Box2DJoint::on_parent_created(Node *p_parent) {
	Box2DPhysicsBody *body_a = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyA_cache));
	Box2DPhysicsBody *body_b = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyB_cache));

	if (body_a && body_b) {
		jointDef->bodyA = body_a->body;
		jointDef->bodyB = body_b->body;
		if (create_b2Joint()) {
			print_line("joint created");
		}
	}
}

String Box2DJoint::get_configuration_warning() const {
	String warning = Node2D::get_configuration_warning();

	if (!Object::cast_to<Box2DWorld>(get_parent())) {
		if (warning != String()) {
			warning += "\n\n";
		}
		warning += TTR("Box2DJoint only serves to create joints under a Box2DWorld node. Please only use it as a child of Box2DWorld.");
	}

	if (a.is_empty() || b.is_empty()) {
		if (warning != String()) {
			warning += "\n\n";
		}
		warning += TTR("This node does not have NodePaths defined for two Box2DPhysicsBody nodes, so it can't act on any bodies.");
	}

	return warning;
}

void Box2DJoint::set_node_a(const NodePath &p_node_a) {
	if (a == p_node_a)
		return;
	a = p_node_a;
	update_joint_bodies();
}

NodePath Box2DJoint::get_node_a() const {
	return a;
}

void Box2DJoint::set_node_b(const NodePath &p_node_b) {
	if (b == p_node_b)
		return;
	b = p_node_b;
	update_joint_bodies();
}

NodePath Box2DJoint::get_node_b() const {
	return b;
}

void Box2DJoint::set_collide_connected(bool p_collide) {
	jointDef->collideConnected = p_collide;
	if (joint)
		; // TODO recreate the joint // TODO test if this even works
}

bool Box2DJoint::get_collide_connected() const {
	return jointDef->collideConnected;
}

void Box2DJoint::set_breaking_enabled(bool p_enabled) {
	breaking_enabled = p_enabled;
	set_physics_process_internal(breaking_enabled);
}

bool Box2DJoint::is_breaking_enabled() const {
	return breaking_enabled;
}

void Box2DJoint::set_max_force(real_t p_max_force) {
	max_force = p_max_force;
}

real_t Box2DJoint::get_max_force() const {
	return max_force;
}

void Box2DJoint::set_max_torque(real_t p_max_torque) {
	max_torque = p_max_torque;
}

real_t Box2DJoint::get_max_torque() const {
	return max_torque;
}

Vector2 Box2DJoint::get_reaction_force() const {
	ERR_FAIL_COND_V_MSG(!joint, Vector2(), "b2Joint is null.");
	return b2_to_gd(joint->GetReactionForce(1.0f / get_physics_process_delta_time()));
}

real_t Box2DJoint::get_reaction_torque() const {
	ERR_FAIL_COND_V_MSG(!joint, real_t(), "b2Joint is null.");
	return joint->GetReactionTorque(1.0f / get_physics_process_delta_time());
}

Box2DJoint::Box2DJoint() :
		world_node(NULL), bodyA_cache(0), bodyB_cache(0), breaking_enabled(false), max_force(0.0f), max_torque(0.0f) {
}

Box2DJoint::~Box2DJoint() {
	if (joint && world_node) {
		destroy_b2Joint();
	} // else Box2D has/will clean up joint
}

void Box2DRevoluteJoint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_reference_angle"), &Box2DRevoluteJoint::get_reference_angle);
	ClassDB::bind_method(D_METHOD("get_joint_angle"), &Box2DRevoluteJoint::get_joint_angle);
	ClassDB::bind_method(D_METHOD("get_joint_speed"), &Box2DRevoluteJoint::get_joint_speed);

	ClassDB::bind_method(D_METHOD("set_limit_enabled", "limit_enabled"), &Box2DRevoluteJoint::set_limit_enabled);
	ClassDB::bind_method(D_METHOD("is_limit_enabled"), &Box2DRevoluteJoint::is_limit_enabled);
	ClassDB::bind_method(D_METHOD("set_upper_limit", "upper_limit"), &Box2DRevoluteJoint::set_upper_limit);
	ClassDB::bind_method(D_METHOD("get_upper_limit"), &Box2DRevoluteJoint::get_upper_limit);
	ClassDB::bind_method(D_METHOD("set_lower_limit", "lower_limit"), &Box2DRevoluteJoint::set_lower_limit);
	ClassDB::bind_method(D_METHOD("get_lower_limit"), &Box2DRevoluteJoint::get_lower_limit);

	ClassDB::bind_method(D_METHOD("set_limits", "lower", "upper"), &Box2DRevoluteJoint::set_limits);

	ClassDB::bind_method(D_METHOD("set_motor_enabled", "motor_enabled"), &Box2DRevoluteJoint::set_motor_enabled);
	ClassDB::bind_method(D_METHOD("is_motor_enabled"), &Box2DRevoluteJoint::is_motor_enabled);
	ClassDB::bind_method(D_METHOD("set_motor_speed", "motor_speed"), &Box2DRevoluteJoint::set_motor_speed);
	ClassDB::bind_method(D_METHOD("get_motor_speed"), &Box2DRevoluteJoint::get_motor_speed);
	ClassDB::bind_method(D_METHOD("set_max_motor_torque", "max_motor_torque"), &Box2DRevoluteJoint::set_max_motor_torque);
	ClassDB::bind_method(D_METHOD("get_max_motor_torque"), &Box2DRevoluteJoint::get_max_motor_torque);

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "limit_enabled"), "set_limit_enabled", "is_limit_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "upper_limit"), "set_upper_limit", "get_upper_limit");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "lower_limit"), "set_lower_limit", "get_lower_limit");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "motor_enabled"), "set_motor_enabled", "is_motor_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "motor_speed"), "set_motor_speed", "get_motor_speed");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_motor_torque"), "set_max_motor_torque", "get_max_motor_torque");
}

void Box2DRevoluteJoint::init_b2JointDef() {
	jointDef.Initialize(jointDef.bodyA, jointDef.bodyB, gd_to_b2(get_transform().get_origin()));
}

void Box2DRevoluteJoint::debug_draw(RID p_to_rid, Color p_color) {
	Point2 p1;
	Point2 p2;
	b2RevoluteJoint *j;
	if (j = static_cast<b2RevoluteJoint *>(get_b2Joint())) {
		p1 += b2_to_gd(j->GetAnchorA()) - get_position();
		p2 += b2_to_gd(j->GetAnchorB()) - get_position();
	}
	draw_arc(p1, 5, 0, Math_PI * 2.0f, 12, p_color, 2.0f);
	draw_circle(p2, 2, p_color);

	if (jointDef.enableLimit) {
		draw_line(p1, p1 + Point2(8, 0).rotated(jointDef.referenceAngle), p_color);
		draw_line(p1, p1 + Point2(8, 0).rotated(jointDef.lowerAngle), p_color);
		draw_line(p1, p1 + Point2(8, 0).rotated(jointDef.upperAngle), p_color);
	}
	if (jointDef.enableMotor) {
		float a = jointDef.referenceAngle;
		Color c = jointDef.motorSpeed > 0 ? Color(0.0, 1.0, 0.0, 0.5) : Color(1.0, 0.0, 0.0, 0.5);
		float arclen = jointDef.motorSpeed;
		draw_arc(p1, 7, a, a + arclen, MAX(static_cast<int>(5.0f * arclen), 2), c, 2.0f);
		if (j) {
			float torqueUsage = j->GetReactionTorque(1.0f / get_physics_process_delta_time()) / j->GetMaxMotorTorque();
			arclen *= torqueUsage;
			c = Color(1.0, 1.0, 0.0, 0.5);
			draw_arc(p1, 9, a, a + arclen, MAX(static_cast<int>(5.0f * arclen), 2), c, 2.0f);
		}
	}
}

real_t Box2DRevoluteJoint::get_reference_angle() const {
	ERR_FAIL_COND_V_MSG(!get_b2Joint(), real_t(), "b2Joint is null.");
	return static_cast<b2RevoluteJoint *>(get_b2Joint())->GetReferenceAngle();
}

real_t Box2DRevoluteJoint::get_joint_angle() const {
	ERR_FAIL_COND_V_MSG(!get_b2Joint(), real_t(), "b2Joint is null.");
	return static_cast<b2RevoluteJoint *>(get_b2Joint())->GetJointAngle();
}

real_t Box2DRevoluteJoint::get_joint_speed() const {
	ERR_FAIL_COND_V_MSG(!get_b2Joint(), real_t(), "b2Joint is null.");
	return static_cast<b2RevoluteJoint *>(get_b2Joint())->GetJointSpeed();
}

void Box2DRevoluteJoint::set_limit_enabled(bool p_enabled) {
	if (get_b2Joint())
		static_cast<b2RevoluteJoint *>(get_b2Joint())->EnableLimit(p_enabled);
	jointDef.enableLimit = p_enabled;
}

bool Box2DRevoluteJoint::is_limit_enabled() const {
	return jointDef.enableLimit;
}

void Box2DRevoluteJoint::set_upper_limit(real_t p_angle) {
	if (get_b2Joint())
		static_cast<b2RevoluteJoint *>(get_b2Joint())->SetLimits(get_lower_limit(), p_angle);
	jointDef.upperAngle = p_angle;
}

real_t Box2DRevoluteJoint::get_upper_limit() const {
	return jointDef.upperAngle;
}

void Box2DRevoluteJoint::set_lower_limit(real_t p_angle) {
	if (get_b2Joint())
		static_cast<b2RevoluteJoint *>(get_b2Joint())->SetLimits(p_angle, get_upper_limit());
	jointDef.lowerAngle = p_angle;
}

real_t Box2DRevoluteJoint::get_lower_limit() const {
	return jointDef.lowerAngle;
}

void Box2DRevoluteJoint::set_limits(real_t p_lower, real_t p_upper) {
	if (get_b2Joint())
		static_cast<b2RevoluteJoint *>(get_b2Joint())->SetLimits(p_lower, p_upper);
	jointDef.lowerAngle = p_lower;
	jointDef.upperAngle = p_upper;
}

void Box2DRevoluteJoint::set_motor_enabled(bool p_enabled) {
	if (get_b2Joint())
		static_cast<b2RevoluteJoint *>(get_b2Joint())->EnableMotor(p_enabled);
	jointDef.enableMotor = p_enabled;
}

bool Box2DRevoluteJoint::is_motor_enabled() const {
	return jointDef.enableMotor;
}

void Box2DRevoluteJoint::set_motor_speed(real_t p_speed) {
	if (get_b2Joint())
		static_cast<b2RevoluteJoint *>(get_b2Joint())->SetMotorSpeed(p_speed);
	jointDef.motorSpeed = p_speed;
}

real_t Box2DRevoluteJoint::get_motor_speed() const {
	return jointDef.motorSpeed;
}

void Box2DRevoluteJoint::set_max_motor_torque(real_t p_torque) {
	if (get_b2Joint())
		static_cast<b2RevoluteJoint *>(get_b2Joint())->SetMaxMotorTorque(p_torque);
	jointDef.maxMotorTorque = p_torque;
}

real_t Box2DRevoluteJoint::get_max_motor_torque() const {
	return jointDef.maxMotorTorque;
}

void Box2DWeldJoint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &Box2DWeldJoint::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &Box2DWeldJoint::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_damping", "hz"), &Box2DWeldJoint::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &Box2DWeldJoint::get_damping);
	//ClassDB::bind_method(D_METHOD("set_", ""), &Box2DWeldJoint::set_);
	//ClassDB::bind_method(D_METHOD("get_"), &Box2DWeldJoint::get_);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "stiffness"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "damping"), "set_damping", "get_damping");
	//ADD_PROPERTY(PropertyInfo(Variant::BOOL, ""), "set_", "get_");
}

void Box2DWeldJoint::init_b2JointDef() {
	jointDef.Initialize(jointDef.bodyA, jointDef.bodyB, gd_to_b2(get_transform().get_origin()));
}

void Box2DWeldJoint::debug_draw(RID p_to_rid, Color p_color) {
	Point2 p1;
	Point2 p2;
	b2RevoluteJoint *j;
	if (j = static_cast<b2RevoluteJoint *>(get_b2Joint())) {
		p1 += b2_to_gd(j->GetAnchorA()) - get_position();
		p2 += b2_to_gd(j->GetAnchorB()) - get_position();
	}
	draw_line(p1 + Point2(-5, 0), p1 + Point2(+5, 0), p_color, 2);
	draw_line(p1 + Point2(0, -5), p1 + Point2(0, +5), p_color, 2);
}

void Box2DWeldJoint::set_stiffness(real_t p_hz) {
	if (get_b2Joint())
		static_cast<b2WeldJoint *>(get_b2Joint())->SetStiffness(p_hz);
	jointDef.stiffness = p_hz;
}

real_t Box2DWeldJoint::get_stiffness() const {
	return jointDef.stiffness;
}

void Box2DWeldJoint::set_damping(real_t p_damping) {
	if (get_b2Joint())
		static_cast<b2WeldJoint *>(get_b2Joint())->SetDamping(p_damping);
	jointDef.damping = p_damping;
}

real_t Box2DWeldJoint::get_damping() const {
	return jointDef.damping;
}
