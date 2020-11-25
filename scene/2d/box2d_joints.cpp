#include "box2d_joints.h"
#include "box2d_world.h"

#include <core/engine.h>

/**
* @author Brian Semrau
*/

void Box2DJoint::on_b2Joint_destroyed() {
	joint = NULL;

	// Check if destroyed because nodes were freed
	// TODO instead, attach signal Box2DPhysicsBody.freed (name uncertain) to Box2DJoint within update_joint_bodies
	Node *node_a = has_node(get_nodepath_a()) ? get_node(get_nodepath_a()) : (Node *)NULL;
	Node *node_b = has_node(get_nodepath_b()) ? get_node(get_nodepath_b()) : (Node *)NULL;
	Box2DPhysicsBody *body_a = Object::cast_to<Box2DPhysicsBody>(node_a);
	Box2DPhysicsBody *body_b = Object::cast_to<Box2DPhysicsBody>(node_b);

	// both nodes freed is possible
	if (!body_a) {
		a = NodePath();
		update_joint_bodies();
	}
	if (!body_b) {
		b = NodePath();
		update_joint_bodies();
	}

	update();
}

void Box2DJoint::update_joint_bodies(bool p_recalc_if_unchanged) {
	// This is called whenever the joint's body nodes are reassigned via set_node_a/b.
	// If the bodies haven't actually changed, it is assumed that the joint should
	// be unchanged, unless `p_force_reinit = true`

	// Check if bodies have changed
	Node *node_a = has_node(get_nodepath_a()) ? get_node(get_nodepath_a()) : (Node *)NULL;
	Node *node_b = has_node(get_nodepath_b()) ? get_node(get_nodepath_b()) : (Node *)NULL;
	Box2DPhysicsBody *body_a = Object::cast_to<Box2DPhysicsBody>(node_a);
	Box2DPhysicsBody *body_b = Object::cast_to<Box2DPhysicsBody>(node_b);

	const bool joint_invalid = !joint || (!joint->GetBodyA() || !joint->GetBodyB());

	b2Body *body_a_body = body_a ? body_a->body : NULL;
	b2Body *body_b_body = body_b ? body_b->body : NULL;
	b2Body *joint_body_a = joint ? joint->GetBodyA() : NULL;
	b2Body *joint_body_b = joint ? joint->GetBodyB() : NULL;
	const bool bodies_unchanged = (body_a_body == joint_body_a) && (body_b_body == joint_body_b);

	const bool recalc = joint_invalid || (p_recalc_if_unchanged && bodies_unchanged);

	if (recalc) {
		// Clear preexisting state
		destroy_b2Joint();

		// Clear previous cache
		if (bodyA_cache) {
			Box2DPhysicsBody *body_a = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyA_cache));
			if (body_a) {
				body_a->joints.erase(this);
				body_a->disconnect("tree_entered", this, "_node_a_tree_entered");
			}
			bodyA_cache = 0;
		}
		if (bodyB_cache) {
			Box2DPhysicsBody *body_b = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyB_cache));
			if (body_b) {
				body_b->joints.erase(this);
				body_b->disconnect("tree_entered", this, "_node_b_tree_entered");
			}
			bodyB_cache = 0;
		}

		// If valid, update node cache

		if (!node_a || !node_b || !body_a || !body_b) {
			jointDef->bodyA = NULL;
			jointDef->bodyB = NULL;
			return;
		}

		bodyA_cache = body_a->get_instance_id();
		bodyB_cache = body_b->get_instance_id();
		// Make sure we receive b2Body creation/deletion events
		body_a->joints.insert(this);
		body_b->joints.insert(this);
		body_a->connect("tree_entered", this, "_node_a_tree_entered");
		body_b->connect("tree_entered", this, "_node_b_tree_entered");

		// Create joint if b2Bodys are already created

		jointDef->bodyA = body_a->body;
		jointDef->bodyB = body_b->body;

		if (jointDef->bodyA && jointDef->bodyB) {
			// Allow subtypes to do final initialization
			b2Vec2 joint_pos = get_b2_pos();
			init_b2JointDef(joint_pos);

			if (!broken)
				create_b2Joint();
		}
	}
}

bool Box2DJoint::create_b2Joint() {
	if (world_node && !joint) {
		ERR_FAIL_COND_V(!world_node->world, false);
		ERR_FAIL_COND_V_MSG(!jointDef->bodyA, false, "Tried to create joint with invalid bodyA.");
		ERR_FAIL_COND_V_MSG(!jointDef->bodyB, false, "Tried to create joint with invalid bodyB.");

		// Don't reinit the joint. That should only happen when calling update_joint_bodies.

		joint = world_node->world->CreateJoint(jointDef);
		joint->GetUserData().owner = this;

		// TODO determine whether we should wake bodies
		// Cleanest solution may be to add a param `p_wake_bodies` to set_broken, set_node_a/b
		// Those params would pass to this function with new param `p_wake_bodies`
		joint->GetBodyA()->SetAwake(true);
		joint->GetBodyB()->SetAwake(true);

		//print_line("joint created");
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

		//print_line("joint destroyed");
		return true;
	}
	return false;
}

void Box2DJoint::on_node_predelete(Box2DPhysicsBody *node) {
	ObjectID id = node->get_instance_id();
	if (bodyA_cache == id) {
		set_nodepath_a(NodePath());
	} else if (bodyB_cache == id) {
		set_nodepath_b(NodePath());
	} else {
		ERR_FAIL_MSG("A joint's callback was triggered from a node it does not recognize.");
	}
}

void Box2DJoint::_node_a_tree_entered() {
	// Update path in case node moved
	// TODO figure out: is this weird to do? This is making the nodepath property "sticky"
	Node *node = Object::cast_to<Node>(ObjectDB::get_instance(bodyA_cache));
	if (node)
		a = node->get_path();
	// Don't update_joint_bodies because the body node hasn't changed
}

void Box2DJoint::_node_b_tree_entered() {
	// Update path in case node moved
	// TODO figure out: is this weird to do? This is making the nodepath property "sticky"
	Node *node = Object::cast_to<Node>(ObjectDB::get_instance(bodyB_cache));
	if (node)
		b = node->get_path();
	// Don't update_joint_bodies because the body node hasn't changed
}

b2Vec2 Box2DJoint::get_b2_pos() const {
	ERR_FAIL_COND_V(!world_node, b2Vec2());
	return gd_to_b2(get_global_position() - world_node->get_global_position());
}

void Box2DJoint::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PREDELETE: {

			Box2DPhysicsBody *body_a = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyA_cache));
			Box2DPhysicsBody *body_b = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyB_cache));
			if (body_a)
				body_a->joints.erase(this);
			if (body_b)
				body_b->joints.erase(this);

		} break;
		case NOTIFICATION_ENTER_TREE: {

			// Find the Box2DWorld
			Node *_ancestor = get_parent();
			Box2DWorld *new_world = NULL;
			while (_ancestor && !new_world) {
				new_world = Object::cast_to<Box2DWorld>(_ancestor);
				_ancestor = _ancestor->get_parent();
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

			destroy_b2Joint();
			set_process_internal(false);

		} break;
		case NOTIFICATION_POST_ENTER_TREE: {

			// After all bodies created in ENTER_TREE, create joint if valid.
			// If just exiting/entering tree, joint isn't at risk for reinitialization.
			update_joint_bodies();

		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {

			if (breaking_enabled && joint) {
				Vector2 force = get_reaction_force();
				real_t torque = abs(get_reaction_torque());

				const bool exceeded_force = max_force > 0 && force.length() > max_force;
				const bool exceeded_torque = max_torque > 0 && torque > max_torque;

				if (exceeded_force || exceeded_torque) {
					emit_signal("joint_broken", force, torque);
					set_broken(true);
				}
			}

		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {

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
					// TODO which max/abs functions should I be using? not that it matters, just for cleanliness.
					// Using a macro MAX and function abs feels dirty.
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
	ClassDB::bind_method(D_METHOD("set_node_a", "node_a", "force_reinit"), &Box2DJoint::set_nodepath_a), DEFVAL(false);
	ClassDB::bind_method(D_METHOD("get_node_a"), &Box2DJoint::get_nodepath_a);
	ClassDB::bind_method(D_METHOD("set_node_b", "node_b", "force_reinit"), &Box2DJoint::set_nodepath_b), DEFVAL(false);
	ClassDB::bind_method(D_METHOD("get_node_b"), &Box2DJoint::get_nodepath_b);

	// TODO temporary until resolved: https://github.com/godotengine/godot/issues/43821
	// remember to update ADD_PROPERTY set functions when this is removed
	ClassDB::bind_method(D_METHOD("_set_node_a", "node_a"), &Box2DJoint::_set_nodepath_a);
	ClassDB::bind_method(D_METHOD("_set_node_b", "node_b"), &Box2DJoint::_set_nodepath_b);

	ClassDB::bind_method(D_METHOD("set_collide_connected", "collide_connected"), &Box2DJoint::set_collide_connected);
	ClassDB::bind_method(D_METHOD("get_collide_connected"), &Box2DJoint::get_collide_connected);
	ClassDB::bind_method(D_METHOD("set_broken", "broken"), &Box2DJoint::set_broken);
	ClassDB::bind_method(D_METHOD("is_broken"), &Box2DJoint::is_broken);
	ClassDB::bind_method(D_METHOD("set_breaking_enabled", "breaking_enabled"), &Box2DJoint::set_breaking_enabled);
	ClassDB::bind_method(D_METHOD("is_breaking_enabled"), &Box2DJoint::is_breaking_enabled);
	ClassDB::bind_method(D_METHOD("set_free_on_break", "free_on_break"), &Box2DJoint::set_free_on_break);
	ClassDB::bind_method(D_METHOD("get_free_on_break"), &Box2DJoint::get_free_on_break);
	ClassDB::bind_method(D_METHOD("set_max_force", "max_force"), &Box2DJoint::set_max_force);
	ClassDB::bind_method(D_METHOD("get_max_force"), &Box2DJoint::get_max_force);
	ClassDB::bind_method(D_METHOD("set_max_torque", "max_torque"), &Box2DJoint::set_max_torque);
	ClassDB::bind_method(D_METHOD("get_max_torque"), &Box2DJoint::get_max_torque);

	ClassDB::bind_method(D_METHOD("get_reaction_force"), &Box2DJoint::get_reaction_force);
	ClassDB::bind_method(D_METHOD("get_reaction_torque"), &Box2DJoint::get_reaction_torque);

	ClassDB::bind_method(D_METHOD("_node_a_tree_entered"), &Box2DJoint::_node_a_tree_entered);
	ClassDB::bind_method(D_METHOD("_node_b_tree_entered"), &Box2DJoint::_node_b_tree_entered);

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "node_a"), "_set_node_a", "get_node_a");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "node_b"), "_set_node_b", "get_node_b");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collide_connected"), "set_collide_connected", "get_collide_connected");
	ADD_GROUP("Breaking", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "broken"), "set_broken", "is_broken");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "breaking_enabled"), "set_breaking_enabled", "is_breaking_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "free_on_break"), "set_free_on_break", "get_free_on_break");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_force", PROPERTY_HINT_EXP_RANGE, "0,65535,0.01"), "set_max_force", "get_max_force");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_torque", PROPERTY_HINT_EXP_RANGE, "0,65535,0.01"), "set_max_torque", "get_max_torque");

	ADD_SIGNAL(MethodInfo("joint_broken", PropertyInfo(Variant::VECTOR2, "break_force"), PropertyInfo(Variant::REAL, "break_torque")));
}

void Box2DJoint::on_parent_created(Node *p_parent) {
	Box2DPhysicsBody *body_a = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyA_cache));
	Box2DPhysicsBody *body_b = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyB_cache));

	if (body_a && body_b) {
		jointDef->bodyA = body_a->body;
		jointDef->bodyB = body_b->body;
		if (!broken)
			if (create_b2Joint()) { // TODO this might need to call update_joint_bodies because joint may not be initialized
				print_line("JOINT CREATED FROM CALLBACK");
			}
	}
}

String Box2DJoint::get_configuration_warning() const {
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
		warning += TTR("Box2DJoint only serves to create joints under the hierarchy of a Box2DWorld node. Please only use it as a descendant of Box2DWorld.");
	}

	if (a.is_empty() || b.is_empty()) {
		if (warning != String()) {
			warning += "\n\n";
		}
		warning += TTR("This node does not have NodePaths defined for two Box2DPhysicsBody nodes, so it can't act on any bodies.");
	}

	return warning;
}

void Box2DJoint::set_nodepath_a(const NodePath &p_node_a, bool p_force_reinit) {
	if (a == p_node_a)
		return;
	a = p_node_a;
	if (Engine::get_singleton()->is_editor_hint()) {
		update_configuration_warning();
	}
	if (is_inside_tree()) {
		update_joint_bodies(p_force_reinit);
	}
}

NodePath Box2DJoint::get_nodepath_a() const {
	return a;
}

void Box2DJoint::set_nodepath_b(const NodePath &p_node_b, bool p_force_reinit) {
	if (b == p_node_b)
		return;
	b = p_node_b;
	if (Engine::get_singleton()->is_editor_hint()) {
		update_configuration_warning();
	}
	if (is_inside_tree()) {
		update_joint_bodies(p_force_reinit);
	}
}

NodePath Box2DJoint::get_nodepath_b() const {
	return b;
}

void Box2DJoint::set_collide_connected(bool p_collide) {
	jointDef->collideConnected = p_collide;
	if (joint) {
		// Joints anchors are not reinitialized using these functions
		if (!broken) {
			destroy_b2Joint();
			create_b2Joint();
		}
	}
}

bool Box2DJoint::get_collide_connected() const {
	return jointDef->collideConnected;
}

void Box2DJoint::set_broken(bool p_broken) {
	if (p_broken && !broken) {
		// TODO fix: when broken, collision exception between bodies does not always reset
		// This may be a Box2D issue. Needs testing.
		destroy_b2Joint();
		if (free_on_break) {
			queue_delete();
		}
	} else if (!p_broken && broken) {
		if (jointDef->bodyA && jointDef->bodyB) {
			create_b2Joint();
		}
	}
	broken = p_broken;
}

bool Box2DJoint::is_broken() const {
	return broken;
}

void Box2DJoint::set_breaking_enabled(bool p_enabled) {
	breaking_enabled = p_enabled;
	set_physics_process_internal(breaking_enabled);
}

bool Box2DJoint::is_breaking_enabled() const {
	return breaking_enabled;
}

void Box2DJoint::set_free_on_break(bool p_should_free) {
	free_on_break = p_should_free;
}

bool Box2DJoint::get_free_on_break() const {
	return free_on_break;
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
	if (broken)
		return Vector2(); // Don't print a fail message for intended behavior
	ERR_FAIL_COND_V_MSG(!joint, Vector2(), "b2Joint is null.");
	return b2_to_gd(joint->GetReactionForce(1.0f / get_physics_process_delta_time()));
}

real_t Box2DJoint::get_reaction_torque() const {
	if (broken)
		return real_t(); // Don't print a fail message for intended behavior
	ERR_FAIL_COND_V_MSG(!joint, real_t(), "b2Joint is null.");
	return joint->GetReactionTorque(1.0f / get_physics_process_delta_time());
}

Box2DJoint::Box2DJoint() :
		jointDef(NULL),
		joint(NULL),
		world_node(NULL),
		bodyA_cache(0),
		bodyB_cache(0),
		broken(false),
		breaking_enabled(false),
		free_on_break(false),
		max_force(0.0f),
		max_torque(0.0f) {
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

void Box2DRevoluteJoint::init_b2JointDef(const b2Vec2 &p_joint_pos) {
	jointDef.Initialize(jointDef.bodyA, jointDef.bodyB, p_joint_pos);
}

void Box2DRevoluteJoint::debug_draw(RID p_to_rid, Color p_color) {
	Point2 p1;
	Point2 p2;
	b2RevoluteJoint *j = static_cast<b2RevoluteJoint *>(get_b2Joint());
	if (j) {
		b2Vec2 b2p1 = j->GetAnchorA();
		b2p1 -= get_b2_pos();
		b2Vec2 b2p2 = j->GetAnchorB();
		b2p2 -= get_b2_pos();
		p1 += b2_to_gd(b2p1); // - get_position();
		p2 += b2_to_gd(b2p2); // - get_position();
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

void Box2DWeldJoint::init_b2JointDef(const b2Vec2 &p_joint_pos) {
	jointDef.Initialize(jointDef.bodyA, jointDef.bodyB, p_joint_pos);
}

void Box2DWeldJoint::debug_draw(RID p_to_rid, Color p_color) {
	Point2 p1;
	Point2 p2;
	b2RevoluteJoint *j;
	if (j = static_cast<b2RevoluteJoint *>(get_b2Joint())) {
		b2Vec2 b2p1 = j->GetAnchorA();
		b2p1 -= get_b2_pos();
		b2Vec2 b2p2 = j->GetAnchorB();
		b2p2 -= get_b2_pos();
		p1 += b2_to_gd(b2p1);
		p2 += b2_to_gd(b2p2);
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
