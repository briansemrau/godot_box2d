#include "box2d_joints.h"
#include "box2d_world.h"

#include <core/engine.h>

/**
* @author Brian Semrau
*/

void Box2DJoint::on_b2Joint_destroyed() {
	joint = NULL;

	// Check if destroyed because nodes were freed
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

void Box2DJoint::on_node_predelete(Box2DPhysicsBody *node) {
	ObjectID id = node->get_instance_id();
	if (bodyA_cache == id) {
		set_nodepath_a(NodePath());
	} else if (bodyB_cache == id) {
		set_nodepath_b(NodePath());
	} else {
		ERR_FAIL_MSG("A joint's callback was triggered from a node it does not recognize.");
	}
	// stop drawing every frame
	set_process_internal(false);
}

void Box2DJoint::update_joint_bodies() {
	// This is called whenever the joint's body nodes are reassigned via set_node_a/b.
	// If the bodies haven't actually changed, it is assumed that the joint should
	// be unchanged.

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
	const bool bodies_changed = (body_a_body != joint_body_a) || (body_b_body != joint_body_b);

	// Only recreate the joint if it will do anything
	if (joint_invalid || bodies_changed) {
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

		if (!node_a || !node_b || !body_a || !body_b || (body_a == body_b)) {
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

		// Init and create joint
		jointDef->bodyA = body_a->body;
		jointDef->bodyB = body_b->body;

		recreate_joint();
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

void Box2DJoint::recreate_joint() {
	destroy_b2Joint();

	if (is_valid()) {
		// Allow subtypes to do final initialization
		b2Vec2 joint_pos = get_b2_pos();
		init_b2JointDef(joint_pos);

		if (!broken)
			create_b2Joint();
	}
}

b2Vec2 Box2DJoint::get_b2_pos() const {
	ERR_FAIL_COND_V(!world_node, b2Vec2());
	return gd_to_b2(get_global_position() - world_node->get_global_position());
}

void Box2DJoint::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PREDELETE: {
			destroy_b2Joint();
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
			// Will attempt to recreate in POST_ENTER_TREE.
			if (new_world != world_node) {
				if (world_node) {
					world_node->joints.erase(this);
				}
				destroy_b2Joint();

				world_node = new_world;
			}

			if (Engine::get_singleton()->is_editor_hint() || get_tree()->is_debugging_collisions_hint()) {
				// start drawing every frame for debug draw
				set_process_internal(true);
				set_notify_transform(true);
			}
		} break;

		case NOTIFICATION_EXIT_TREE: {
			// Detatch from body nodes
			Box2DPhysicsBody *body_a = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyA_cache));
			Box2DPhysicsBody *body_b = Object::cast_to<Box2DPhysicsBody>(ObjectDB::get_instance(bodyB_cache));
			if (body_a) {
				body_a->joints.erase(this);
				body_a->disconnect("tree_entered", this, "_node_a_tree_entered");
				bodyA_cache = 0;
			}
			if (body_b) {
				body_b->joints.erase(this);
				body_b->disconnect("tree_entered", this, "_node_b_tree_entered");
				bodyB_cache = 0;
			}

			destroy_b2Joint();

			// stop debug drawing
			set_process_internal(false);
			set_notify_transform(false);
		} break;

		case NOTIFICATION_POST_ENTER_TREE: {
			// After all bodies created in ENTER_TREE, create joint if valid.
			// If just exiting/entering tree, joint isn't at risk for reinitialization.
			update_joint_bodies();
		} break;

		case NOTIFICATION_TRANSFORM_CHANGED: {
			// Changing transform only does anything if reinitialize_joint() is called

			// Update in editor to represent what initialized state will look like
			if (Engine::get_singleton()->is_editor_hint()) {
				on_editor_transforms_changed();
			}
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

			Color debug_col = Color(0.5f, 0.8f, 0.8f);
			if (!is_enabled()) {
				debug_col = Color(0.5f, 0.5f, 0.3f);
			} else if (broken) {
				debug_col = Color(0.5f, 0.25f, 0.0f);
			} else if (joint) {
				// TODO redo this maybe, meh
				if (max_force > 0) {
					real_t stress = get_reaction_force().length() / max_force;
					debug_col.r = CLAMP(stress * 2.0f, 0.0f, 1.0f);
					debug_col.g = CLAMP(2.0f - (stress * 2.0f), 0.0f, 1.0f);
					debug_col.b = 0.0f;
					if (max_torque > 0) {
						stress = Math::abs(get_reaction_torque() / max_torque);
						debug_col.b = stress;
					}
				} else if (max_torque > 0) {
					real_t stress = Math::abs(get_reaction_torque() / max_torque);
					debug_col.r = CLAMP(stress * 2.0f, 0.0f, 1.0f);
					debug_col.g = CLAMP(2.0f - (stress * 2.0f), 0.0f, 1.0f);
					debug_col.b = 0.0f;
				} else {
					// color UNCHANGED
				}
			}
			if (!is_valid()) {
				// joint invalid
				debug_col = Color(1.0f, 0.0f, 0.0f);
			}
			debug_draw(get_canvas_item(), debug_col);

		} break;
	}
}

void Box2DJoint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_node_a", "node_a"), &Box2DJoint::set_nodepath_a);
	ClassDB::bind_method(D_METHOD("get_node_a"), &Box2DJoint::get_nodepath_a);
	ClassDB::bind_method(D_METHOD("set_node_b", "node_b"), &Box2DJoint::set_nodepath_b);
	ClassDB::bind_method(D_METHOD("get_node_b"), &Box2DJoint::get_nodepath_b);

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

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "node_a"), "set_node_a", "get_node_a");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "node_b"), "set_node_b", "get_node_b");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collide_connected"), "set_collide_connected", "get_collide_connected");
	ADD_GROUP("Breaking", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "broken"), "set_broken", "is_broken");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "breaking_enabled"), "set_breaking_enabled", "is_breaking_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "free_on_break"), "set_free_on_break", "get_free_on_break");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_force", PROPERTY_HINT_EXP_RANGE, "0,65535,0.01"), "set_max_force", "get_max_force");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_torque", PROPERTY_HINT_EXP_RANGE, "0,65535,0.01"), "set_max_torque", "get_max_torque");

	ADD_SIGNAL(MethodInfo("joint_broken", PropertyInfo(Variant::VECTOR2, "break_force"), PropertyInfo(Variant::REAL, "break_torque")));
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

void Box2DJoint::set_nodepath_a(const NodePath &p_node_a) {
	if (a == p_node_a)
		return;
	a = p_node_a;
	if (Engine::get_singleton()->is_editor_hint()) {
		update_configuration_warning();
	}
	if (is_inside_tree()) {
		update_joint_bodies();
	}
}

NodePath Box2DJoint::get_nodepath_a() const {
	return a;
}

void Box2DJoint::set_nodepath_b(const NodePath &p_node_b) {
	if (b == p_node_b)
		return;
	b = p_node_b;
	if (Engine::get_singleton()->is_editor_hint()) {
		update_configuration_warning();
	}
	if (is_inside_tree()) {
		update_joint_bodies();
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
		if (is_valid()) {
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

bool Box2DJoint::is_enabled() const {
	// This impl might not be what we want
	if (joint)
		return joint->IsEnabled();
	return is_valid() && jointDef->bodyA->IsEnabled() && jointDef->bodyB->IsEnabled();
}

bool Box2DJoint::is_valid() const {
	return jointDef->bodyA && jointDef->bodyB;
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
		max_torque(0.0f) {}

Box2DJoint::~Box2DJoint() {
	if (joint && world_node) {
		WARN_PRINT("b2Joint is being deleted in destructor, not NOTIFICATION_PREDELETE.");
		destroy_b2Joint();
	} // else Box2D has/will clean up joint
}

void Box2DRevoluteJoint::on_editor_transforms_changed() {
	if (editor_use_default_anchors) {
		// We need set reset anchors and reinit the jointDef
		if (is_inside_tree()) {
			reset_joint_anchors();
			_change_notify();
		}
	} else {
		// Some relative coordinate has changed
		// We want to move our anchors to keep their relative location to each body the same
		// This leaves the jointDef anchors unchanged
		anchor_a = to_local(b2_to_gd(jointDef.bodyA->GetWorldPoint(jointDef.localAnchorA)));
		anchor_b = to_local(b2_to_gd(jointDef.bodyB->GetWorldPoint(jointDef.localAnchorB)));
		_change_notify();
	}
}

void Box2DRevoluteJoint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_reference_angle"), &Box2DRevoluteJoint::get_reference_angle);
	ClassDB::bind_method(D_METHOD("get_joint_angle"), &Box2DRevoluteJoint::get_joint_angle);
	ClassDB::bind_method(D_METHOD("get_joint_speed"), &Box2DRevoluteJoint::get_joint_speed);

	ClassDB::bind_method(D_METHOD("set_editor_use_default_anchors", "default"), &Box2DRevoluteJoint::set_editor_use_default_anchors);
	ClassDB::bind_method(D_METHOD("get_editor_use_default_anchors"), &Box2DRevoluteJoint::get_editor_use_default_anchors);
	ClassDB::bind_method(D_METHOD("set_anchor_a", "anchor_a"), &Box2DRevoluteJoint::set_anchor_a);
	ClassDB::bind_method(D_METHOD("get_anchor_a"), &Box2DRevoluteJoint::get_anchor_a);
	ClassDB::bind_method(D_METHOD("set_anchor_b", "anchor_b"), &Box2DRevoluteJoint::set_anchor_b);
	ClassDB::bind_method(D_METHOD("get_anchor_b"), &Box2DRevoluteJoint::get_anchor_b);
	ClassDB::bind_method(D_METHOD("reset_joint_anchors"), &Box2DRevoluteJoint::reset_joint_anchors);

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
	ClassDB::bind_method(D_METHOD("get_motor_torque"), &Box2DRevoluteJoint::get_motor_torque);

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "limit_enabled"), "set_limit_enabled", "is_limit_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "upper_limit"), "set_upper_limit", "get_upper_limit");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "lower_limit"), "set_lower_limit", "get_lower_limit");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "motor_enabled"), "set_motor_enabled", "is_motor_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "motor_speed"), "set_motor_speed", "get_motor_speed");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_motor_torque"), "set_max_motor_torque", "get_max_motor_torque");
	ADD_GROUP("Anchors", "");
	// TODO rename this property to configurable_anchors and invert the bool
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_default_anchors"), "set_editor_use_default_anchors", "get_editor_use_default_anchors");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "anchor_a"), "set_anchor_a", "get_anchor_a");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "anchor_b"), "set_anchor_b", "get_anchor_b");
}

void Box2DRevoluteJoint::init_b2JointDef(const b2Vec2 &p_joint_pos) {
	jointDef.Initialize(jointDef.bodyA, jointDef.bodyB, p_joint_pos);

	// Make the jointDef use the configured anchors
	jointDef.localAnchorA = jointDef.bodyA->GetLocalPoint(gd_to_b2(get_global_transform().xform(anchor_a)));
	jointDef.localAnchorB = jointDef.bodyB->GetLocalPoint(gd_to_b2(get_global_transform().xform(anchor_b)));
}

void Box2DRevoluteJoint::debug_draw(RID p_to_rid, Color p_color) {
	b2RevoluteJoint *j = static_cast<b2RevoluteJoint *>(get_b2Joint());
	b2Vec2 anchorA = gd_to_b2(get_global_position());
	b2Vec2 anchorB = anchorA;
	if (jointDef.bodyA) {
		anchorA = jointDef.bodyA->GetWorldPoint(jointDef.localAnchorA);
	}
	if (jointDef.bodyB) {
		anchorB = jointDef.bodyB->GetWorldPoint(jointDef.localAnchorB);
	}

	b2Vec2 posA = anchorA;
	b2Vec2 posB = anchorB;
	if (jointDef.bodyA) {
		posA = jointDef.bodyA->GetWorldCenter();
	}
	if (jointDef.bodyB) {
		posB = jointDef.bodyB->GetWorldCenter();
	}

	Point2 p1 = to_local(b2_to_gd(anchorA));
	Point2 p2 = to_local(b2_to_gd(anchorB));
	Point2 x1 = to_local(b2_to_gd(posA));
	Point2 x2 = to_local(b2_to_gd(posB));

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
			float torqueUsage = get_motor_torque() / get_max_motor_torque();
			arclen *= torqueUsage;
			c = Color(1.0, 1.0, 0.0, 0.5);
			draw_arc(p1, 9, a, a + arclen, MAX(static_cast<int>(5.0f * arclen), 2), c, 2.0f);
		}
	}

	Color c(p_color);
	c.set_hsv(c.get_h(), c.get_s() * 0.5f, c.get_v(), c.a * 0.2f);
	draw_line(x1, p1, c, 1.0f);
	draw_line(x2, p2, c, 1.0f);
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

void Box2DRevoluteJoint::set_editor_use_default_anchors(bool p_update) {
	editor_use_default_anchors = p_update;
	if (editor_use_default_anchors)
		reset_joint_anchors();
}

bool Box2DRevoluteJoint::get_editor_use_default_anchors() const {
	return editor_use_default_anchors;
}

void Box2DRevoluteJoint::set_anchor_a(const Vector2 &p_anchor) {
	anchor_a = p_anchor;

	recreate_joint();
}

Vector2 Box2DRevoluteJoint::get_anchor_a() const {
	if (jointDef.bodyA)
		return to_local(b2_to_gd(jointDef.bodyA->GetWorldPoint(jointDef.localAnchorA)));
	return anchor_a;
}

void Box2DRevoluteJoint::set_anchor_b(const Vector2 &p_anchor) {
	anchor_b = p_anchor;

	recreate_joint();
}

Vector2 Box2DRevoluteJoint::get_anchor_b() const {
	if (jointDef.bodyB)
		return to_local(b2_to_gd(jointDef.bodyB->GetWorldPoint(jointDef.localAnchorB)));
	return anchor_b;
}

void Box2DRevoluteJoint::reset_joint_anchors() {
	ERR_FAIL_COND_MSG(!is_inside_tree(), "Can't reinitialize a joint outside of the SceneTree. It requires relative locations of bodies to initialize.");
	anchor_a = Vector2();
	anchor_b = Vector2();
	recreate_joint();
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

real_t Box2DRevoluteJoint::get_motor_torque() const {
	ERR_FAIL_COND_V_MSG(!get_b2Joint(), real_t(), "b2Joint is null.");
	return static_cast<b2RevoluteJoint *>(get_b2Joint())->GetMotorTorque(1.0f / get_physics_process_delta_time());
}

Box2DRevoluteJoint::Box2DRevoluteJoint() :
		Box2DJoint(&jointDef) {

	if (!Engine::get_singleton()->is_editor_hint()) {
		editor_use_default_anchors = false;
	}
};

void Box2DPrismaticJoint::on_editor_transforms_changed() {
	if (editor_use_default_anchors) {
		// We need set reset anchors and reinit the jointDef
		if (is_inside_tree()) {
			reset_joint_anchors();
			_change_notify();
		}
	} else {
		// Some relative coordinate has changed
		// We want to move our anchors to keep their relative location to each body the same
		// This leaves the jointDef anchors unchanged
		anchor_a = to_local(b2_to_gd(jointDef.bodyA->GetWorldPoint(jointDef.localAnchorA)));
		anchor_b = to_local(b2_to_gd(jointDef.bodyB->GetWorldPoint(jointDef.localAnchorB)));
		_change_notify();
	}
}

void Box2DPrismaticJoint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_reference_angle"), &Box2DPrismaticJoint::get_reference_angle);
	ClassDB::bind_method(D_METHOD("get_joint_translation"), &Box2DPrismaticJoint::get_joint_translation);
	ClassDB::bind_method(D_METHOD("get_joint_speed"), &Box2DPrismaticJoint::get_joint_speed);

	ClassDB::bind_method(D_METHOD("set_local_axis", "axis"), &Box2DPrismaticJoint::set_local_axis);
	ClassDB::bind_method(D_METHOD("get_local_axis"), &Box2DPrismaticJoint::get_local_axis);

	ClassDB::bind_method(D_METHOD("set_editor_use_default_anchors", "default"), &Box2DPrismaticJoint::set_editor_use_default_anchors);
	ClassDB::bind_method(D_METHOD("get_editor_use_default_anchors"), &Box2DPrismaticJoint::get_editor_use_default_anchors);
	ClassDB::bind_method(D_METHOD("set_anchor_a", "anchor_a"), &Box2DPrismaticJoint::set_anchor_a);
	ClassDB::bind_method(D_METHOD("get_anchor_a"), &Box2DPrismaticJoint::get_anchor_a);
	ClassDB::bind_method(D_METHOD("set_anchor_b", "anchor_b"), &Box2DPrismaticJoint::set_anchor_b);
	ClassDB::bind_method(D_METHOD("get_anchor_b"), &Box2DPrismaticJoint::get_anchor_b);
	ClassDB::bind_method(D_METHOD("reset_joint_anchors"), &Box2DPrismaticJoint::reset_joint_anchors);

	ClassDB::bind_method(D_METHOD("set_limit_enabled", "limit_enabled"), &Box2DPrismaticJoint::set_limit_enabled);
	ClassDB::bind_method(D_METHOD("is_limit_enabled"), &Box2DPrismaticJoint::is_limit_enabled);
	ClassDB::bind_method(D_METHOD("set_upper_limit", "upper_limit"), &Box2DPrismaticJoint::set_upper_limit);
	ClassDB::bind_method(D_METHOD("get_upper_limit"), &Box2DPrismaticJoint::get_upper_limit);
	ClassDB::bind_method(D_METHOD("set_lower_limit", "lower_limit"), &Box2DPrismaticJoint::set_lower_limit);
	ClassDB::bind_method(D_METHOD("get_lower_limit"), &Box2DPrismaticJoint::get_lower_limit);

	ClassDB::bind_method(D_METHOD("set_limits", "lower", "upper"), &Box2DPrismaticJoint::set_limits);

	ClassDB::bind_method(D_METHOD("set_motor_enabled", "motor_enabled"), &Box2DPrismaticJoint::set_motor_enabled);
	ClassDB::bind_method(D_METHOD("is_motor_enabled"), &Box2DPrismaticJoint::is_motor_enabled);
	ClassDB::bind_method(D_METHOD("set_motor_speed", "motor_speed"), &Box2DPrismaticJoint::set_motor_speed);
	ClassDB::bind_method(D_METHOD("get_motor_speed"), &Box2DPrismaticJoint::get_motor_speed);
	ClassDB::bind_method(D_METHOD("set_max_motor_force", "max_motor_force"), &Box2DPrismaticJoint::set_max_motor_force);
	ClassDB::bind_method(D_METHOD("get_max_motor_force"), &Box2DPrismaticJoint::get_max_motor_force);
	ClassDB::bind_method(D_METHOD("get_motor_force"), &Box2DPrismaticJoint::get_motor_force);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "local_axis"), "set_local_axis", "get_local_axis");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "limit_enabled"), "set_limit_enabled", "is_limit_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "upper_limit"), "set_upper_limit", "get_upper_limit");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "lower_limit"), "set_lower_limit", "get_lower_limit");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "motor_enabled"), "set_motor_enabled", "is_motor_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "motor_speed"), "set_motor_speed", "get_motor_speed");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_motor_force"), "set_max_motor_force", "get_max_motor_force");
	ADD_GROUP("Anchors", "");
	// TODO rename this property to configurable_anchors and invert the bool
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_default_anchors"), "set_editor_use_default_anchors", "get_editor_use_default_anchors");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "anchor_a"), "set_anchor_a", "get_anchor_a");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "anchor_b"), "set_anchor_b", "get_anchor_b");
}

void Box2DPrismaticJoint::init_b2JointDef(const b2Vec2 &p_joint_pos) {
	Vector2 global_axis = get_global_transform().basis_xform(local_axis);

	axis_body_ref_angle = jointDef.bodyA->GetAngle();

	jointDef.Initialize(jointDef.bodyA, jointDef.bodyB, p_joint_pos, b2Vec2(global_axis.x, global_axis.y));

	// Make the jointDef use the configured anchors
	jointDef.localAnchorA = jointDef.bodyA->GetLocalPoint(gd_to_b2(get_global_transform().xform(anchor_a)));
	jointDef.localAnchorB = jointDef.bodyB->GetLocalPoint(gd_to_b2(get_global_transform().xform(anchor_b)));
}

void Box2DPrismaticJoint::debug_draw(RID p_to_rid, Color p_color) {
	b2PrismaticJoint *j = static_cast<b2PrismaticJoint *>(get_b2Joint());
	b2Vec2 anchorA = gd_to_b2(get_global_position());
	b2Vec2 anchorB = anchorA;
	if (jointDef.bodyA) {
		anchorA = jointDef.bodyA->GetWorldPoint(jointDef.localAnchorA);
	}
	if (jointDef.bodyB) {
		anchorB = jointDef.bodyB->GetWorldPoint(jointDef.localAnchorB);
	}

	b2Vec2 posA = anchorA;
	b2Vec2 posB = anchorB;
	if (jointDef.bodyA) {
		posA = jointDef.bodyA->GetWorldCenter();
	}
	if (jointDef.bodyB) {
		posB = jointDef.bodyB->GetWorldCenter();
	}

	Point2 p1 = to_local(b2_to_gd(anchorA));
	Point2 p2 = to_local(b2_to_gd(anchorB));
	Point2 x1 = to_local(b2_to_gd(posA));
	Point2 x2 = to_local(b2_to_gd(posB));

	if (jointDef.bodyA)
		draw_set_transform(p1, jointDef.bodyA->GetUserData().owner->get_global_rotation() - axis_body_ref_angle, Vector2(1, 1));
	else
		draw_set_transform(p1, -axis_body_ref_angle, Vector2(1, 1));

	draw_rect(Rect2(-Vector2(5, 5), Vector2(10, 10)), p_color, false, 2.0f);
	draw_rect(Rect2(-Vector2(1, 1), Vector2(2, 2)), p_color);

	Vector2 axis = get_local_axis();

	if (jointDef.enableLimit) {
		Vector2 start = axis * get_lower_limit();
		Vector2 end = axis * get_upper_limit();
		draw_line(start, end, p_color, 1.0f);
		Vector2 tan = axis.rotated(Math_PI * 0.5f) * 5.0f;
		draw_line(start + tan, start - tan, p_color, 1.0f);
		draw_line(end + tan, end - tan, p_color, 1.0f);
	} else {
		draw_line(Vector2(0, 0), axis * 25.0, p_color);
	}
	if (jointDef.enableMotor) {
		float a = jointDef.referenceAngle;
		Color c = jointDef.motorSpeed > 0 ? Color(0.0, 1.0, 0.0, 0.5) : Color(1.0, 0.0, 0.0, 0.5);

		Vector2 start_p = axis * get_joint_translation();
		Vector2 tan = axis.rotated(Math_PI * 0.5f);

		draw_line(start_p, start_p + axis * get_joint_speed(), c, 1.5f);
		if (j) {
			float forceUsage = get_motor_force() / get_max_motor_force();
			c = Color(1.0, 1.0, 0.0, 0.5);
			draw_line(start_p + tan, start_p + tan + axis * get_joint_speed(), c, 1.5f);
		}
	}

	draw_set_transform(Vector2(0, 0), 0, Vector2(1, 1));
	Color c(p_color);
	c.set_hsv(c.get_h(), c.get_s() * 0.5f, c.get_v(), c.a * 0.2f);
	draw_line(x1, p1, c, 1.0f);
	draw_line(x2, p2, c, 1.0f);
}

real_t Box2DPrismaticJoint::get_reference_angle() const {
	ERR_FAIL_COND_V_MSG(!get_b2Joint(), real_t(), "b2Joint is null.");
	return static_cast<b2PrismaticJoint *>(get_b2Joint())->GetReferenceAngle();
}

real_t Box2DPrismaticJoint::get_joint_translation() const {
	ERR_FAIL_COND_V_MSG(!get_b2Joint(), real_t(), "b2Joint is null.");
	return static_cast<b2PrismaticJoint *>(get_b2Joint())->GetJointTranslation();
}

real_t Box2DPrismaticJoint::get_joint_speed() const {
	ERR_FAIL_COND_V_MSG(!get_b2Joint(), real_t(), "b2Joint is null.");
	return static_cast<b2PrismaticJoint *>(get_b2Joint())->GetJointSpeed();
}

void Box2DPrismaticJoint::set_editor_use_default_anchors(bool p_default) {
	editor_use_default_anchors = p_default;
	if (editor_use_default_anchors)
		reset_joint_anchors();
}

bool Box2DPrismaticJoint::get_editor_use_default_anchors() const {
	return editor_use_default_anchors;
}

void Box2DPrismaticJoint::set_anchor_a(const Vector2 &p_anchor) {
	anchor_a = p_anchor;

	recreate_joint();
}

Vector2 Box2DPrismaticJoint::get_anchor_a() const {
	if (jointDef.bodyA)
		return to_local(b2_to_gd(jointDef.bodyA->GetWorldPoint(jointDef.localAnchorA)));
	return anchor_a;
}

void Box2DPrismaticJoint::set_anchor_b(const Vector2 &p_anchor) {
	anchor_b = p_anchor;

	recreate_joint();
}

Vector2 Box2DPrismaticJoint::get_anchor_b() const {
	if (jointDef.bodyB)
		return to_local(b2_to_gd(jointDef.bodyB->GetWorldPoint(jointDef.localAnchorB)));
	return anchor_b;
}

void Box2DPrismaticJoint::reset_joint_anchors() {
	ERR_FAIL_COND_MSG(!is_inside_tree(), "Can't reinitialize a joint outside of the SceneTree. It requires relative locations of bodies to initialize.");
	anchor_a = Vector2();
	anchor_b = Vector2();
	recreate_joint();
}

void Box2DPrismaticJoint::set_local_axis(Vector2 p_axis) {
	local_axis = p_axis.normalized();
	if (is_inside_tree())
		recreate_joint();
}

Vector2 Box2DPrismaticJoint::get_local_axis() const {
	return local_axis;
}

void Box2DPrismaticJoint::set_limit_enabled(bool p_enabled) {
	if (get_b2Joint())
		static_cast<b2PrismaticJoint *>(get_b2Joint())->EnableLimit(p_enabled);
	jointDef.enableLimit = p_enabled;
}

bool Box2DPrismaticJoint::is_limit_enabled() const {
	return jointDef.enableLimit;
}

void Box2DPrismaticJoint::set_upper_limit(real_t p_distance) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	float distance = p_distance * factor;
	if (get_b2Joint())
		static_cast<b2PrismaticJoint *>(get_b2Joint())->SetLimits(jointDef.lowerTranslation, distance);
	jointDef.upperTranslation = distance;
}

real_t Box2DPrismaticJoint::get_upper_limit() const {
	const float factor = static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	return jointDef.upperTranslation * factor;
}

void Box2DPrismaticJoint::set_lower_limit(real_t p_distance) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	float distance = p_distance * factor;
	if (get_b2Joint())
		static_cast<b2PrismaticJoint *>(get_b2Joint())->SetLimits(distance, jointDef.upperTranslation);
	jointDef.lowerTranslation = distance;
}

real_t Box2DPrismaticJoint::get_lower_limit() const {
	const float factor = static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	return jointDef.lowerTranslation * factor;
}

void Box2DPrismaticJoint::set_limits(real_t p_lower, real_t p_upper) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	float lower = p_lower * factor;
	float upper = p_upper * factor;
	if (get_b2Joint())
		static_cast<b2PrismaticJoint *>(get_b2Joint())->SetLimits(lower, upper);
	jointDef.lowerTranslation = lower;
	jointDef.upperTranslation = upper;
}

void Box2DPrismaticJoint::set_motor_enabled(bool p_enabled) {
	if (get_b2Joint())
		static_cast<b2PrismaticJoint *>(get_b2Joint())->EnableMotor(p_enabled);
	jointDef.enableMotor = p_enabled;
}

bool Box2DPrismaticJoint::is_motor_enabled() const {
	return jointDef.enableMotor;
}

void Box2DPrismaticJoint::set_motor_speed(real_t p_speed) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	float speed = p_speed * factor;
	if (get_b2Joint())
		static_cast<b2PrismaticJoint *>(get_b2Joint())->SetMotorSpeed(speed);
	jointDef.motorSpeed = speed;
}

real_t Box2DPrismaticJoint::get_motor_speed() const {
	const float factor = static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	return jointDef.motorSpeed * factor;
}

void Box2DPrismaticJoint::set_max_motor_force(real_t p_force) {
	if (get_b2Joint())
		static_cast<b2PrismaticJoint *>(get_b2Joint())->SetMaxMotorForce(p_force);
	jointDef.maxMotorForce = p_force;
}

real_t Box2DPrismaticJoint::get_max_motor_force() const {
	return jointDef.maxMotorForce;
}

real_t Box2DPrismaticJoint::get_motor_force() const {
	ERR_FAIL_COND_V_MSG(!get_b2Joint(), real_t(), "b2Joint is null.");
	return static_cast<b2PrismaticJoint *>(get_b2Joint())->GetMotorForce(1.0f / get_physics_process_delta_time());
}

Box2DPrismaticJoint::Box2DPrismaticJoint() :
		Box2DJoint(&jointDef) {

	if (!Engine::get_singleton()->is_editor_hint()) {
		editor_use_default_anchors = false;
	}
}

void Box2DDistanceJoint::on_editor_transforms_changed() {
	// Some relative coordinate has changed
	// We want to move our anchors to keep their relative location to each body the same
	// This leaves the jointDef anchors unchanged
	if (jointDef.bodyA && jointDef.bodyB) {
		anchor_a = to_local(b2_to_gd(jointDef.bodyA->GetWorldPoint(jointDef.localAnchorA)));
		anchor_b = to_local(b2_to_gd(jointDef.bodyB->GetWorldPoint(jointDef.localAnchorB)));
		if (editor_use_default_rest_length)
			reset_rest_length();
		_change_notify();
	}
}

void Box2DDistanceJoint::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_editor_use_default_rest_length", "default"), &Box2DDistanceJoint::set_editor_use_default_rest_length);
	ClassDB::bind_method(D_METHOD("get_editor_use_default_rest_length"), &Box2DDistanceJoint::get_editor_use_default_rest_length);

	ClassDB::bind_method(D_METHOD("set_anchor_a", "anchor_a"), &Box2DDistanceJoint::set_anchor_a);
	ClassDB::bind_method(D_METHOD("get_anchor_a"), &Box2DDistanceJoint::get_anchor_a);
	ClassDB::bind_method(D_METHOD("set_anchor_b", "anchor_b"), &Box2DDistanceJoint::set_anchor_b);
	ClassDB::bind_method(D_METHOD("get_anchor_b"), &Box2DDistanceJoint::get_anchor_b);
	ClassDB::bind_method(D_METHOD("set_rest_length", "rest_length"), &Box2DDistanceJoint::set_rest_length);
	ClassDB::bind_method(D_METHOD("get_rest_length"), &Box2DDistanceJoint::get_rest_length);
	ClassDB::bind_method(D_METHOD("reset_rest_length"), &Box2DDistanceJoint::reset_rest_length);
	ClassDB::bind_method(D_METHOD("set_min_length", "min_length"), &Box2DDistanceJoint::set_min_length);
	ClassDB::bind_method(D_METHOD("get_min_length"), &Box2DDistanceJoint::get_min_length);
	ClassDB::bind_method(D_METHOD("set_max_length", "max_length"), &Box2DDistanceJoint::set_max_length);
	ClassDB::bind_method(D_METHOD("get_max_length"), &Box2DDistanceJoint::get_max_length);

	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &Box2DDistanceJoint::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &Box2DDistanceJoint::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_damping", "damping"), &Box2DDistanceJoint::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &Box2DDistanceJoint::get_damping);

	ClassDB::bind_method(D_METHOD("get_current_length"), &Box2DDistanceJoint::get_current_length);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "anchor_a"), "set_anchor_a", "get_anchor_a");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "anchor_b"), "set_anchor_b", "get_anchor_b");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_default_rest_length"), "set_editor_use_default_rest_length", "get_editor_use_default_rest_length");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "rest_length"), "set_rest_length", "get_rest_length");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "min_length"), "set_min_length", "get_min_length");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_length"), "set_max_length", "get_max_length");
	ADD_GROUP("Spring Tuning", "");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "stiffness"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "damping"), "set_damping", "get_damping");
}

void Box2DDistanceJoint::init_b2JointDef(const b2Vec2 &) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));

	b2Vec2 a = gd_to_b2(get_global_transform().xform(anchor_a));
	b2Vec2 b = gd_to_b2(get_global_transform().xform(anchor_b));

	jointDef.Initialize(jointDef.bodyA, jointDef.bodyB, a, b);
	jointDef.length = rest_length * factor;
	jointDef.minLength = min_length * factor;
	jointDef.maxLength = max_length * factor;
}

void Box2DDistanceJoint::debug_draw(RID p_to_rid, Color p_color) {
	b2PrismaticJoint *j = static_cast<b2PrismaticJoint *>(get_b2Joint());
	b2Vec2 anchorA = gd_to_b2(get_global_position());
	b2Vec2 anchorB = anchorA;
	if (jointDef.bodyA) {
		anchorA = jointDef.bodyA->GetWorldPoint(jointDef.localAnchorA);
	}
	if (jointDef.bodyB) {
		anchorB = jointDef.bodyB->GetWorldPoint(jointDef.localAnchorB);
	}

	b2Vec2 posA = anchorA;
	b2Vec2 posB = anchorB;
	if (jointDef.bodyA) {
		posA = jointDef.bodyA->GetWorldCenter();
	}
	if (jointDef.bodyB) {
		posB = jointDef.bodyB->GetWorldCenter();
	}

	Point2 p1 = to_local(b2_to_gd(anchorA));
	Point2 p2 = to_local(b2_to_gd(anchorB));
	Point2 x1 = to_local(b2_to_gd(posA));
	Point2 x2 = to_local(b2_to_gd(posB));

	draw_rect(Rect2(p1 - Point2(1, 1), Size2(2, 2)), p_color, true);
	draw_rect(Rect2(p2 - Point2(1, 1), Size2(2, 2)), p_color, true);

	Vector2 axis = (p2 - p1).normalized();
	Vector2 tan = axis.rotated(Math_PI * 0.5f);

	Vector2 p_min = p1 + axis * get_min_length();
	Vector2 p_max = p1 + axis * get_max_length();
	Vector2 p_rest = p1 + axis * get_rest_length();

	// TODO draw spring lines

	draw_line(p1, p_min, p_color);
	draw_line(p_min, p_max, p_color, 1.25f);

	draw_line(p_rest + tan * 5.0f, p_rest - tan * 5.0f, p_color, 1.25f);

	Color c(p_color);
	c.set_hsv(c.get_h(), c.get_s() * 0.5f, c.get_v(), c.a * 0.2f);
	draw_line(x1, p1, c, 1.0f);
	draw_line(x2, p2, c, 1.0f);
}

void Box2DDistanceJoint::set_editor_use_default_rest_length(bool p_default) {
	editor_use_default_rest_length = p_default;
	if (editor_use_default_rest_length) {
		reset_rest_length();
	}
}

bool Box2DDistanceJoint::get_editor_use_default_rest_length() const {
	return editor_use_default_rest_length;
}

void Box2DDistanceJoint::set_anchor_a(const Vector2 &p_anchor) {
	anchor_a = p_anchor;
	recreate_joint();
}

Vector2 Box2DDistanceJoint::get_anchor_a() const {
	if (jointDef.bodyA)
		return to_local(b2_to_gd(jointDef.bodyA->GetWorldPoint(jointDef.localAnchorA)));
	return anchor_a;
}

void Box2DDistanceJoint::set_anchor_b(const Vector2 &p_anchor) {
	anchor_b = p_anchor;
	recreate_joint();
}

Vector2 Box2DDistanceJoint::get_anchor_b() const {
	if (jointDef.bodyB)
		return to_local(b2_to_gd(jointDef.bodyB->GetWorldPoint(jointDef.localAnchorB)));
	return anchor_b;
}

void Box2DDistanceJoint::set_rest_length(real_t p_length) {
	rest_length = p_length;
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	float length = p_length * factor;
	if (get_b2Joint())
		static_cast<b2DistanceJoint *>(get_b2Joint())->SetLength(length);
	jointDef.length = length;

	if (rest_length < min_length) {
		set_min_length(rest_length);
	}
	if (rest_length > max_length) {
		set_max_length(rest_length);
	}
}

real_t Box2DDistanceJoint::get_rest_length() const {
	return rest_length;
}

void Box2DDistanceJoint::reset_rest_length() {
	real_t length = anchor_a.distance_to(anchor_b);
	set_rest_length(length);
	if (length < get_min_length())
		set_min_length(length);
	if (length > get_max_length())
		set_max_length(length);
}

void Box2DDistanceJoint::set_min_length(real_t p_length) {
	min_length = p_length;

	if (rest_length < min_length) {
		min_length = rest_length;
	}

	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	float length = p_length * factor;
	if (get_b2Joint())
		static_cast<b2DistanceJoint *>(get_b2Joint())->SetMinLength(length);
	jointDef.length = length;
}

real_t Box2DDistanceJoint::get_min_length() const {
	return min_length;
}

void Box2DDistanceJoint::set_max_length(real_t p_length) {
	max_length = p_length;

	if (rest_length > max_length) {
		max_length = rest_length;
	}

	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	float length = p_length * factor;
	if (get_b2Joint())
		static_cast<b2DistanceJoint *>(get_b2Joint())->SetMaxLength(length);
	jointDef.length = length;
}

real_t Box2DDistanceJoint::get_max_length() const {
	return max_length;
}

void Box2DDistanceJoint::set_stiffness(real_t p_stiffness) {
	// TODO fix unit conversion mismatch (weldjoint too)
	if (get_b2Joint())
		static_cast<b2DistanceJoint *>(get_b2Joint())->SetStiffness(p_stiffness);
	jointDef.stiffness = p_stiffness;
}

real_t Box2DDistanceJoint::get_stiffness() const {
	return jointDef.stiffness;
}

void Box2DDistanceJoint::set_damping(real_t p_damping) {
	if (get_b2Joint())
		static_cast<b2DistanceJoint *>(get_b2Joint())->SetDamping(p_damping);
	jointDef.damping = p_damping;
}

real_t Box2DDistanceJoint::get_damping() const {
	return jointDef.damping;
}

real_t Box2DDistanceJoint::get_current_length() const {
	ERR_FAIL_COND_V_MSG(!get_b2Joint(), real_t(), "b2Joint is null.");
	const float factor = static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	return static_cast<b2DistanceJoint *>(get_b2Joint())->GetCurrentLength();
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
	//b2WeldJoint *j = static_cast<b2WeldJoint *>(get_b2Joint());
	b2Vec2 anchorA = gd_to_b2(get_global_position());
	b2Vec2 anchorB = anchorA;
	if (jointDef.bodyA) {
		anchorA = jointDef.bodyA->GetWorldPoint(jointDef.localAnchorA);
	}
	if (jointDef.bodyB) {
		anchorB = jointDef.bodyB->GetWorldPoint(jointDef.localAnchorB);
	}

	b2Vec2 posA = anchorA;
	b2Vec2 posB = anchorB;
	if (jointDef.bodyA) {
		posA = jointDef.bodyA->GetWorldCenter();
	}
	if (jointDef.bodyB) {
		posB = jointDef.bodyB->GetWorldCenter();
	}

	Point2 p1 = to_local(b2_to_gd(anchorA));
	Point2 p2 = to_local(b2_to_gd(anchorB));
	Point2 x1 = to_local(b2_to_gd(posA));
	Point2 x2 = to_local(b2_to_gd(posB));

	draw_line(p1 + Point2(-5, 0), p1 + Point2(+5, 0), p_color, 2.0f);
	draw_line(p1 + Point2(0, -5), p1 + Point2(0, +5), p_color, 2.0f);

	Color c(p_color);
	c.set_hsv(c.get_h(), c.get_s() * 0.5f, c.get_v(), c.a * 0.2f);
	draw_line(x1, p1, c, 1.0f);
	draw_line(x2, p2, c, 1.0f);
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
