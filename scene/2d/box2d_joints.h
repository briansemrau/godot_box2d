#ifndef BOX2D_JOINTS_H
#define BOX2D_JOINTS_H

#include <core/io/resource.h>
#include <core/object/object.h>
#include <core/object/ref_counted.h>
#include <scene/2d/node_2d.h>

#include <box2d/b2_distance_joint.h>
#include <box2d/b2_friction_joint.h>
#include <box2d/b2_gear_joint.h>
#include <box2d/b2_joint.h>
#include <box2d/b2_motor_joint.h>
#include <box2d/b2_mouse_joint.h>
#include <box2d/b2_prismatic_joint.h>
#include <box2d/b2_pulley_joint.h>
#include <box2d/b2_revolute_joint.h>
#include <box2d/b2_rope.h>
#include <box2d/b2_weld_joint.h>
#include <box2d/b2_wheel_joint.h>

#include "../../editor/box2d_joint_editor_plugin.h"
#include "../../util/box2d_types_converter.h"
#include "box2d_physics_body.h"

/**
* @author Brian Semrau
*/

class Box2DWorld;
class Box2DPhysicsBody;

class Box2DJoint : public Node2D {
	GDCLASS(Box2DJoint, Node2D);

	friend class Box2DWorld;
	friend class Box2DPhysicsBody;

	friend class Box2DJointEditor; // this is questionable TODO remove probably

	b2JointDef *jointDef = NULL;
	b2Joint *joint = NULL;

	Box2DWorld *world_node = NULL;

	NodePath a;
	NodePath b;

	ObjectID bodyA_cache{}; // TODO rename to objA_cache when update_joint_bodies is made virtual
	ObjectID bodyB_cache{};

	bool broken = false;
	bool breaking_enabled = false;
	bool free_on_break = false; // TODO This feature may not be necessary. Perhaps this should be handled by the user in an "on_broken" signal.
	real_t max_force = 0.0f;
	real_t max_torque = 0.0f;

	void on_b2Joint_destroyed();

	bool create_b2Joint();
	bool destroy_b2Joint();

	void on_parent_created(Node *p_parent);
	void on_node_predelete(Box2DPhysicsBody *node);
	virtual void on_editor_transforms_changed();

	// Rescans the nodepaths to find b2Bodies and create our b2joint
	void update_joint_bodies(); // TODO make virtual. Gear joint links joints, not bodies. Rename "update_joint_linkages/connections/nodepaths" or similar

	void _node_a_tree_entered();
	void _node_b_tree_entered();

protected:
	Box2DJointEditor::AnchorMode editor_anchor_mode = Box2DJointEditor::AnchorMode::MODE_ANCHORS_LOCAL;

	// Destroys and recreates the b2Joint, if valid.
	// Useful for modifying const b2 parameters, such as anchors. In these cases, set p_soft_reset to true.
	// `p_soft_reset` indicates whether initial-configuration properties should not be reset, such as relative body angles.
	void recreate_joint(bool p_soft_reset);

	b2Joint *get_b2Joint() { return joint; }
	b2Joint *get_b2Joint() const { return joint; }

	b2Vec2 get_b2_pos() const; // TODO verify that this is correct. Maybe modify to match changes in PR #31

	void _notification(int p_what);
	static void _bind_methods();

	// Joint node local-space anchor points. These sync with jointDef's body-local anchor points.
	// Allows configuring anchor points to be in different world positions when added to the scene.
	// Modifying these is generally not recommended unless required for initializing a broken joint that starts separated.
	Vector2 anchor_a = Vector2();
	Vector2 anchor_b = Vector2();

	// Allows all joint types to perform their custom initializations.
	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos, bool p_soft_reset) = 0;

	virtual void debug_draw(RID p_to_rid, Color p_color) = 0;

	// TODO maybe this should not be pure virtual?
	virtual Vector2 get_body_local_anchor_a() const = 0;
	virtual Vector2 get_body_local_anchor_b() const = 0;

	// These functions have to be proxied in all subclasses that bind them.
	//     ClassDB binds to the function's class, irrespective of class scope used when binding.
	//     That is, bind_method(..., &Box2DWeldJoint::set_anchor_a); would bind to the Godot class Box2DJoint.
	void set_anchor_a(const Vector2 &p_anchor);
	Vector2 get_anchor_a() const;

	void set_anchor_b(const Vector2 &p_anchor);
	Vector2 get_anchor_b() const;

	void reset_joint_anchors();

public:
	virtual PackedStringArray get_configuration_warnings() const override;

	void set_nodepath_a(const NodePath &p_node_a);
	NodePath get_nodepath_a() const;

	void set_nodepath_b(const NodePath &p_node_b);
	NodePath get_nodepath_b() const;

	void set_collide_connected(bool p_collide);
	bool get_collide_connected() const;

	void set_broken(bool p_broken);
	bool is_broken() const;

	void set_breaking_enabled(bool p_enabled);
	bool is_breaking_enabled() const;

	void set_free_on_break(bool p_should_free);
	bool get_free_on_break() const;

	void set_max_force(real_t p_max_force);
	real_t get_max_force() const;

	void set_max_torque(real_t p_max_torque);
	real_t get_max_torque() const;

	Vector2 get_reaction_force() const;
	real_t get_reaction_torque() const;

	bool is_enabled() const;

	bool is_valid() const;

	Box2DJoint();
	~Box2DJoint();

protected:
	Box2DJoint(b2JointDef *p_def) :
			Box2DJoint() {
		jointDef = p_def;
	}
};

class Box2DRevoluteJoint : public Box2DJoint {
	GDCLASS(Box2DRevoluteJoint, Box2DJoint);

	b2RevoluteJointDef jointDef;

protected:
	static void _bind_methods();

	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos, bool p_soft_reset) override;

	virtual void debug_draw(RID p_to_rid, Color p_color) override;

public:
	Vector2 get_body_local_anchor_a() const override { return b2_to_gd(jointDef.localAnchorA); }
	Vector2 get_body_local_anchor_b() const override { return b2_to_gd(jointDef.localAnchorB); }

	// Proxy to Box2DJoint for correct ClassDB binding
	void set_anchor_a(const Vector2 &p_anchor) { Box2DJoint::set_anchor_a(p_anchor); }
	Vector2 get_anchor_a() const { return Box2DJoint::get_anchor_a(); }
	void set_anchor_b(const Vector2 &p_anchor) { Box2DJoint::set_anchor_b(p_anchor); }
	Vector2 get_anchor_b() const { return Box2DJoint::get_anchor_b(); }
	void reset_joint_anchors() { Box2DJoint::reset_joint_anchors(); }

	// TODO all relevant joints should have a property for reference_angle for save/load retention
	//      Somehow it needs to overwrite the initial hard init, perhaps with a custom_ref_angle flag?
	// TODO
	// Changes the reference angle between bodies.
	//void set_reference_angle(real_t p_angle);
	real_t get_reference_angle() const;

	real_t get_joint_angle() const;
	real_t get_joint_speed() const;

	void set_limit_enabled(bool p_enabled);
	bool is_limit_enabled() const;

	void set_upper_limit(real_t p_angle);
	real_t get_upper_limit() const;
	void set_lower_limit(real_t p_angle);
	real_t get_lower_limit() const;

	void set_limits(real_t p_lower, real_t p_upper);

	void set_motor_enabled(bool p_enabled);
	bool is_motor_enabled() const;

	void set_motor_speed(real_t p_speed);
	real_t get_motor_speed() const;

	void set_max_motor_torque(real_t p_torque);
	real_t get_max_motor_torque() const;

	real_t get_motor_torque() const;

	Box2DRevoluteJoint();
};

class Box2DPrismaticJoint : public Box2DJoint {
	GDCLASS(Box2DPrismaticJoint, Box2DJoint);

	b2PrismaticJointDef jointDef;

	// The relative angle between node A and this joint node. Used for debug drawing.
	float axis_body_ref_angle = 0.0f;
	// The prismatic axis to initialize the joint with. In joint-node-local coordinates.
	Vector2 local_axis = Vector2(25.0f, 0);

protected:
	static void _bind_methods();

	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos, bool p_soft_reset) override;

	virtual void debug_draw(RID p_to_rid, Color p_color) override;

public:
	Vector2 get_body_local_anchor_a() const override { return b2_to_gd(jointDef.localAnchorA); }
	Vector2 get_body_local_anchor_b() const override { return b2_to_gd(jointDef.localAnchorB); }

	// Proxy to Box2DJoint for correct ClassDB binding
	void set_anchor_a(const Vector2 &p_anchor) { Box2DJoint::set_anchor_a(p_anchor); }
	Vector2 get_anchor_a() const { return Box2DJoint::get_anchor_a(); }
	void set_anchor_b(const Vector2 &p_anchor) { Box2DJoint::set_anchor_b(p_anchor); }
	Vector2 get_anchor_b() const { return Box2DJoint::get_anchor_b(); }
	void reset_joint_anchors() { Box2DJoint::reset_joint_anchors(); }

	real_t get_reference_angle() const;
	real_t get_joint_translation() const;
	real_t get_joint_speed() const;

	// Changing axis length has no effect
	void set_local_axis(const Vector2 &p_axis);
	Vector2 get_local_axis() const;

	void set_local_axis_angle_degrees(float p_degrees);
	float get_local_axis_angle_degrees() const;

	void set_limit_enabled(bool p_enabled);
	bool is_limit_enabled() const;

	void set_upper_limit(real_t p_distance);
	real_t get_upper_limit() const;
	void set_lower_limit(real_t p_distance);
	real_t get_lower_limit() const;

	void set_limits(real_t p_lower, real_t p_upper);

	void set_motor_enabled(bool p_enabled);
	bool is_motor_enabled() const;

	void set_motor_speed(real_t p_speed);
	real_t get_motor_speed() const;

	void set_max_motor_force(real_t p_force);
	real_t get_max_motor_force() const;

	real_t get_motor_force() const;

	Box2DPrismaticJoint();
};

class Box2DDistanceJoint : public Box2DJoint {
	GDCLASS(Box2DDistanceJoint, Box2DJoint);

	b2DistanceJointDef jointDef;

	real_t rest_length = 50;
	real_t min_length = 50;
	real_t max_length = 50;

	// TODO decide if there should be an editor toggle to lock relative limits
	// with the the anchor or with the rest point
	//bool editor_relative_limits = false;

	//virtual void on_editor_transforms_changed() override;

protected:
	static void _bind_methods();

	virtual void init_b2JointDef(const b2Vec2 &, bool) override;

	virtual void debug_draw(RID p_to_rid, Color p_color) override;

public:
	Vector2 get_body_local_anchor_a() const override { return b2_to_gd(jointDef.localAnchorA); }
	Vector2 get_body_local_anchor_b() const override { return b2_to_gd(jointDef.localAnchorB); }

	// Proxy to Box2DJoint for correct ClassDB binding
	void set_anchor_a(const Vector2 &p_anchor) { Box2DJoint::set_anchor_a(p_anchor); }
	Vector2 get_anchor_a() const { return Box2DJoint::get_anchor_a(); }
	void set_anchor_b(const Vector2 &p_anchor) { Box2DJoint::set_anchor_b(p_anchor); }
	Vector2 get_anchor_b() const { return Box2DJoint::get_anchor_b(); }

	void set_rest_length(real_t p_length);
	real_t get_rest_length() const;

	void reset_rest_length();

	void set_min_length(real_t p_length);
	real_t get_min_length() const;

	void set_max_length(real_t p_length);
	real_t get_max_length() const;

	void set_stiffness(real_t p_stiffness);
	real_t get_stiffness() const;

	void set_damping(real_t p_damping);
	real_t get_damping() const;

	// TODO set_spring_params()
	// OR   set/get spring_frequency (0-30hz), damping_ratio (0-1+)

	real_t get_current_length() const;

	Box2DDistanceJoint() :
			Box2DJoint(&jointDef) {
		anchor_b = Vector2(0, 50);
	}
};

// TODO pulley

// TODO mouse

// TODO gear

// TODO wheel

class Box2DWeldJoint : public Box2DJoint {
	GDCLASS(Box2DWeldJoint, Box2DJoint);

	b2WeldJointDef jointDef;

protected:
	static void _bind_methods();

	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos, bool p_soft_reset) override;

	virtual void debug_draw(RID p_to_rid, Color p_color) override;

public:
	Vector2 get_body_local_anchor_a() const override { return b2_to_gd(jointDef.localAnchorA); }
	Vector2 get_body_local_anchor_b() const override { return b2_to_gd(jointDef.localAnchorB); }

	// Proxy to Box2DJoint for correct ClassDB binding
	void set_anchor_a(const Vector2 &p_anchor) { Box2DJoint::set_anchor_a(p_anchor); }
	Vector2 get_anchor_a() const { return Box2DJoint::get_anchor_a(); }
	void set_anchor_b(const Vector2 &p_anchor) { Box2DJoint::set_anchor_b(p_anchor); }
	Vector2 get_anchor_b() const { return Box2DJoint::get_anchor_b(); }
	void reset_joint_anchors() { Box2DJoint::reset_joint_anchors(); }

	void set_stiffness(real_t p_hz); // TODO is this the best param name?
	real_t get_stiffness() const;

	void set_damping(real_t p_damping);
	real_t get_damping() const;

	Box2DWeldJoint() :
			Box2DJoint(&jointDef){};
};

// TODO friction

// TODO rope

// TODO motor

#endif // BOX2D_JOINTS_H
