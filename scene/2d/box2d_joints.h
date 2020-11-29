#ifndef BOX2D_JOINTS_H
#define BOX2D_JOINTS_H

#include <core/object.h>
#include <core/reference.h>
#include <core/resource.h>
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

#include "../../util/box2d_types_converter.h"
#include "box2d_physics_body.h"

/**
* @author Brian Semrau
*/

class Box2DWorld;

class Box2DJoint : public Node2D {
	GDCLASS(Box2DJoint, Node2D);

	friend class Box2DWorld;
	friend class Box2DPhysicsBody;

	b2JointDef *jointDef;
	b2Joint *joint;

	Box2DWorld *world_node;

	NodePath a;
	NodePath b;

	ObjectID bodyA_cache;
	ObjectID bodyB_cache;

	bool broken;
	bool breaking_enabled;
	bool free_on_break; // TODO This feature may not be necessary. Perhaps this should be handled by a "on_broken" signal.
	real_t max_force;
	real_t max_torque;

	// Joint node local-space anchor points. These sync with jointDef's body-local anchor points.
	// Allows configuring anchor points to be in different world positions when added to the scene.
	Vector2 anchor_a = Vector2();
	Vector2 anchor_b = Vector2();
	// Controls whether moving the joint or linked bodies in the editor can modify the jointDef body-local anchor points.
	// With this option off, joints anchors can be configured in different world locations.
	// Useful for broken joints that start broken and separated.
	bool editor_use_default_anchors = true;

	void on_b2Joint_destroyed();

	bool create_b2Joint();
	bool destroy_b2Joint();

	void on_parent_created(Node *p_parent);
	void on_node_predelete(Box2DPhysicsBody *node);
	void on_editor_transforms_changed();

	void _node_a_tree_entered();
	void _node_b_tree_entered();

protected:
	// Rescans the nodepaths to find b2Bodies and create our joint
	// Call this function with p_reinit_if_unchanged=true if you need to recreate the b2Joint
	//    from an updated jointDef whether the bodies have changed or not
	void update_joint_bodies(bool p_force_reinit = false);

	b2Joint *get_b2Joint() { return joint; }
	b2Joint *get_b2Joint() const { return joint; }

	b2Vec2 get_b2_pos() const;

	void _notification(int p_what);
	static void _bind_methods();

	// Allows all joint types to perform their custom initializations.
	void _init_b2JointDef(const b2Vec2 &p_joint_pos);
	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos) = 0;

	virtual void set_b2_anchor_a(b2Vec2 &p_vec) = 0;
	virtual b2Vec2 get_b2_anchor_a() const = 0;
	virtual void set_b2_anchor_b(b2Vec2 &p_vec) = 0;
	virtual b2Vec2 get_b2_anchor_b() const = 0;

	virtual void debug_draw(RID p_to_rid, Color p_color) = 0;

public:
	virtual String get_configuration_warning() const override;

	void set_nodepath_a(const NodePath &p_node_a);
	NodePath get_nodepath_a() const;

	void set_nodepath_b(const NodePath &p_node_b);
	NodePath get_nodepath_b() const;

	void set_editor_use_default_anchors(bool p_update);
	bool get_editor_use_default_anchors() const;

	// TODO documentation: Setting anchors is advanced behavior. These values are in joint node local space.
	void set_anchor_a(const Vector2 &p_anchor);
	Vector2 get_anchor_a() const;

	void set_anchor_b(const Vector2 &p_anchor);
	Vector2 get_anchor_b() const;

	// TODO IMPORTANT for (future) documentation:
	//      this function lets you re-place the joint on the same bodies with the joint's current pos and current body relative positions
	void reset_joint_anchors();

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

	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos) override;
	virtual void set_b2_anchor_a(b2Vec2 &p_vec) override { jointDef.localAnchorA = p_vec; }
	virtual b2Vec2 get_b2_anchor_a() const override { return jointDef.localAnchorA; }
	virtual void set_b2_anchor_b(b2Vec2 &p_vec) override { jointDef.localAnchorB = p_vec; }
	virtual b2Vec2 get_b2_anchor_b() const override { return jointDef.localAnchorB; }

	virtual void debug_draw(RID p_to_rid, Color p_color) override;

public:
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

	Box2DRevoluteJoint() :
			Box2DJoint(&jointDef){};
};

class Box2DPrismaticJoint : public Box2DJoint {
	GDCLASS(Box2DPrismaticJoint, Box2DJoint);

	b2PrismaticJointDef jointDef;

	// TODO the way this is managed is far from final
	// there are some issues I have with it
	Vector2 local_axis = Vector2(1.0f, 0);
	bool editor_use_default_axis = false;

protected:
	static void _bind_methods();

	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos) override;
	virtual void set_b2_anchor_a(b2Vec2 &p_vec) override { jointDef.localAnchorA = p_vec; }
	virtual b2Vec2 get_b2_anchor_a() const override { return jointDef.localAnchorA; }
	virtual void set_b2_anchor_b(b2Vec2 &p_vec) override { jointDef.localAnchorB = p_vec; }
	virtual b2Vec2 get_b2_anchor_b() const override { return jointDef.localAnchorB; }

	virtual void debug_draw(RID p_to_rid, Color p_color) override;

public:
	real_t get_reference_angle() const;
	real_t get_joint_translation() const;
	real_t get_joint_speed() const;

	void set_editor_use_default_axis(bool p_default);
	bool get_editor_use_default_axis() const;

	void set_local_axis(Vector2 p_axis);
	Vector2 get_local_axis() const;

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

// TODO distance

// TODO pulley

// TODO mouse

// TODO gear

// TODO wheel

class Box2DWeldJoint : public Box2DJoint {
	GDCLASS(Box2DWeldJoint, Box2DJoint);

	b2WeldJointDef jointDef;

protected:
	static void _bind_methods();

	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos) override;
	virtual void set_b2_anchor_a(b2Vec2 &p_vec) override { jointDef.localAnchorA = p_vec; }
	virtual b2Vec2 get_b2_anchor_a() const override { return jointDef.localAnchorA; }
	virtual void set_b2_anchor_b(b2Vec2 &p_vec) override { jointDef.localAnchorB = p_vec; }
	virtual b2Vec2 get_b2_anchor_b() const override { return jointDef.localAnchorB; }

	virtual void debug_draw(RID p_to_rid, Color p_color) override;

public:
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
