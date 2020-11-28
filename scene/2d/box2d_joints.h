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

	bool use_anchor_a = false;
	Vector2 anchor_a;
	bool use_anchor_b = false;
	Vector2 anchor_b;
	//bool editor_update_anchors = true;

	void on_b2Joint_destroyed();

	void update_joint_bodies(bool p_recalc_if_unchanged = false);

	bool create_b2Joint();
	bool destroy_b2Joint();

	void on_parent_created(Node *p_parent);
	void on_node_predelete(Box2DPhysicsBody *node);

	void _node_a_tree_entered();
	void _node_b_tree_entered();

protected:
	b2Joint *get_b2Joint() { return joint; }
	b2Joint *get_b2Joint() const { return joint; }

	b2Vec2 get_b2_pos() const;

	void _notification(int p_what);
	static void _bind_methods();

	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos, const b2Vec2 &p_world_anchor_a, const b2Vec2 &p_world_anchor_b) = 0;

	virtual void debug_draw(RID p_to_rid, Color p_color) = 0;

public:
	virtual String get_configuration_warning() const override;

	void set_nodepath_a(const NodePath &p_node_a);
	NodePath get_nodepath_a() const;

	void set_nodepath_b(const NodePath &p_node_b);
	NodePath get_nodepath_b() const;

	//void set_editor_update_anchors(bool p_update);
	//bool get_editor_update_anchors() const;

	// TODO documentation: Setting anchors is advanced behavior. These values are in world space.
	void set_use_custom_anchor_a(bool p_enable);
	bool get_use_custom_anchor_a() const;

	void set_anchor_a(const Vector2 &p_anchor);
	Vector2 get_anchor_a() const;

	void set_use_custom_anchor_b(bool p_enable);
	bool get_use_custom_anchor_b() const;

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

	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos, const b2Vec2 &p_world_anchor_a, const b2Vec2 &p_world_anchor_b) override;

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

protected:
	static void _bind_methods();

	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos, const b2Vec2 &p_world_anchor_a, const b2Vec2 &p_world_anchor_b) override;

	virtual void debug_draw(RID p_to_rid, Color p_color) override;

public:
	real_t get_reference_angle() const;
	Vector2 get_local_axis() const;
	real_t get_joint_translation() const;
	real_t get_joint_speed() const;

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

	Box2DPrismaticJoint() :
			Box2DJoint(&jointDef){};
};

//class Box2DDistanceJoint : public Box2DJoint {
//	GDCLASS(Box2DDistanceJoint, Box2DJoint);
//
//	b2DistanceJointDef jointDef;
//
//protected:
//	static void _bind_methods();
//
//	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos, const b2Vec2 &p_world_anchor_a, const b2Vec2 &p_world_anchor_b) override;
//
//	virtual void debug_draw(RID p_to_rid, Color p_color) override;
//
//public:
//	// TODO set/get
//
//	Box2DDistanceJoint() :
//			Box2DJoint(&jointDef){};
//};

// TODO pulley

// TODO mouse

// TODO gear

// TODO wheel

class Box2DWeldJoint : public Box2DJoint {
	GDCLASS(Box2DWeldJoint, Box2DJoint);

	b2WeldJointDef jointDef;

protected:
	static void _bind_methods();

	virtual void init_b2JointDef(const b2Vec2 &p_joint_pos, const b2Vec2 &p_world_anchor_a, const b2Vec2 &p_world_anchor_b) override;

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
