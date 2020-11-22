
#ifndef BOX2D_JOINTS_H
#define BOX2D_JOINTS_H

#include "core/object.h"
#include "core/reference.h"
#include "core/resource.h"

#include "scene/2d/node_2d.h"

#include "box2d/b2_distance_joint.h"
#include "box2d/b2_friction_joint.h"
#include "box2d/b2_gear_joint.h"
#include "box2d/b2_joint.h"
#include "box2d/b2_motor_joint.h"
#include "box2d/b2_mouse_joint.h"
#include "box2d/b2_prismatic_joint.h"
#include "box2d/b2_pulley_joint.h"
#include "box2d/b2_revolute_joint.h"
#include "box2d/b2_rope.h"
#include "box2d/b2_weld_joint.h"
#include "box2d/b2_wheel_joint.h"

#include "box2d_child_object.h"
#include "box2d_physics_body.h"
#include "box2d_types_converter.h"

class Box2DWorld;
class IBox2DChildObject;

class Box2DJoint : public Node2D, public virtual IBox2DChildObject {
	GDCLASS(Box2DJoint, Node2D);

	friend class Box2DWorld;

public:
	// TODO do we need a joint type enum?

private:
	b2JointDef *jointDef = nullptr;
	b2Joint *joint = nullptr;

	Box2DWorld *parent;

	NodePath a;
	NodePath b;

	ObjectID bodyA_cache;
	ObjectID bodyB_cache;

	bool breaking_enabled;
	real_t max_force;
	real_t max_torque;

	void on_b2Joint_destroyed();

	void update_joint_bodies();

	bool create_b2Joint();
	bool destroy_b2Joint();

protected:
	b2Joint *get_b2Joint() { return joint; }
	b2Joint *get_b2Joint() const { return joint; }

	void _notification(int p_what);
	static void _bind_methods();

	virtual void on_parent_created(Node *p_parent) override;

	virtual void init_b2JointDef() = 0;

	virtual void debug_draw(RID p_to_rid, Color p_color) = 0;

public:
	virtual String get_configuration_warning() const override;

	// TODO enabled property
	// Breaking should set enabled -> false

	// TODO decide on free_on_break property

	void set_node_a(const NodePath &p_node_a);
	NodePath get_node_a() const;

	void set_node_b(const NodePath &p_node_b);
	NodePath get_node_b() const;

	void set_collide_connected(bool p_collide);
	bool get_collide_connected() const;

	void set_breaking_enabled(bool p_enabled);
	bool is_breaking_enabled() const;

	void set_max_force(real_t p_max_force);
	real_t get_max_force() const;

	void set_max_torque(real_t p_max_torque);
	real_t get_max_torque() const;

	Vector2 get_reaction_force() const;
	real_t get_reaction_torque() const;

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

	virtual void init_b2JointDef() override;

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

	Box2DRevoluteJoint() :
			Box2DJoint(&jointDef){};
};

// TODO prismatic

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

	virtual void init_b2JointDef() override;

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

#endif // !BOX2D_JOINTS_H
