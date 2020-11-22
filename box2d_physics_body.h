#ifndef BOX2D_PHYSICS_BODY_H
#define BOX2D_PHYSICS_BODY_H

#include "core/object.h"
#include "core/reference.h"
#include "core/resource.h"

#include "scene/2d/node_2d.h"

#include "box2d/b2_body.h"
#include "box2d/b2_world.h"

#include "box2d_types_converter.h"
#include "box2d_child_object.h"

class Box2DWorld;

class Box2DPhysicsBody : public Node2D, public virtual IBox2DChildObject {
	GDCLASS(Box2DPhysicsBody, Node2D);

	friend class Box2DWorld;
	friend class Box2DFixture;
	friend class Box2DJoint;

public:
	enum Mode {
		MODE_STATIC = b2BodyType::b2_staticBody,
		MODE_KINEMATIC = b2BodyType::b2_kinematicBody,
		MODE_RIGID = b2BodyType::b2_dynamicBody,
	};

private:
	b2BodyDef bodyDef;
	b2Body *body = nullptr;

	Box2DWorld *world_node;

	Set<IBox2DChildObject *> fixtures;
	Set<IBox2DChildObject *> joints;

	Transform2D last_valid_xform;

	//void _state_changed();// TODO but probably not because box2D doesn't support this

	bool create_b2Body();
	bool destroy_b2Body();

protected:
	void _notification(int p_what);
	static void _bind_methods();

	virtual void on_parent_created(Node *) override;

public:
	virtual String get_configuration_warning() const override;

	//const Vector2 &get_world_center() const;
	//const Vector2 &get_local_center() const;

	void set_linear_velocity(const Vector2 &p_vel);
	Vector2 get_linear_velocity() const;

	void set_angular_velocity(const real_t p_omega);
	real_t get_angular_velocity() const;

	//real_t get_mass() const;
	//real_t get_inertia() const;
	//void set_mass(const real_t p_mass);
	//void set_inertia(const real_t p_inertia);
	//void reset_mass_data();

	// local/global transform functions not needed

	void set_linear_damping(real_t p_damping);
	real_t get_linear_damping() const;

	void set_angular_damping(real_t p_damping);
	real_t get_angular_damping() const;

	void set_gravity_scale(real_t p_scale);
	real_t get_gravity_scale() const;

	void set_type(Mode p_type);
	Mode get_type() const;

	void set_bullet(bool p_ccd);
	bool is_bullet() const;

	// get/set awake
	// get/set can sleep

	void set_fixed_rotation(bool p_fixed);
	bool is_fixed_rotation() const;

	void apply_force(const Vector2 &p_force, const Vector2 &p_point, bool p_wake = true);
	void apply_central_force(const Vector2 &p_force, bool p_wake = true);
	void apply_torque(real_t p_torque, bool p_wake = true);
	void apply_linear_impulse(const Vector2 &p_impulse, const Vector2 &p_point, bool p_wake = true);
	void apply_central_linear_impulse(const Vector2 &p_impulse, bool p_wake = true);
	void apply_torque_impulse(real_t p_impulse, bool p_wake = true);

	Box2DPhysicsBody();
	~Box2DPhysicsBody();
};

VARIANT_ENUM_CAST(Box2DPhysicsBody::Mode);

#endif // BOX2D_PHYSICS_BODY_H
