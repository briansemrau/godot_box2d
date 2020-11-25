#ifndef BOX2D_PHYSICS_BODY_H
#define BOX2D_PHYSICS_BODY_H

#include <core/object.h>
#include <core/reference.h>
#include <core/resource.h>
#include <core/vset.h>
#include <scene/2d/node_2d.h>

#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_world.h>

#include "../../util/box2d_types_converter.h"
#include "box2d_world.h"

/**
* @author Brian Semrau
*/

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
	b2MassData massDataDef;
	bool use_custom_massdata;
	real_t linear_damping;
	real_t angular_damping;
	b2Filter filterDef;

	VSet<Box2DPhysicsBody *> filtered;
	VSet<Box2DPhysicsBody *> filtering_me;
	// TODO i don't care enough right now to let bodies exclude specific fixtures

	b2Body *body;

	Box2DWorld *world_node;

	Set<Box2DJoint *> joints;

	Transform2D last_valid_xform;

	bool create_b2Body();
	bool destroy_b2Body();

	void update_mass(bool p_calc_reset = true);
	void update_filterdata();

protected:
	void _notification(int p_what);
	static void _bind_methods();

	virtual void on_parent_created(Node *) override;

public:
	virtual String get_configuration_warning() const override;

	void set_linear_velocity(const Vector2 &p_vel);
	Vector2 get_linear_velocity() const;

	void set_angular_velocity(const real_t p_omega);
	real_t get_angular_velocity() const;

	void set_use_custom_massdata(bool p_use_custom);
	bool get_use_custom_massdata() const;
	void set_mass(const real_t p_mass);
	real_t get_mass() const;
	void set_inertia(real_t p_inertia);
	real_t get_inertia() const;
	void set_center_of_mass(const Vector2 &p_center);
	Vector2 get_center_of_mass() const;
	void set_mass_data(real_t p_mass, real_t p_inertia, const Vector2 &p_center);

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

	void set_awake(bool p_awake);
	bool is_awake() const;
	void set_can_sleep(bool p_can_sleep);
	bool get_can_sleep() const;

	void set_fixed_rotation(bool p_fixed);
	bool is_fixed_rotation() const;

	void set_collision_layer(uint16_t p_layer);
	uint16_t get_collision_layer() const;

	void set_collision_mask(uint16_t p_mask);
	uint16_t get_collision_mask() const;

	void set_group_index(int16_t p_group_index);
	int16_t get_group_index() const;

	void set_filter_data(uint16_t p_layer, uint16_t p_mask, int16 p_group_index);

	Array get_collision_exceptions();
	void add_collision_exception_with(Node *p_node);
	void remove_collision_exception_with(Node *p_node);

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
