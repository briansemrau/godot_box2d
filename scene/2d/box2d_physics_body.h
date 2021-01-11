#ifndef BOX2D_PHYSICS_BODY_H
#define BOX2D_PHYSICS_BODY_H

#include <core/io/resource.h>
#include <core/object/object.h>
#include <core/object/reference.h>
#include <core/templates/vset.h>
#include <scene/2d/node_2d.h>

#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_world.h>

#include "../../util/box2d_types_converter.h"

#include "box2d_area.h"
#include "box2d_collision_object.h"
#include "box2d_world.h"

/**
* @author Brian Semrau
*/

class Box2DWorld;
class Box2DKinematicCollision;

class Box2DPhysicsBody : public Box2DCollisionObject {
	GDCLASS(Box2DPhysicsBody, Box2DCollisionObject);

	friend class Box2DWorld;
	friend class Box2DFixture;
	friend class Box2DJoint;

public:
	enum Mode {
		MODE_STATIC = b2BodyType::b2_staticBody,
		MODE_KINEMATIC = b2BodyType::b2_kinematicBody,
		MODE_RIGID = b2BodyType::b2_dynamicBody,
	};

	struct KinematicCollision {
		Vector2 collision_point;
		Vector2 normal;
		Vector2 collider_vel;
		Box2DFixture *collider_fixture = nullptr; // use pointer or objectID? might be a crash if obj is freed
		Vector2 remainder;
		Vector2 travel;
		Box2DFixture *local_fixture = nullptr;
	};

private:
	b2MassData massDataDef{ 1.0f, b2Vec2_zero, 0.5f }; // default for a disk of 1kg, 1m radius
	bool use_custom_massdata = false;
	real_t linear_damping = 0.0f;
	real_t angular_damping = 0.0f;

	VSet<Box2DPhysicsBody *> filtered;
	VSet<Box2DPhysicsBody *> filtering_me;
	// TODO i don't care enough right now to let bodies exclude specific fixtures

	Set<Box2DJoint *> joints;

	Transform2D last_valid_xform;

	// For sorting colliding areas by priority
	struct Box2DAreaItem {
		const Box2DArea *area = NULL;
		inline bool operator==(const Box2DAreaItem &p_item) const { return area == p_item.area; }
		inline bool operator<(const Box2DAreaItem &p_item) const { return area->get_priority() < p_item.area->get_priority(); }
		inline Box2DAreaItem() {}
		inline Box2DAreaItem(const Box2DArea *p_area) {
			area = p_area;
		}
	};

	Vector<Box2DAreaItem> colliding_areas;

	//Transform2D last_valid_xform; // this is for sync_to_physics, but is it needed?
	Transform2D old_xform; // For calculating kinematic body movement velocity
	bool kinematic_integrate_velocity = false; // Default false is Godot behavior, true is Box2D behavior

	Ref<Box2DKinematicCollision> motion_cache;

	// TODO maybe keep a list of local state we want this class to track wrt a b2body parameter or field
	// are there any others?  enabled for example can bet set on the fly in code
	bool prev_sleeping_state = true;

	void update_mass(bool p_calc_reset = true);

	void _compute_area_effects(const Box2DArea *p_area, b2Vec2 &p_gravity, float &p_lin_damp, float &p_ang_damp);
	void _update_area_effects();

	void sync_state();

	void teleport(const Transform2D &p_transform);

	bool _move_and_collide(const Vector2 &p_motion, const float p_rotation, const bool p_infinite_inertia, KinematicCollision &r_collision, const bool p_exclude_raycast_shapes = true, const bool p_test_only = false);

	virtual void _on_object_entered(Box2DCollisionObject *p_object) override;
	virtual void _on_object_exited(Box2DCollisionObject *p_object) override;
	virtual void _on_fixture_entered(Box2DFixture *p_fixture) override;
	virtual void _on_fixture_exited(Box2DFixture *p_fixture) override;

protected:
	virtual void on_b2Body_created() override;

	virtual void pre_step(float p_delta) override;

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void _add_area(Box2DArea *p_area);
	void _remove_area(Box2DArea *p_area);
	void _remove_area_variant(const Variant &p_area);

	virtual String get_configuration_warning() const override;

	void _set_linear_velocity_no_check(const Vector2 &p_vel);
	void set_linear_velocity(const Vector2 &p_vel);
	Vector2 get_linear_velocity() const;

	void _set_angular_velocity_no_check(const real_t p_omega);
	void set_angular_velocity(const real_t p_omega);
	real_t get_angular_velocity() const;

	void set_use_custom_massdata(bool p_use_custom);
	bool get_use_custom_massdata() const;
	void set_custom_mass(const real_t p_mass);
	real_t get_custom_mass() const;
	void set_custom_inertia(real_t p_inertia);
	real_t get_custom_inertia() const;
	void set_custom_center_of_mass(const Vector2 &p_center);
	Vector2 get_custom_center_of_mass() const;
	void set_custom_mass_data(real_t p_mass, real_t p_inertia, const Vector2 &p_center);

	real_t get_mass() const;
	real_t get_inertia() const;
	Vector2 get_center_of_mass() const;

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

	Array get_collision_exceptions();
	void add_collision_exception_with(Node *p_node);
	void remove_collision_exception_with(Node *p_node);

	void set_contact_monitor(bool p_enabled);
	bool is_contact_monitor_enabled() const;

	void set_max_contacts_reported(int p_amount);
	int get_max_contacts_reported() const;

	Array get_colliding_bodies() const; // Function exists for Godot feature congruency

	// TODO for documentation: all contact info is in world space
	int get_contact_count() const;
	// get contact ID (for lifecycle tracking purposes)
	Box2DFixture *get_contact_fixture_a(int p_idx) const;
	Box2DFixture *get_contact_fixture_b(int p_idx) const;
	Vector2 get_contact_world_pos(int p_idx) const;
	//Vector2 get_contact_local_pos(int p_idx) const;
	Vector2 get_contact_impact_velocity(int p_idx) const;
	Vector2 get_contact_normal(int p_idx) const;
	float get_contact_normal_impulse(int p_idx) const;
	Vector2 get_contact_tangent_impulse(int p_idx) const;
	//Vector2 get_contact_total_impulse(int p_idx) const;
	//bool get_contact_is_new(int p_idx) const;

	// Rigid body functions

	void apply_force(const Vector2 &p_force, const Vector2 &p_point, bool p_wake = true);
	void apply_central_force(const Vector2 &p_force, bool p_wake = true);
	void apply_torque(real_t p_torque, bool p_wake = true);
	void apply_linear_impulse(const Vector2 &p_impulse, const Vector2 &p_point, bool p_wake = true);
	void apply_central_linear_impulse(const Vector2 &p_impulse, bool p_wake = true);
	void apply_torque_impulse(real_t p_impulse, bool p_wake = true);

	// Kinematic body functions

	void set_kinematic_integrate_velocity(bool p_integrate_vel);
	bool is_kinematic_integrating_velocity() const;

	// p_exclude_raycast_shapes is unused
	Ref<Box2DKinematicCollision> move_and_collide(const Vector2 &p_motion, const float p_rotation, const bool p_infinite_inertia = true, const bool p_exclude_raycast_shapes = true, const bool p_test_only = false);
	//bool test_move(const Transform2D &p_from, const Vector2 &p_motion, bool p_infinite_inertia = true);
	//Vector2 move_and_slide(const Vector2 &p_linear_velocity, const Vector2 &p_up_direction = Vector2(0, 0), bool p_stop_on_slope = false, int p_max_slides = 4, float p_floor_max_angle = Math::deg2rad((float)45), bool p_infinite_inertia = true);
	//Vector2 move_and_slide_with_snap(const Vector2 &p_linear_velocity, const Vector2 &p_snap, const Vector2 &p_up_direction = Vector2(0, 0), bool p_stop_on_slope = false, int p_max_slides = 4, float p_floor_max_angle = Math::deg2rad((float)45), bool p_infinite_inertia = true);
	//bool is_on_floor() const;
	//bool is_on_wall() const;
	//bool is_on_ceiling() const;
	//Vector2 get_floor_normal() const;
	//Vector2 get_floor_velocity() const;

	Box2DPhysicsBody();
	~Box2DPhysicsBody();
};

VARIANT_ENUM_CAST(Box2DPhysicsBody::Mode);

class Box2DKinematicCollision : public Reference {
	GDCLASS(Box2DKinematicCollision, Reference);

	friend class Box2DPhysicsBody;

	Box2DPhysicsBody *owner;
	Box2DPhysicsBody::KinematicCollision collision;

protected:
	static void _bind_methods();

public:
	Vector2 get_position() const;
	Vector2 get_normal() const;
	Vector2 get_travel() const;
	Vector2 get_remainder() const;
	Object *get_local_fixture() const;
	Object *get_collider() const;
	ObjectID get_collider_id() const;
	Object *get_collider_fixture() const;
	ObjectID get_collider_fixture_id() const;
	Vector2 get_collider_velocity() const;

	//Box2DKinematicCollision();
};

#endif // BOX2D_PHYSICS_BODY_H
