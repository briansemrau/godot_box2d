#ifndef BOX2D_AREA_H
#define BOX2D_AREA_H

#include <scene/2d/node_2d.h>
#include <core/array.h>

#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_world.h>

#include "box2d_collision_object.h"

#include "../../util/box2d_types_converter.h"

/**
* @author Brian Semrau
*/

class Box2DWorld;

class Box2DArea : public Box2DCollisionObject {
	GDCLASS(Box2DArea, Box2DCollisionObject);

public:
	enum SpaceOverride {
		SPACE_OVERRIDE_DISABLED,
		SPACE_OVERRIDE_COMBINE,
		SPACE_OVERRIDE_COMBINE_REPLACE,
		SPACE_OVERRIDE_REPLACE,
		SPACE_OVERRIDE_REPLACE_COMBINE
	};

private:
	SpaceOverride space_override = SPACE_OVERRIDE_DISABLED;
	Vector2 gravity_vec = Vector2(0, 1);
	real_t gravity = 98;
	bool gravity_is_point = false;
	real_t gravity_distance_scale = 0;
	real_t linear_damp = 0.1;
	real_t angular_damp = 1;
	int priority = 0;
	bool monitoring = false;
	bool monitorable = true;

	bool audio_bus_override = false;
	StringName audio_bus;

	Transform2D last_step_xform;

	virtual void _on_object_entered(Box2DCollisionObject *p_object) override;
	virtual void _on_object_exited(Box2DCollisionObject *p_object) override;
	virtual void _on_fixture_entered(Box2DFixture *p_fixture) override;
	virtual void _on_fixture_exited(Box2DFixture *p_fixture) override;

	virtual void pre_step(float p_delta) override;

protected:
	void _notification(int p_what);
	static void _bind_methods();
	void _validate_property(PropertyInfo &property) const override;

public:
	void set_space_override_mode(SpaceOverride p_mode);
	SpaceOverride get_space_override_mode() const;

	void set_gravity_is_point(bool p_enabled);
	bool is_gravity_a_point() const;

	void set_gravity_distance_scale(real_t p_scale);
	real_t get_gravity_distance_scale() const;

	void set_gravity_vector(const Vector2 &p_vec);
	Vector2 get_gravity_vector() const;

	void set_gravity(real_t p_gravity);
	real_t get_gravity() const;

	void set_linear_damp(real_t p_linear_damp);
	real_t get_linear_damp() const;

	void set_angular_damp(real_t p_angular_damp);
	real_t get_angular_damp() const;

	void set_priority(int p_priority);
	int get_priority() const;

	void set_monitoring(bool p_enable);
	bool is_monitoring() const;

	void set_monitorable(bool p_enable);
	bool is_monitorable() const;

	void set_collision_mask_bit(int p_bit, bool p_value);
	bool get_collision_mask_bit(int p_bit) const;

	void set_collision_layer_bit(int p_bit, bool p_value);
	bool get_collision_layer_bit(int p_bit) const;

	Array get_overlapping_bodies() const;
	Array get_overlapping_areas() const;

	bool overlaps_area(Node *p_area) const;
	bool overlaps_body(Node *p_body) const;

	void set_audio_bus_override(bool p_override);
	bool is_overriding_audio_bus() const;

	void set_audio_bus_name(const StringName &p_audio_bus);
	StringName get_audio_bus_name() const;
	
	Box2DArea();
	~Box2DArea();
};

VARIANT_ENUM_CAST(Box2DArea::SpaceOverride);

#endif // BOX2D_AREA_H
