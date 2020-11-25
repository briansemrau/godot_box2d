#ifndef BOX2D_FIXTURES_H
#define BOX2D_FIXTURES_H

#include "core/object.h"
#include "core/reference.h"
#include "core/resource.h"

#include "scene/2d/node_2d.h"

#include "box2d/b2_chain_shape.h"
#include "box2d/b2_circle_shape.h"
#include "box2d/b2_edge_shape.h"
#include "box2d/b2_fixture.h"
#include "box2d/b2_polygon_shape.h"

#include "box2d_types_converter.h"

#include "box2d_physics_body.h"
#include "box2d_shapes.h"
#include "box2d_world.h"

class Box2DFixture : public Node2D, public virtual IBox2DChildObject {
	GDCLASS(Box2DFixture, Node2D);

	friend class Box2DWorld;

	Box2DPhysicsBody *body_node;

	void on_b2Fixture_destroyed(){}; // TODO is there any case when this is needed?

	void _shape_changed();

//protected:
	Ref<Box2DShape> shape;
	b2FixtureDef fixtureDef;
	b2Filter filterDef;
	bool override_body_filterdata;
	bool accept_body_collision_exceptions;
	// TODO maybe implement a HashSet or use std
	// Not sure why Godot uses a VSet for this
	VSet<Box2DFixture *> filtered;
	VSet<Box2DFixture *> filtering_me;
	// TODO might fixtures need to filter other whole bodies?

	Vector<b2Fixture *> fixtures;

	void _notification(int p_what);
	static void _bind_methods();

	virtual void on_parent_created(Node *) override final;

	void create_b2Fixture(b2Fixture *&p_fixture_out, const b2FixtureDef &p_def, const Transform2D &p_shape_xform);

	bool create_b2();
	bool destroy_b2();

	void update_shape();
	void update_filterdata();

public:
	// TODO should this be in a TOOLS_ENABLED guard?
	virtual bool _edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const override;

	virtual String get_configuration_warning() const override;

	//virtual bool test_point(const Point2 &p_point); // TODO figure out how to handle this with edge/chain/(semantic poly made of chain?)

	// raycast

	void set_shape(const Ref<Box2DShape> &p_shape);
	Ref<Box2DShape> get_shape();

	// sensor?

	void set_override_body_collision(bool p_override);
	bool get_override_body_collision() const;

	void set_collision_layer(uint16_t p_layer);
	uint16_t get_collision_layer() const;

	void set_collision_mask(uint16_t p_mask);
	uint16_t get_collision_mask() const;

	void set_group_index(int16_t p_group_index);
	int16_t get_group_index() const;

	void set_filter_data(uint16_t p_layer, uint16_t p_mask, int16 p_group_index);

	void set_use_parent_exceptions(bool p_use);
	bool get_use_parent_exceptions() const;

	Array get_collision_exceptions();
	void add_collision_exception_with(Node *p_node);
	void remove_collision_exception_with(Node *p_node);

	void set_density(real_t p_density);
	real_t get_density() const;

	void set_friction(real_t p_friction);
	real_t get_friction() const;

	void set_restitution(real_t p_restitution);
	real_t get_restitution() const;

	// restitution threshold?

	Box2DFixture();
	~Box2DFixture();
};

#endif // BOX2D_FIXTURES_H
