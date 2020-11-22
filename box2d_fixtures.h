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

public:
	//enum FixtureType { // TODO figure out why the frick I added this
	//	CIRCLE = b2Shape::e_circle,
	//	//EDGE = b2Shape::e_edge, // TODO
	//	POLYGON = b2Shape::e_polygon, // TODO polygon: add N>8gon decomposition to allow N>8 polys OR just force use of chain shape
	//	//CHAIN = b2Shape::e_chain, // TODO
	//};

private:
	Box2DPhysicsBody *body_node;

	void on_b2Fixture_destroyed(){}; // TODO is there any case when this is needed?

	void _shape_changed();

protected:
	Ref<Box2DShape> shape;

	b2Fixture *fixture;
	b2FixtureDef fixtureDef;

	void _notification(int p_what);
	static void _bind_methods();

	virtual void on_parent_created(Node *) override final;

	void create_b2Fixture(b2Body *p_body, b2Fixture *&p_fixture_out, const b2FixtureDef &p_def, const Transform2D &p_shape_xform);

	virtual bool create_b2();
	virtual bool destroy_b2();

	void update_shape();

public:
	// TODO should this be in a TOOLS_ENABLED guard?
	virtual bool _edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const; // override;

	virtual String get_configuration_warning() const override;

	//virtual bool test_point(const Point2 &p_point); // TODO figure out how to handle this with edge/chain/(semantic poly made of chain?)

	// raycast
	// mass data?

	// gettype (shape type)
	//FixtureType get_type() const;

	void set_shape(const Ref<Box2DShape> &p_shape);
	Ref<Box2DShape> get_shape();

	// sensor?
	// filterdata?

	void set_density(real_t p_density);
	real_t get_density() const;

	void set_friction(real_t p_friction);
	real_t get_friction() const;

	void set_restitution(real_t p_restitution);
	real_t get_restitution() const;

	// restitution threshold?

	Box2DFixture() :
			fixture(NULL),
			body_node(NULL) {
		fixtureDef.density = 1.0f;
	};
	~Box2DFixture();
};

#endif // BOX2D_FIXTURES_H
