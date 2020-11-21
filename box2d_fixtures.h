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
#include "box2d_world.h"

class Box2DFixture : public Node2D, public virtual IBox2DChildObject {
	GDCLASS(Box2DFixture, Node2D);

	friend class Box2DWorld;

public:
	enum FixtureType { // TODO figure out why the frick I added this
		CIRCLE = b2Shape::e_circle,
		//EDGE = b2Shape::e_edge, // TODO
		POLYGON = b2Shape::e_polygon, // TODO polygon: add N>8gon decomposition to allow N>8 polys OR just force use of chain shape
		//CHAIN = b2Shape::e_chain, // TODO
	};

private:
	b2Fixture *fixture;
	Box2DPhysicsBody *parent;

	void on_b2Fixture_destroyed(){}; // TODO is there any case when this is needed?

	bool create_b2Fixture();
	bool destroy_b2Fixture();

protected:
	void _notification(int p_what);
	static void _bind_methods();

	virtual void on_parent_created(Node *) override;

	b2FixtureDef fixtureDef;

	void update_shape();
	virtual void debug_draw(RID p_to_rid, Color p_color) = 0;

public:
	// TODO should this be in a TOOLS_ENABLED guard?
	virtual bool _edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const; // override;

	virtual String get_configuration_warning() const override;

	//virtual bool test_point(const Point2 &p_point); // TODO figure out how to handle this with edge/chain/(semantic poly made of chain?)

	// raycast
	// mass data?

	// gettype (shape type)
	FixtureType get_type() const;

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
			parent(NULL) {
		fixtureDef.density = 1.0f;
	};
};

class Box2DCircleFixture : public Box2DFixture {
	GDCLASS(Box2DCircleFixture, Box2DFixture);

	b2CircleShape shape;

protected:
	static void _bind_methods();

	virtual void debug_draw(RID p_to_rid, Color p_color) override;

public:
	void set_radius(real_t p_radius);
	real_t get_radius() const;

	Box2DCircleFixture();
};

class Box2DRectFixture : public Box2DFixture {
	GDCLASS(Box2DRectFixture, Box2DFixture);

	b2PolygonShape shape;
	real_t width;
	real_t height;

protected:
	static void _bind_methods();

	virtual void debug_draw(RID p_to_rid, Color p_color) override;

public:
	void set_width(real_t p_width);
	real_t get_width() const;

	void set_height(real_t p_height);
	real_t get_height() const;

	Box2DRectFixture();
};

//class Box2DPolygonFixture : public Box2DFixture {
//	GDCLASS(Box2DPolygonFixture, Box2DFixture);
//
//public:
//	enum BuildMode {
//		BUILD_SOLIDS,
//		//BUILD_SEGMENTS, // TODO
//	};
//
//private:
//	Vector<Point2> polygon;
//	PoolVector<b2PolygonShape> shape_vector; //TODO determine whether to use PoolVector/Vector. PoolVector is meant for larger arrays.
//
//protected:
//	static void _bind_methods();
//
//	virtual void debug_draw(RID p_to_rid, Color p_color) override;
//
//public:
//	void set_build_mode(BuildMode p_mode);
//	BuildMode get_build_mode() const;
//
//	Box2DPolygonFixture();
//};
//
//VARIANT_ENUM_CAST(Box2DPolygonFixture::BuildMode);

#endif // BOX2D_FIXTURES_H
