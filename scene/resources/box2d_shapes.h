#ifndef BOX2D_SHAPES_H
#define BOX2D_SHAPES_H

#include <core/resource.h>

#include <box2d/b2_chain_shape.h>
#include <box2d/b2_circle_shape.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_shape.h>

#include "../../util/box2d_types_converter.h"

/**
* @author Brian Semrau
*/

#define DEBUG_DECOMPOSE_BOX2D

class Box2DShape : public Resource {
	GDCLASS(Box2DShape, Resource);
	OBJ_SAVE_TYPE(Box2DShape);

	friend class Box2DFixture;

	virtual bool is_composite_shape() const;
	virtual const Vector<const b2Shape *> get_shapes() const;
	virtual const b2Shape *get_shape() const = 0;

protected:
	static void _bind_methods();

public:
	virtual bool _edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const;

	virtual void draw(const RID &p_to_rid, const Color &p_color) = 0;

	Box2DShape(){};
	~Box2DShape(){};
};

class Box2DCircleShape : public Box2DShape {
	GDCLASS(Box2DCircleShape, Box2DShape);

	b2CircleShape circleShape;

	virtual const b2Shape *get_shape() const { return &circleShape; }

protected:
	static void _bind_methods();

public:
	void set_radius(real_t p_radius);
	real_t get_radius() const;

	virtual void draw(const RID &p_to_rid, const Color &p_color) override;

	Box2DCircleShape();
};

class Box2DRectShape : public Box2DShape {
	GDCLASS(Box2DRectShape, Box2DShape);

	b2PolygonShape shape;
	real_t width;
	real_t height;
	// TODO replace width/height with a Vector2 for consistency

	virtual const b2Shape *get_shape() const { return &shape; }

protected:
	static void _bind_methods();

public:
	void set_size(const Vector2 &p_size);
	Vector2 get_size() const;

	void set_width(real_t p_width);
	real_t get_width() const;

	void set_height(real_t p_height);
	real_t get_height() const;

	virtual void draw(const RID &p_to_rid, const Color &p_color) override;

	Box2DRectShape();
};

class Box2DSegmentShape : public Box2DShape {
	GDCLASS(Box2DSegmentShape, Box2DShape);

	b2EdgeShape shape;

	virtual const b2Shape *get_shape() const { return &shape; }

protected:
	static void _bind_methods();

public:
	void set_a(const Vector2 &p_a);
	Vector2 get_a() const;

	void set_b(const Vector2 &p_b);
	Vector2 get_b() const;

	void set_a_adjacent(const Vector2 &p_a_adj);
	Vector2 get_a_adjacent() const;

	void set_b_adjacent(const Vector2 &p_b_adj);
	Vector2 get_b_adjacent() const;

	void set_one_sided(bool p_one_sided);
	bool is_one_sided() const;

	void set_as_one_sided(const Vector2 &p_a_adj, const Vector2 &p_a, const Vector2 &p_b, const Vector2 &p_b_adj);
	void set_as_two_sided(const Vector2 &p_a, const Vector2 &p_b);

	virtual void draw(const RID &p_to_rid, const Color &p_color) override;

	Box2DSegmentShape();
};

class Box2DPolygonShape : public Box2DShape {
	GDCLASS(Box2DPolygonShape, Box2DShape);

	friend class Box2DFixture;

public:
	enum BuildMode {
		BUILD_SOLIDS,
		BUILD_SEGMENTS,
		BUILD_OPEN_SEGMENTS,
		BUILD_OVERLAPPING_SOLIDS,
	};

private:
	BuildMode build_mode;

	bool invert_order; // TODO could there be a clearer name for this? does Godot have a naming convention?
	Vector<Vector2> points;

	// Used in SOLIDS mode
	Vector<b2PolygonShape> polygon_shape_vector;

	// Used in SEGMENTS mode
	b2ChainShape *chain_shape;

#ifdef DEBUG_DECOMPOSE_BOX2D
	Vector<Vector<Vector2> > decomposed;
#endif

	void build_polygon();

protected:
	static void _bind_methods();

	virtual bool is_composite_shape() const override;
	virtual const Vector<const b2Shape *> get_shapes() const;
	virtual const b2Shape *get_shape() const { return chain_shape; }

public:
	bool _edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const override;

	void set_point_cloud(const Vector<Vector2> &p_points);
	void set_points(const Vector<Vector2> &p_points);
	Vector<Vector2> get_points() const;

	void set_build_mode(BuildMode p_mode);
	BuildMode get_build_mode() const;

	void set_invert_order(bool p_inverted);
	bool get_invert_order() const;

	virtual void draw(const RID &p_to_rid, const Color &p_color) override;

	Box2DPolygonShape();
	~Box2DPolygonShape();
};

VARIANT_ENUM_CAST(Box2DPolygonShape::BuildMode);

#endif // BOX2D_SHAPES_H
