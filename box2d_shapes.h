#ifndef BOX2D_SHAPES_H
#define BOX2D_SHAPES_H

#include "editor/plugins/abstract_polygon_2d_editor.h"
#include <core/resource.h>

#include <box2d/b2_circle_shape.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_shape.h>

#include "box2d_types_converter.h"

#define DEBUG_DECOMPOSE_BOX2D

class Box2DShape : public Resource {
	GDCLASS(Box2DShape, Resource);
	OBJ_SAVE_TYPE(Box2DShape);

	friend class Box2DFixture;

	b2Shape *shape;

protected:
	static void _bind_methods();

	Box2DShape(b2Shape *const p_shape);

public:
	virtual bool _edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const;

	virtual void draw(const RID &p_to_rid, const Color &p_color) = 0;

	Box2DShape();
	~Box2DShape();
};

class Box2DCircleShape : public Box2DShape {
	GDCLASS(Box2DCircleShape, Box2DShape);

	b2CircleShape circleShape;

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

protected:
	static void _bind_methods();

public:
	void set_width(real_t p_width);
	real_t get_width() const;

	void set_height(real_t p_height);
	real_t get_height() const;

	virtual void draw(const RID &p_to_rid, const Color &p_color) override;

	Box2DRectShape();
};

class Box2DPolygonShape : public Box2DShape {
	GDCLASS(Box2DPolygonShape, Box2DShape);

	friend class Box2DFixture;

	Vector<Vector2> points;
	Vector<b2PolygonShape> shape_vector;

#ifdef DEBUG_DECOMPOSE_BOX2D
	Vector<Vector<Vector2> > decomposed;
#endif

	void build_polygon();

protected:
	static void _bind_methods();

public:
	bool _edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const override;

	void set_point_cloud(const Vector<Vector2> &p_points);
	void set_points(const Vector<Vector2> &p_points);
	Vector<Vector2> get_points() const;

	virtual void draw(const RID &p_to_rid, const Color &p_color) override;

	Box2DPolygonShape();
};

//class Box2DPolygonFixture : public Box2DFixture {
//	GDCLASS(Box2DPolygonFixture, Box2DFixture);
//
//public:
//	enum BuildMode {
//		BUILD_SOLIDS,
//		BUILD_SEGMENTS,
//	};
//
//private:
//	BuildMode build_mode;
//
//	Vector<Point2> points;
//	Vector<b2PolygonShape> shape_vector; //TODO determine whether to use PoolVector/Vector. PoolVector is meant for larger arrays.
//
//	Vector<b2Fixture *> b2FixtureVec;
//
//	void build_polygon();
//
//protected:
//	static void _bind_methods();
//
//	virtual bool create_b2Fixture() override;
//	virtual bool destroy_b2Fixture() override;
//
//	virtual void debug_draw(RID p_to_rid, Color p_color) override;
//
//public:
//	void set_build_mode(BuildMode p_mode);
//	BuildMode get_build_mode() const;
//
//	void set_point_cloud(const Vector<Vector2> &p_points);
//	void set_points(const Vector<Vector2> &p_points);
//	Vector<Vector2> get_points() const;
//
//	Box2DPolygonFixture();
//};
//
//VARIANT_ENUM_CAST(Box2DPolygonFixture::BuildMode);

//class Box2DChainFixture : public Box2DFixture {
//	GDCLASS(Box2DChainFixture, Box2DFixture);
//	// TODO
//};


// TODO move this to a separate header
class Box2DPolygonEditor : public AbstractPolygon2DEditor {
	GDCLASS(Box2DPolygonEditor, AbstractPolygon2DEditor);

	Box2DFixture *node;
	Box2DPolygonShape *shape;

protected:
	virtual Node2D *_get_node() const override;
	virtual void _set_node(Node *p_polygon) override;

	virtual Variant _get_polygon(int p_idx) const;
	virtual void _set_polygon(int p_idx, const Variant &p_polygon) const;

	virtual void _action_set_polygon(int p_idx, const Variant &p_previous, const Variant &p_polygon);

public:
	Box2DPolygonEditor(EditorNode *p_editor);
};

class Box2DPolygonEditorPlugin : public AbstractPolygon2DEditorPlugin {
	GDCLASS(Box2DPolygonEditorPlugin, AbstractPolygon2DEditorPlugin);

public:
	virtual bool handles(Object *p_object) const;

	Box2DPolygonEditorPlugin(EditorNode *p_node);
};

#endif // BOX2D_SHAPES_H
