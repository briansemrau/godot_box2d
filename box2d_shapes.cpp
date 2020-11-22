#include "box2d_shapes.h"

#include <core/project_settings.h>
#include <servers/visual_server.h>

void Box2DShape::_bind_methods() {
	// Anything to bind?
}

Box2DShape::Box2DShape(b2Shape *const p_shape) :
		shape(p_shape) {}

bool Box2DShape::_edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const {
	const float factor = static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));

	b2CircleShape cursor;
	cursor.m_radius = p_tolerance / factor;

	Transform2D cursor_pos(0, p_point);

	return b2TestOverlap(shape, 0, &cursor, 0, gd_to_b2(Transform2D()), gd_to_b2(cursor_pos));
}

Box2DShape::Box2DShape() :
		shape(NULL) {
}

Box2DShape::~Box2DShape() {
	shape = NULL;
}

void Box2DCircleShape::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &Box2DCircleShape::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &Box2DCircleShape::get_radius);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "radius"), "set_radius", "get_radius");
}

void Box2DCircleShape::set_radius(real_t p_radius) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	circleShape.m_radius = p_radius * factor;
	emit_changed();
}

real_t Box2DCircleShape::get_radius() const {
	const float factor = static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	return circleShape.m_radius * factor;
}

void Box2DCircleShape::draw(const RID &p_to_rid, const Color &p_color) {
	// Same as in Godot's CircleShape2D::draw
	Vector<Vector2> points;
	for (int i = 0; i < 24; i++) {
		points.push_back(Vector2(Math::cos(i * Math_PI * 2 / 24.0), Math::sin(i * Math_PI * 2 / 24.0)) * get_radius());
	}

	Vector<Color> col;
	col.push_back(p_color);
	VisualServer::get_singleton()->canvas_item_add_polygon(p_to_rid, points, col);
}

Box2DCircleShape::Box2DCircleShape() :
		Box2DShape(&circleShape) {
	set_radius(25.0f);
}

void Box2DRectShape::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_width", "width"), &Box2DRectShape::set_width);
	ClassDB::bind_method(D_METHOD("get_width"), &Box2DRectShape::get_width);
	ClassDB::bind_method(D_METHOD("set_height", "height"), &Box2DRectShape::set_height);
	ClassDB::bind_method(D_METHOD("get_height"), &Box2DRectShape::get_height);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "width"), "set_width", "get_width");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "height"), "set_height", "get_height");
}

void Box2DRectShape::set_width(real_t p_width) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	width = p_width;
	shape.SetAsBox(width * factor * 0.5, height * factor * 0.5);
	emit_changed();
}

real_t Box2DRectShape::get_width() const {
	return width;
}

void Box2DRectShape::set_height(real_t p_height) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	height = p_height;
	shape.SetAsBox(width * factor * 0.5, height * factor * 0.5);
	emit_changed();
}

real_t Box2DRectShape::get_height() const {
	return height;
}

void Box2DRectShape::draw(const RID &p_to_rid, const Color &p_color) {
	Vector<Vector2> points;

	const real_t hx = width * 0.5;
	const real_t hy = height * 0.5;
	points.push_back(Vector2(hx, hy));
	points.push_back(Vector2(-hx, hy));
	points.push_back(Vector2(-hx, -hy));
	points.push_back(Vector2(hx, -hy));

	Vector<Color> col;
	col.push_back(p_color);
	VisualServer::get_singleton()->canvas_item_add_polygon(p_to_rid, points, col);
}

Box2DRectShape::Box2DRectShape() :
		Box2DShape(&shape), width(50.0f), height(50.0f) {
	shape.SetAsBox(width, height);
}

/*
void Box2DPolygonFixture::build_polygon() {
	Vector<Vector2> final_points = points;
	if (Geometry::is_polygon_clockwise(final_points)) {
		final_points.invert();
	}

	if (build_mode == BUILD_SOLIDS) {
		Geometry::decompose_polygon_in_convex(final_points);
	} else {
		// TODO
	}

	update_shape(); // TODO make this create multiple fixtures
}

void Box2DPolygonFixture::set_point_cloud(const Vector<Vector2> &p_points) {
	Vector<Point2> hull = Geometry::convex_hull_2d(p_points);
	ERR_FAIL_COND(hull.size() < 3);
	set_points(hull);
}

void Box2DPolygonFixture::set_points(const Vector<Vector2> &p_points) {
	points = p_points;
	build_polygon();
}

Vector<Vector2> Box2DPolygonFixture::get_points() const {
	return points;
}
//*/
