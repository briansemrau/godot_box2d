#include "box2d_shapes.h"

#include <core/project_settings.h>
#include <servers/visual_server.h>

#include "box2d_fixtures.h"
#include "box2d_types_converter.h"

#include <string> // TODO remove this. For debugging only

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
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	shape.SetAsBox(width * factor * 0.5, height * factor * 0.5);
}

bool isPolygonValid(const b2Vec2 *vertices, int32 count) {
	// This function is copied from b2PolygonShape::Set
	// See why this is needed: https://github.com/erincatto/box2d/issues/671
	// All b2Assert calls are replaced with return statements, and irrelevant computations are removed.

	if (3 > count || count > b2_maxPolygonVertices) {
		return false;
	}

	int32 n = b2Min(count, b2_maxPolygonVertices);

	// Perform welding and copy vertices into local buffer.
	b2Vec2 ps[b2_maxPolygonVertices];
	int32 tempCount = 0;
	for (int32 i = 0; i < n; ++i) {
		b2Vec2 v = vertices[i];

		bool unique = true;
		for (int32 j = 0; j < tempCount; ++j) {
			if (b2DistanceSquared(v, ps[j]) < ((0.5f * b2_linearSlop) * (0.5f * b2_linearSlop))) {
				unique = false;
				break;
			}
		}

		if (unique) {
			ps[tempCount++] = v;
		}
	}

	n = tempCount;
	if (n < 3) {
		// Polygon is degenerate.
		return false;
	}

	// Create the convex hull using the Gift wrapping algorithm
	// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

	// Find the right most point on the hull
	int32 i0 = 0;
	float x0 = ps[0].x;
	for (int32 i = 1; i < n; ++i) {
		float x = ps[i].x;
		if (x > x0 || (x == x0 && ps[i].y < ps[i0].y)) {
			i0 = i;
			x0 = x;
		}
	}

	int32 hull[b2_maxPolygonVertices];
	int32 m = 0;
	int32 ih = i0;

	for (;;) {
		if (m >= b2_maxPolygonVertices) {
			return false;
		}
		hull[m] = ih;

		int32 ie = 0;
		for (int32 j = 1; j < n; ++j) {
			if (ie == ih) {
				ie = j;
				continue;
			}

			b2Vec2 r = ps[ie] - ps[hull[m]];
			b2Vec2 v = ps[j] - ps[hull[m]];
			float c = b2Cross(r, v);
			if (c < 0.0f) {
				ie = j;
			}

			// Collinearity check
			if (c == 0.0f && v.LengthSquared() > r.LengthSquared()) {
				ie = j;
			}
		}

		++m;
		ih = ie;

		if (ie == i0) {
			break;
		}
	}

	if (m < 3) {
		// Polygon is degenerate.
		return false;
	}

	// Compute normals. Ensure the edges have non-zero length.
	for (int32 i = 0; i < m; ++i) {
		int32 i1 = i;
		int32 i2 = i + 1 < m ? i + 1 : 0;
		b2Vec2 edge = ps[hull[i2]] - ps[hull[i1]];
		if (edge.LengthSquared() <= b2_epsilon * b2_epsilon) {
			return false;
		}
	}
}

void Box2DPolygonShape::build_polygon() {
	// Ensure all points are counterclockwise
	Vector<Vector2> ccw_points = points;
	if (Geometry::is_polygon_clockwise(ccw_points)) {
		ccw_points.invert();
	}

	// Remove previous b2Shapes
	shape_vector.clear();

	//if (build_mode == BUILD_SOLIDS)
	{
#ifdef DEBUG_DECOMPOSE_BOX2D
		decomposed.clear();
#endif

		ERR_FAIL_COND_MSG(ccw_points.size() < 3, "Solid polygon must have N>2 points.");

		// Decompose concave into multiple convex
		Vector<Vector<Vector2> > decomp = Geometry::decompose_polygon_in_convex(ccw_points);

		// Cut convex into small N<=8 gons and create b2Shapes
		constexpr int N = b2_maxPolygonVertices;
		for (int i = 0; i < decomp.size(); i++) {
			const Vector<Vector2> *bigpoly = &decomp[i];

			b2Vec2 b2_pts[N];

			int count = N;
			for (int j = 0; j < bigpoly->size() - 2; j += (count - 2)) {
				Vector<Vector2> smallpoly;

				count = bigpoly->size() - j;
				if (count > N && count < N + 3)
					count = N - 2;
				else
					count = MIN(count, N);

				smallpoly.push_back((*bigpoly)[0]);
				//print_line((std::to_string(0)).c_str());
				for (int k = 1; k < count; k++) {
					smallpoly.push_back((*bigpoly)[j + k]);
					//print_line((std::to_string(j + k)).c_str());
				}

				// Create b2Shape
				b2PolygonShape shape;
				for (int k = 0; k < count; k++) {
					b2_pts[k] = gd_to_b2(smallpoly[k]);
				}

				if (likely(isPolygonValid(b2_pts, count))) {
					shape.Set(b2_pts, count);
#ifdef DEBUG_DECOMPOSE_BOX2D
					decomposed.push_back(smallpoly);
#endif
				} else {
					// Box2D thinks our polygon is degenerate. Abort!
					emit_changed();
					ERR_FAIL_MSG("Polygon is ill-formed.");
				}

				shape_vector.push_back(shape);
			}
		}
	}
	//else {
	//	// TODO
	//}

	emit_changed();
}

void Box2DPolygonShape::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_point_cloud", "points"), &Box2DPolygonShape::set_point_cloud);
	ClassDB::bind_method(D_METHOD("set_points", "points"), &Box2DPolygonShape::set_points);
	ClassDB::bind_method(D_METHOD("get_points"), &Box2DPolygonShape::get_points);

	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "points"), "set_points", "get_points");
}

bool Box2DPolygonShape::_edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const {
	const float factor = static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));

	b2CircleShape cursor;
	cursor.m_radius = p_tolerance / factor;

	Transform2D cursor_pos(0, p_point);

	for (int i = 0; i < shape_vector.size(); i++) {
		if (b2TestOverlap(&shape_vector[i], 0, &cursor, 0, gd_to_b2(Transform2D()), gd_to_b2(cursor_pos)))
			return true;
	}
	return false;
}

void Box2DPolygonShape::set_point_cloud(const Vector<Vector2> &p_points) {
	Vector<Point2> hull = Geometry::convex_hull_2d(p_points);
	ERR_FAIL_COND(hull.size() < 3);
	set_points(hull);
}

void Box2DPolygonShape::set_points(const Vector<Vector2> &p_points) {
	points = p_points;
	build_polygon();
}

Vector<Vector2> Box2DPolygonShape::get_points() const {
	return points;
}

void Box2DPolygonShape::draw(const RID &p_to_rid, const Color &p_color) {
	int vertex_count = points.size();
	for (int i = 0; i < vertex_count; i++) {
		Vector2 p = points[i];
		Vector2 n = points[(i + 1) % vertex_count];
		VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, p, n, Color(0.9, 0.2, 0.0, 0.8), 1);
	}

#if defined(TOOLS_ENABLED) && defined(DEBUG_DECOMPOSE_BOX2D)
	Color c(0.4, 0.9, 0.1);
	for (int i = 0; i < decomposed.size(); i++) {
		c.set_hsv(Math::fmod(c.get_h() + 0.738, 1), c.get_s(), c.get_v(), 0.5);
		Vector<Color> colors;
		colors.push_back(c);
		VisualServer::get_singleton()->canvas_item_add_polygon(p_to_rid, decomposed[i], colors);
	}
#else
	Vector<Color> colors;
	colors.push_back(p_color);
	VisualServer::get_singleton()->canvas_item_add_polygon(p_to_rid, points, colors);
#endif
}

Box2DPolygonShape::Box2DPolygonShape() :
		Box2DShape() {
}

Node2D *Box2DPolygonEditor::_get_node() const {
	return node;
}

void Box2DPolygonEditor::_set_node(Node *p_polygon) {
	node = Object::cast_to<Box2DFixture>(p_polygon);
}

Variant Box2DPolygonEditor::_get_polygon(int p_idx) const {
	if (node->get_shape().is_valid()) {
		Box2DPolygonShape *poly = dynamic_cast<Box2DPolygonShape *>(*node->get_shape());
		if (poly) {
			return poly->get_points();
		}
	}
	return Vector<Vector2>();
}

void Box2DPolygonEditor::_set_polygon(int p_idx, const Variant &p_polygon) const {
	if (node->get_shape().is_valid()) {
		Box2DPolygonShape *poly = dynamic_cast<Box2DPolygonShape *>(*node->get_shape());
		if (poly) {
			poly->set_points(p_polygon);
		}
	}
}

void Box2DPolygonEditor::_action_set_polygon(int p_idx, const Variant &p_previous, const Variant &p_polygon) {
	undo_redo->add_do_method(*node->get_shape(), "set_points", p_polygon);
	undo_redo->add_undo_method(*node->get_shape(), "set_points", p_previous);
}

Box2DPolygonEditor::Box2DPolygonEditor(EditorNode *p_editor) :
		AbstractPolygon2DEditor(p_editor), node(NULL), shape(NULL) {
}

bool Box2DPolygonEditorPlugin::handles(Object *p_object) const {
	Box2DFixture *node = Object::cast_to<Box2DFixture>(p_object);
	return node && node->get_shape().is_valid() && dynamic_cast<Box2DPolygonShape *>(*node->get_shape());
}

Box2DPolygonEditorPlugin::Box2DPolygonEditorPlugin(EditorNode *p_node) :
		AbstractPolygon2DEditorPlugin(p_node, memnew(Box2DPolygonEditor(p_node)), "Box2DFixture") {
}
