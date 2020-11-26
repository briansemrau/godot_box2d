#include "box2d_shapes.h"

#include <core/project_settings.h>
#include <servers/visual_server.h>

#include "../../util/box2d_types_converter.h"
#include "../2d/box2d_fixtures.h"

#include <string> // TODO remove this. For debugging only

/**
* @author Brian Semrau
*/

void draw_arrow(const RID &p_to_rid, const Vector2 &start, const Vector2 &end, const Color &p_color, float p_width) {
	Vector2 norm = (end - start).normalized();
	VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, start, end, p_color, p_width);
	VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, end, end - norm.rotated(Math_PI * 0.17f) * 4.0f, p_color, p_width);
	VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, end, end - norm.rotated(-Math_PI * 0.17f) * 4.0f, p_color, p_width);
}

void Box2DShape::_bind_methods() {
	// Anything to bind?
}

bool Box2DShape::is_composite_shape() const {
	return false;
}

const Vector<const b2Shape *> Box2DShape::get_shapes() const {
	ERR_FAIL_V(Vector<const b2Shape *>());
}

bool Box2DShape::_edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const {
	const float factor = static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));

	b2CircleShape cursor;
	cursor.m_radius = p_tolerance / factor;

	Transform2D cursor_pos(0, p_point);

	return b2TestOverlap(get_shape(), 0, &cursor, 0, gd_to_b2(Transform2D()), gd_to_b2(cursor_pos));
}

void Box2DCircleShape::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &Box2DCircleShape::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &Box2DCircleShape::get_radius);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "radius", PROPERTY_HINT_EXP_RANGE, "0.5,16384,0.5"), "set_radius", "get_radius");
}

void Box2DCircleShape::set_radius(real_t p_radius) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	circleShape.m_radius = MAX(p_radius * factor, b2_linearSlop);
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

	int vertex_count = points.size();
	for (int i = 0; i < vertex_count; i++) {
		Vector2 p = points[i];
		Vector2 n = points[(i + 1) % vertex_count];
		VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, p, n, p_color, 1.0f);
	}

	Vector<Color> col;
	Color c(p_color);
	c.a *= 0.5;
	col.push_back(c);
	VisualServer::get_singleton()->canvas_item_add_polygon(p_to_rid, points, col);
}

Box2DCircleShape::Box2DCircleShape() {
	set_radius(25.0f);
}

void Box2DRectShape::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_size", "size"), &Box2DRectShape::set_size);
	ClassDB::bind_method(D_METHOD("get_size"), &Box2DRectShape::get_size);
	ClassDB::bind_method(D_METHOD("set_width", "width"), &Box2DRectShape::set_width);
	ClassDB::bind_method(D_METHOD("get_width"), &Box2DRectShape::get_width);
	ClassDB::bind_method(D_METHOD("set_height", "height"), &Box2DRectShape::set_height);
	ClassDB::bind_method(D_METHOD("get_height"), &Box2DRectShape::get_height);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "size"), "set_size", "get_size");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "width", PROPERTY_HINT_EXP_RANGE, "0.5,16384,0.5"), "set_width", "get_width");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "height", PROPERTY_HINT_EXP_RANGE, "0.5,16384,0.5"), "set_height", "get_height");
}

void Box2DRectShape::set_size(const Vector2 &p_size) {
	set_width(p_size.width);
	set_height(p_size.height);
}

Vector2 Box2DRectShape::get_size() const {
	return Vector2(width, height);
}

void Box2DRectShape::set_width(real_t p_width) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	width = MAX(p_width * factor, b2_linearSlop) / factor;
	shape.SetAsBox(width * factor * 0.5, height * factor * 0.5);
	emit_changed();
}

real_t Box2DRectShape::get_width() const {
	return width;
}

void Box2DRectShape::set_height(real_t p_height) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	height = MAX(p_height * factor, b2_linearSlop) / factor;
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

	int vertex_count = points.size();
	for (int i = 0; i < vertex_count; i++) {
		Vector2 p = points[i];
		Vector2 n = points[(i + 1) % vertex_count];
		VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, p, n, p_color, 1.0f);
	}

	Vector<Color> col;
	Color c(p_color);
	c.a *= 0.5;
	col.push_back(c);
	VisualServer::get_singleton()->canvas_item_add_polygon(p_to_rid, points, col);
}

Box2DRectShape::Box2DRectShape() :
		width(50.0f), height(50.0f) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	shape.SetAsBox(width * factor * 0.5, height * factor * 0.5);
}

void Box2DSegmentShape::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_a", "a"), &Box2DSegmentShape::set_a);
	ClassDB::bind_method(D_METHOD("get_a"), &Box2DSegmentShape::get_a);
	ClassDB::bind_method(D_METHOD("set_b", "b"), &Box2DSegmentShape::set_b);
	ClassDB::bind_method(D_METHOD("get_b"), &Box2DSegmentShape::get_b);
	ClassDB::bind_method(D_METHOD("set_a_adjacent", "a_adjacent"), &Box2DSegmentShape::set_a_adjacent);
	ClassDB::bind_method(D_METHOD("get_a_adjacent"), &Box2DSegmentShape::get_a_adjacent);
	ClassDB::bind_method(D_METHOD("set_b_adjacent", "b_adjacent"), &Box2DSegmentShape::set_b_adjacent);
	ClassDB::bind_method(D_METHOD("get_b_adjacent"), &Box2DSegmentShape::get_b_adjacent);
	ClassDB::bind_method(D_METHOD("set_one_sided", "one_sided"), &Box2DSegmentShape::set_one_sided);
	ClassDB::bind_method(D_METHOD("is_one_sided"), &Box2DSegmentShape::is_one_sided);

	ClassDB::bind_method(D_METHOD("set_as_one_sided", "p_a_adj", "p_a", "p_b", "p_b_adj"), &Box2DSegmentShape::set_as_one_sided);
	ClassDB::bind_method(D_METHOD("set_as_two_sided", "p_a", "p_b"), &Box2DSegmentShape::set_as_two_sided);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "a"), "set_a", "get_a");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "b"), "set_b", "get_b");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "a_adjacent"), "set_a_adjacent", "get_a_adjacent");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "b_adjacent"), "set_b_adjacent", "get_b_adjacent");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "one_sided"), "set_one_sided", "is_one_sided");
}

void Box2DSegmentShape::set_a(const Vector2 &p_a) {
	shape.m_vertex1 = gd_to_b2(p_a);
	emit_changed();
}

Vector2 Box2DSegmentShape::get_a() const {
	return b2_to_gd(shape.m_vertex1);
}

void Box2DSegmentShape::set_b(const Vector2 &p_b) {
	shape.m_vertex2 = gd_to_b2(p_b);
	emit_changed();
}

Vector2 Box2DSegmentShape::get_b() const {
	return b2_to_gd(shape.m_vertex2);
}

void Box2DSegmentShape::set_a_adjacent(const Vector2 &p_a_adj) {
	if (!is_one_sided())
		WARN_PRINT("Modifying adjacent vertices on a two-sided segment shape has no effect.")
	shape.m_vertex0 = gd_to_b2(p_a_adj);
	emit_changed();
}

Vector2 Box2DSegmentShape::get_a_adjacent() const {
	return b2_to_gd(shape.m_vertex0);
}

void Box2DSegmentShape::set_b_adjacent(const Vector2 &p_b_adj) {
	if (!is_one_sided())
		WARN_PRINT("Modifying adjacent vertices on a two-sided segment shape has no effect.")
	shape.m_vertex3 = gd_to_b2(p_b_adj);
	emit_changed();
}

Vector2 Box2DSegmentShape::get_b_adjacent() const {
	return b2_to_gd(shape.m_vertex3);
}

void Box2DSegmentShape::set_one_sided(bool p_one_sided) {
	shape.m_oneSided = p_one_sided;
	emit_changed();
}

bool Box2DSegmentShape::is_one_sided() const {
	return shape.m_oneSided;
}

void Box2DSegmentShape::set_as_one_sided(const Vector2 &p_a_adj, const Vector2 &p_a, const Vector2 &p_b, const Vector2 &p_b_adj) {
	shape.SetOneSided(gd_to_b2(p_a_adj), gd_to_b2(p_a), gd_to_b2(p_b), gd_to_b2(p_b_adj));
	emit_changed();
}

void Box2DSegmentShape::set_as_two_sided(const Vector2 &p_a, const Vector2 &p_b) {
	shape.SetTwoSided(gd_to_b2(p_a), gd_to_b2(p_b));
	emit_changed();
}

void Box2DSegmentShape::draw(const RID &p_to_rid, const Color &p_color) {
	const Vector2 a = get_a();
	const Vector2 b = get_b();

	VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, a, b, p_color, 2.0f);

	if (is_one_sided()) {
		Color c_adj = p_color;
		c_adj.set_hsv(c_adj.get_h(), c_adj.get_s() * 0.5, c_adj.get_v(), c_adj.a * 0.5);

		VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, get_a_adjacent(), a, c_adj, 1.0f);
		VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, get_b_adjacent(), b, c_adj, 1.0f);

		// Draw arrow in normal direction
		Vector2 midpoint = (a + b) / 2.0;
		Vector2 norm = (b - a).normalized().rotated(Math_PI * -0.5f);
		Vector2 tip = midpoint + norm * 10.0f;
		draw_arrow(p_to_rid, midpoint, tip, p_color, 1.0f);
	}
}

Box2DSegmentShape::Box2DSegmentShape() {
	shape.SetTwoSided(gd_to_b2(Vector2(-25, 0)), gd_to_b2(Vector2(25, 0)));
	shape.m_vertex0 = gd_to_b2(Vector2(-50, 0));
	shape.m_vertex3 = gd_to_b2(Vector2(50, 0));
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

	return true;
}

void Box2DPolygonShape::build_polygon() {
	// Remove previous b2Shapes
	polygon_shape_vector.clear();

	memdelete_notnull(chain_shape);

	// can he do it? yes he can!
	switch (build_mode) {
		case Box2DPolygonShape::BUILD_SOLIDS: {
#ifdef DEBUG_DECOMPOSE_BOX2D
			decomposed.clear();
#endif
			ERR_FAIL_COND_MSG(points.size() < 3, "Solid polygon must have N>2 points.");

			// Ensure all points are counterclockwise
			Vector<Vector2> ccw_points = points;
			if (Geometry::is_polygon_clockwise(ccw_points)) {
				ccw_points.invert();
			}

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

					polygon_shape_vector.push_back(shape);
				}
			}
		} break;

		case Box2DPolygonShape::BUILD_SEGMENTS: {
			ERR_FAIL_COND_MSG(points.size() < 2, "Segment polygon must have N>1 points.");

			Vector<Vector2> ordered_points = points;
			if (invert_order) {
				ordered_points.invert();
			}

			// Convert vertices
			const int n = ordered_points.size();
			b2Vec2 *b2Vertices = static_cast<b2Vec2 *>(memalloc(n * sizeof(b2Vec2)));

			for (int i = 0; i < n; i++) {
				b2Vertices[i] = gd_to_b2(ordered_points[i]);
			}

			chain_shape = memnew(b2ChainShape);
			chain_shape->CreateLoop(b2Vertices, n);
			memfree(b2Vertices);
		} break;

		case Box2DPolygonShape::BUILD_OPEN_SEGMENTS: {
			ERR_FAIL_COND_MSG(points.size() < 4, "Open segment polygon must have N>3 points.");

			Vector<Vector2> ordered_points = points;
			if (invert_order) {
				ordered_points.invert();
			}

			// Convert vertices
			const int n = ordered_points.size();
			b2Vec2 *b2Vertices = static_cast<b2Vec2 *>(memalloc(n * sizeof(b2Vec2)));

			for (int i = 0; i < n; i++) {
				b2Vertices[i] = gd_to_b2(ordered_points[i]);
			}

			chain_shape = memnew(b2ChainShape);
			chain_shape->CreateChain(b2Vertices + 1, n - 2, b2Vertices[0], b2Vertices[n - 1]);
			memfree(b2Vertices);
		} break;

		case Box2DPolygonShape::BUILD_OVERLAPPING_SOLIDS: {
			emit_changed();
			ERR_FAIL_MSG("Not yet implemented.");
		} break;

		default: {
			emit_changed();
			ERR_FAIL();
		} break;
	}

	emit_changed();
}

void Box2DPolygonShape::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_point_cloud", "points"), &Box2DPolygonShape::set_point_cloud);
	ClassDB::bind_method(D_METHOD("set_points", "points"), &Box2DPolygonShape::set_points);
	ClassDB::bind_method(D_METHOD("get_points"), &Box2DPolygonShape::get_points);
	ClassDB::bind_method(D_METHOD("set_invert_order", "invert_order"), &Box2DPolygonShape::set_invert_order);
	ClassDB::bind_method(D_METHOD("get_invert_order"), &Box2DPolygonShape::get_invert_order);
	ClassDB::bind_method(D_METHOD("set_build_mode", "build_mode"), &Box2DPolygonShape::set_build_mode);
	ClassDB::bind_method(D_METHOD("get_build_mode"), &Box2DPolygonShape::get_build_mode);

	ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "points"), "set_points", "get_points");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "invert_order"), "set_invert_order", "get_invert_order");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "build_mode", PROPERTY_HINT_ENUM, "Solids,Segments,Open Segments,Overlapping Solids"), "set_build_mode", "get_build_mode");

	BIND_ENUM_CONSTANT(BUILD_SOLIDS);
	BIND_ENUM_CONSTANT(BUILD_SEGMENTS);
	BIND_ENUM_CONSTANT(BUILD_OPEN_SEGMENTS);
	BIND_ENUM_CONSTANT(BUILD_OVERLAPPING_SOLIDS);
}

bool Box2DPolygonShape::is_composite_shape() const {
	return build_mode == BUILD_SOLIDS || build_mode == BUILD_OVERLAPPING_SOLIDS;
}

const Vector<const b2Shape *> Box2DPolygonShape::get_shapes() const {
	Vector<const b2Shape *> out;
	out.resize(polygon_shape_vector.size());
	for (int i = 0; i < polygon_shape_vector.size(); i++) {
		out.set(i, &(polygon_shape_vector[i]));
	}
	return out;
}

bool Box2DPolygonShape::_edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const {
	const float factor = static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));

	b2CircleShape cursor;
	cursor.m_radius = p_tolerance / factor;

	Transform2D cursor_pos(0, p_point);

	if (build_mode == BUILD_SOLIDS || build_mode == BUILD_OVERLAPPING_SOLIDS) {
		for (int i = 0; i < polygon_shape_vector.size(); i++) {
			if (b2TestOverlap(&polygon_shape_vector[i], 0, &cursor, 0, gd_to_b2(Transform2D()), gd_to_b2(cursor_pos))) {
				return true;
			}
		}
		return false;
	} else {
		if (!chain_shape) {
			return false;
		}
		for (int i = 0; i < chain_shape->m_count; i++) {
			if (b2TestOverlap(chain_shape, i, &cursor, 0, gd_to_b2(Transform2D()), gd_to_b2(cursor_pos))) {
				return true;
			}
		}
		return false;
	}
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

void Box2DPolygonShape::set_build_mode(BuildMode p_mode) {
	if (build_mode != p_mode) {
		build_mode = p_mode;
		build_polygon();
	}
}

Box2DPolygonShape::BuildMode Box2DPolygonShape::get_build_mode() const {
	return build_mode;
}

void Box2DPolygonShape::set_invert_order(bool p_inverted) {
	if (invert_order != p_inverted) {
		invert_order = p_inverted;
		build_polygon();
	}
}

bool Box2DPolygonShape::get_invert_order() const {
	return invert_order;
}

void Box2DPolygonShape::draw(const RID &p_to_rid, const Color &p_color) {
	if (build_mode == BUILD_SOLIDS || build_mode == BUILD_OVERLAPPING_SOLIDS) {

		int vertex_count = points.size();
		for (int i = 0; i < vertex_count; i++) {
			Vector2 p = points[i];
			Vector2 n = points[(i + 1) % vertex_count];
			VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, p, n, p_color, 1.0f);
		}

		Color c(p_color);
		c.a *= 0.5f;

#if defined(TOOLS_ENABLED) && defined(DEBUG_DECOMPOSE_BOX2D)
		for (int i = 0; i < decomposed.size(); i++) {
			// Math::fmod(c.get_h() + 0.738, 1)
			c.set_hsv(c.get_h(), c.get_s(), Math::fmod(c.get_v() - 0.7f + 0.221f, 0.3f) + 0.7, 0.5f);
			Vector<Color> colors;
			colors.push_back(c);
			VisualServer::get_singleton()->canvas_item_add_polygon(p_to_rid, decomposed[i], colors);
		}
#else
		Vector<Color> colors;
		colors.push_back(c);
		VisualServer::get_singleton()->canvas_item_add_polygon(p_to_rid, points, colors);
#endif

	} else if (build_mode == BUILD_SEGMENTS) {

		int vertex_count = points.size();
		for (int i = 0; i < vertex_count; i++) {
			Vector2 p = points[i];
			Vector2 n = points[(i + 1) % vertex_count];
			VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, p, n, p_color, 2.0f);

			// Draw normal arrow
			Vector2 midpoint = (p + n) / 2.0;
			Vector2 norm = (n - p).normalized().rotated(Math_PI * -0.5f);
			if (invert_order)
				norm = -norm;
			Vector2 tip = midpoint + norm * 10.0f;
			draw_arrow(p_to_rid, midpoint, tip, p_color, 1.0f);
		}

	} else if (build_mode == BUILD_OPEN_SEGMENTS) {

		int vertex_count = points.size();
		for (int i = 0; i < vertex_count - 1; i++) {
			Vector2 p = points[i];
			Vector2 n = points[i + 1];

			Color c_tmp = p_color;
			float width = 2.0f;
			if (i == 0 || i == vertex_count - 2) {
				c_tmp.set_hsv(c_tmp.get_h(), c_tmp.get_s() * 0.5, c_tmp.get_v(), c_tmp.a * 0.5);
				width = 1.0f;
			}
			VisualServer::get_singleton()->canvas_item_add_line(p_to_rid, p, n, c_tmp, width);

			if (!(i == 0 || i == vertex_count - 2)) {
				// Draw arrow in normal direction
				Vector2 midpoint = (p + n) / 2.0;
				Vector2 norm = (n - p).normalized().rotated(Math_PI * -0.5f);
				if (invert_order)
					norm = -norm;
				Vector2 tip = midpoint + norm * 10.0f;
				draw_arrow(p_to_rid, midpoint, tip, p_color, 1.0f);
			}
		}
	}
}

Box2DPolygonShape::Box2DPolygonShape() :
		build_mode(BuildMode::BUILD_SOLIDS),
		chain_shape(NULL),
		invert_order(false) {}

Box2DPolygonShape::~Box2DPolygonShape() {
	memdelete_notnull(chain_shape);
}
