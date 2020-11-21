#include "box2d_fixtures.h"

#include <core/engine.h>

bool Box2DFixture::create_b2Fixture() {
	// TODO should this be abstracted and handled by each fixture node type?
	//      Probably not, because Polygon/Rect nodes are both e_polygon
	if (!fixture) {
		ERR_FAIL_COND_V(!parent, false);
		ERR_FAIL_COND_V(!parent->body, false);

		// Transform shape with local transform
		b2FixtureDef transformedDef = b2FixtureDef(fixtureDef);
		switch (fixtureDef.shape->m_type) {
			case b2Shape::Type::e_circle: {
				b2CircleShape shp = b2CircleShape(*dynamic_cast<const b2CircleShape *>(fixtureDef.shape));
				shp.m_p = gd_to_b2(get_transform().xform(b2_to_gd(shp.m_p)));
				transformedDef.shape = &shp;
				fixture = parent->body->CreateFixture(&transformedDef); // Write here because shp is in scope
			} break;
			case b2Shape::Type::e_edge: {
				b2EdgeShape shp = b2EdgeShape(*dynamic_cast<const b2EdgeShape *>(fixtureDef.shape));
				shp.m_vertex0 = gd_to_b2(get_transform().xform(b2_to_gd(shp.m_vertex0)));
				shp.m_vertex1 = gd_to_b2(get_transform().xform(b2_to_gd(shp.m_vertex1)));
				shp.m_vertex2 = gd_to_b2(get_transform().xform(b2_to_gd(shp.m_vertex2)));
				shp.m_vertex3 = gd_to_b2(get_transform().xform(b2_to_gd(shp.m_vertex3)));
				transformedDef.shape = &shp;
				fixture = parent->body->CreateFixture(&transformedDef); // Write here because shp is in scope
			} break;
			case b2Shape::Type::e_polygon: {
				b2PolygonShape shp = b2PolygonShape(*dynamic_cast<const b2PolygonShape *>(fixtureDef.shape));
				for (int i = 0; i < shp.m_count; i++) {
					shp.m_vertices[i] = gd_to_b2(get_transform().xform(b2_to_gd(shp.m_vertices[i])));
					shp.m_normals[i] = gd_to_b2(get_transform().basis_xform(b2_to_gd(shp.m_normals[i])));
				}
				shp.m_centroid = gd_to_b2(get_transform().xform(b2_to_gd(shp.m_centroid)));
				transformedDef.shape = &shp;
				fixture = parent->body->CreateFixture(&transformedDef); // Write here because shp is in scope
			} break;
			case b2Shape::Type::e_chain: {
				// TODO
				ERR_FAIL_V_MSG(false, "Not yet implemented");
				fixture = parent->body->CreateFixture(&transformedDef); // Write here because shp is in scope
			} break;
			default: {
				ERR_FAIL_V(false);
			} break;
		}

		fixture->GetUserData().owner = this;
		print_line("fixture created");
		return true;
	}
	return false;
}

bool Box2DFixture::destroy_b2Fixture() {
	if (fixture) {
		ERR_FAIL_COND_V(!parent, false);
		if (parent->body) {
			parent->body->DestroyFixture(fixture);
			print_line("fixture destroyed");
		}
		fixture = NULL;
		return true;
	}
	return false;
}

void Box2DFixture::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PARENTED: {
			parent = Object::cast_to<Box2DPhysicsBody>(get_parent());
			if (parent) {
				parent->fixtures.insert(this);
			}
		} break;
		case NOTIFICATION_LOCAL_TRANSFORM_CHANGED: {
			update_shape();
		} break;
		case NOTIFICATION_UNPARENTED: {
			destroy_b2Fixture();
			if (parent) {
				parent->fixtures.erase(this);
			}
			parent = NULL;
		} break;
		case NOTIFICATION_DRAW: {
			if (!Engine::get_singleton()->is_editor_hint() && !get_tree()->is_debugging_collisions_hint()) {
				break;
			}

			Color draw_col = get_tree()->get_debug_collisions_color();
			debug_draw(get_canvas_item(), draw_col);
		} break;
	}
}

void Box2DFixture::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_density", "density"), &Box2DFixture::set_density);
	ClassDB::bind_method(D_METHOD("get_density"), &Box2DFixture::get_density);
	ClassDB::bind_method(D_METHOD("set_friction", "friction"), &Box2DFixture::set_friction);
	ClassDB::bind_method(D_METHOD("get_friction"), &Box2DFixture::get_friction);
	ClassDB::bind_method(D_METHOD("set_restitution", "restitution"), &Box2DFixture::set_restitution);
	ClassDB::bind_method(D_METHOD("get_restitution"), &Box2DFixture::get_restitution);
	//ClassDB::bind_method(D_METHOD("set_", ""), &Box2DFixture::set_);
	//ClassDB::bind_method(D_METHOD("get_"), &Box2DFixture::get_);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "density"), "set_density", "get_density");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "friction"), "set_friction", "get_friction");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "restitution"), "set_restitution", "get_restitution");
	//ADD_PROPERTY(PropertyInfo(Variant::BOOL, ""), "set_", "get_");
}

void Box2DFixture::on_parent_created(Node *) {
	create_b2Fixture();
}

void Box2DFixture::update_shape() {
	if (parent && parent->body) {
		// If shape has changed, the fixture must be completely recreated
		destroy_b2Fixture();
		create_b2Fixture();
	}
	update();
	update_configuration_warning();
}

bool Box2DFixture::_edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const {
	const float factor = static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));

	b2CircleShape cursor;
	cursor.m_radius = p_tolerance / factor;

	Transform2D cursor_pos(0, p_point);

	return b2TestOverlap(fixtureDef.shape, 0, &cursor, 0, gd_to_b2(Transform2D()), gd_to_b2(cursor_pos));
}

String Box2DFixture::get_configuration_warning() const {
	String warning = Node2D::get_configuration_warning();

	if (!Object::cast_to<Box2DPhysicsBody>(get_parent())) {
		if (warning != String()) {
			warning += "\n\n";
		}
		warning += TTR("Box2DFixture subtypes only serve to provide collision fixtures to a Box2DPhysicsBody node. Please use it as a child of Box2DPhysicsBody to give it collision.");
	}

	return warning;
}

Box2DFixture::FixtureType Box2DFixture::get_type() const {
	if (fixture) {
		return static_cast<Box2DFixture::FixtureType>(fixture->GetType());
	} else {
		return static_cast<Box2DFixture::FixtureType>(fixtureDef.shape->m_type);
	}
}

void Box2DFixture::set_density(real_t p_density) {
	if (fixture) {
		fixture->SetDensity(p_density);
		parent->body->ResetMassData();
	}
	fixtureDef.density = p_density;
}

real_t Box2DFixture::get_density() const {
	return fixtureDef.density;
}

void Box2DFixture::set_friction(real_t p_friction) {
	if (fixture) {
		fixture->SetFriction(p_friction);
	}
	fixtureDef.friction = p_friction;
}

real_t Box2DFixture::get_friction() const {
	return fixtureDef.friction;
}

void Box2DFixture::set_restitution(real_t p_restitution) {
	if (fixture) {
		fixture->SetRestitution(p_restitution);
	}
	fixtureDef.restitution = p_restitution;
}

real_t Box2DFixture::get_restitution() const {
	return fixtureDef.restitution;
}

void Box2DCircleFixture::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_radius", "radius"), &Box2DCircleFixture::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &Box2DCircleFixture::get_radius);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "radius"), "set_radius", "get_radius");
}

void Box2DCircleFixture::debug_draw(RID p_to_rid, Color p_color) {
	// Same as in Godot's CircleShape2D::draw
	Vector<Vector2> points;
	for (int i = 0; i < 24; i++) {
		points.push_back(Vector2(Math::cos(i * Math_PI * 2 / 24.0), Math::sin(i * Math_PI * 2 / 24.0)) * get_radius());
	}

	Vector<Color> col;
	col.push_back(p_color);
	VisualServer::get_singleton()->canvas_item_add_polygon(p_to_rid, points, col);
}

void Box2DCircleFixture::set_radius(real_t p_radius) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	shape.m_radius = p_radius * factor;
	update_shape();
}

real_t Box2DCircleFixture::get_radius() const {
	const float factor = static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	return shape.m_radius * factor;
}

Box2DCircleFixture::Box2DCircleFixture() :
		Box2DFixture() {
	fixtureDef.shape = &shape;
}

void Box2DRectFixture::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_width", "width"), &Box2DRectFixture::set_width);
	ClassDB::bind_method(D_METHOD("get_width"), &Box2DRectFixture::get_width);
	ClassDB::bind_method(D_METHOD("set_height", "height"), &Box2DRectFixture::set_height);
	ClassDB::bind_method(D_METHOD("get_height"), &Box2DRectFixture::get_height);

	ADD_PROPERTY(PropertyInfo(Variant::REAL, "width"), "set_width", "get_width");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "height"), "set_height", "get_height");
}

void Box2DRectFixture::debug_draw(RID p_to_rid, Color p_color) {
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

void Box2DRectFixture::set_width(real_t p_width) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	width = p_width;
	shape.SetAsBox(width * factor * 0.5, height * factor * 0.5);
	update_shape();
}

real_t Box2DRectFixture::get_width() const {
	return width;
}

void Box2DRectFixture::set_height(real_t p_height) {
	const float factor = 1.0f / static_cast<float>(GLOBAL_GET("physics/2d/box2d_conversion_factor"));
	height = p_height;
	shape.SetAsBox(width * factor * 0.5, height * factor * 0.5);
	update_shape();
}

real_t Box2DRectFixture::get_height() const {
	return height;
}

Box2DRectFixture::Box2DRectFixture() :
		Box2DFixture(),
		width(1.0f),
		height(1.0f) {
	fixtureDef.shape = &shape;
	// Set shape arbitrarily to avoid Box2D errors
	shape.SetAsBox(1.0f, 1.0f);
}
