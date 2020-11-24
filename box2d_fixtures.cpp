#include "box2d_fixtures.h"

#include <core/engine.h>

void Box2DFixture::create_b2Fixture(b2Fixture *&p_fixture_out, const b2FixtureDef &p_def, const Transform2D &p_shape_xform) {
	// Transform shape with local transform
	b2FixtureDef transformedDef = b2FixtureDef(p_def);
	switch (p_def.shape->m_type) {
		case b2Shape::Type::e_circle: {
			b2CircleShape shp = b2CircleShape(*dynamic_cast<const b2CircleShape *>(p_def.shape));
			shp.m_p = gd_to_b2(p_shape_xform.xform(b2_to_gd(shp.m_p)));
			transformedDef.shape = &shp;
			p_fixture_out = body_node->body->CreateFixture(&transformedDef); // Write here because shp is in scope
		} break;
		case b2Shape::Type::e_edge: {
			b2EdgeShape shp = b2EdgeShape(*dynamic_cast<const b2EdgeShape *>(p_def.shape));
			shp.m_vertex0 = gd_to_b2(p_shape_xform.xform(b2_to_gd(shp.m_vertex0)));
			shp.m_vertex1 = gd_to_b2(p_shape_xform.xform(b2_to_gd(shp.m_vertex1)));
			shp.m_vertex2 = gd_to_b2(p_shape_xform.xform(b2_to_gd(shp.m_vertex2)));
			shp.m_vertex3 = gd_to_b2(p_shape_xform.xform(b2_to_gd(shp.m_vertex3)));
			transformedDef.shape = &shp;
			p_fixture_out = body_node->body->CreateFixture(&transformedDef); // Write here because shp is in scope
		} break;
		case b2Shape::Type::e_polygon: {
			b2PolygonShape shp = b2PolygonShape(*dynamic_cast<const b2PolygonShape *>(p_def.shape));
			for (int i = 0; i < shp.m_count; i++) {
				shp.m_vertices[i] = gd_to_b2(p_shape_xform.xform(b2_to_gd(shp.m_vertices[i])));
				shp.m_normals[i] = gd_to_b2(p_shape_xform.basis_xform(b2_to_gd(shp.m_normals[i])));
			}
			shp.m_centroid = gd_to_b2(p_shape_xform.xform(b2_to_gd(shp.m_centroid)));
			transformedDef.shape = &shp;
			p_fixture_out = body_node->body->CreateFixture(&transformedDef); // Write here because shp is in scope
		} break;
		case b2Shape::Type::e_chain: {
			// TODO
			ERR_FAIL_MSG("Not yet implemented");
			p_fixture_out = body_node->body->CreateFixture(&transformedDef); // Write here because shp is in scope
		} break;
		default: {
			ERR_FAIL();
		} break;
	}

	p_fixture_out->GetUserData().owner = this;
	body_node->update_mass();
}

bool Box2DFixture::create_b2() {
	if (fixtures.size() <= 0) {
		ERR_FAIL_COND_V(!body_node, false);
		ERR_FAIL_COND_V(!body_node->body, false);
		ERR_FAIL_COND_V(!shape.is_valid(), false);

		Box2DPolygonShape *polyshape = dynamic_cast<Box2DPolygonShape *>(shape.ptr());
		if (polyshape) {
			for (int i = 0; i < polyshape->shape_vector.size(); i++) {
				fixtureDef.shape = &polyshape->shape_vector[i];
				b2Fixture *fixture = NULL;
				create_b2Fixture(fixture, fixtureDef, get_transform());
				if (fixture)
					fixtures.push_back(fixture);
			}
		} else {
			ERR_FAIL_COND_V(!shape->shape, false);

			fixtureDef.shape = shape->shape;
			b2Fixture *fixture = NULL;
			create_b2Fixture(fixture, fixtureDef, get_transform());
			if (fixture)
				fixtures.push_back(fixture);
		}

		//print_line("fixture created");
		return true;
	}
	return false;
}

bool Box2DFixture::destroy_b2() {
	if (fixtures.size() > 0) {
		ERR_FAIL_COND_V(!body_node, false);
		if (body_node->body) {
			for (int i = 0; i < fixtures.size(); i++) {
				body_node->body->DestroyFixture(fixtures[i]);
			}
			fixtures.clear();
			//print_line("fixture destroyed");
		}
		return true;
	}
	return false;
}

void Box2DFixture::_shape_changed() {

	update_shape();
}

void Box2DFixture::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {

			Box2DPhysicsBody *new_body = Object::cast_to<Box2DPhysicsBody>(get_parent());

			// If new parent, recreate fixture
			if (body_node != new_body) {
				// Remove from previous parent
				if (body_node) {
					body_node->fixtures.erase(this);
					destroy_b2();
				}

				body_node = new_body;
				if (body_node) {
					body_node->fixtures.insert(this);
					if (body_node->body && shape.is_valid()) {
						create_b2();
					}
				}
			}

		} break;
		case NOTIFICATION_EXIT_TREE: {

			// Don't destroy fixture. It could be exiting/entering.
			// Fixture should be destroyed in destructor if node is being freed.

		} break;
		case NOTIFICATION_LOCAL_TRANSFORM_CHANGED: {

			update_shape();

		} break;
		case NOTIFICATION_DRAW: {

			if (!Engine::get_singleton()->is_editor_hint() && !get_tree()->is_debugging_collisions_hint()) {
				break;
			}

			Color draw_col = get_tree()->get_debug_collisions_color();
			if (shape.is_valid()) {
				shape->draw(get_canvas_item(), draw_col);
			}

		} break;
	}
}

void Box2DFixture::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_shape", "shape"), &Box2DFixture::set_shape);
	ClassDB::bind_method(D_METHOD("get_shape"), &Box2DFixture::get_shape);
	ClassDB::bind_method(D_METHOD("set_density", "density"), &Box2DFixture::set_density);
	ClassDB::bind_method(D_METHOD("get_density"), &Box2DFixture::get_density);
	ClassDB::bind_method(D_METHOD("set_friction", "friction"), &Box2DFixture::set_friction);
	ClassDB::bind_method(D_METHOD("get_friction"), &Box2DFixture::get_friction);
	ClassDB::bind_method(D_METHOD("set_restitution", "restitution"), &Box2DFixture::set_restitution);
	ClassDB::bind_method(D_METHOD("get_restitution"), &Box2DFixture::get_restitution);

	ClassDB::bind_method(D_METHOD("_shape_changed"), &Box2DFixture::_shape_changed);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "shape", PROPERTY_HINT_RESOURCE_TYPE, "Box2DShape"), "set_shape", "get_shape");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "density"), "set_density", "get_density");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "friction"), "set_friction", "get_friction");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "restitution"), "set_restitution", "get_restitution");
}

void Box2DFixture::on_parent_created(Node *) {
	//destroy_b2();
	//create_b2();
	WARN_PRINT("FIXTURE CREATED IN CALLBACK");
}

void Box2DFixture::update_shape() {
	if (body_node && body_node->body) {
		// If shape has changed, the fixture must be completely recreated
		destroy_b2();
		create_b2();
	}
	update();
	if (Engine::get_singleton()->is_editor_hint()) {
		update_configuration_warning();
	}
}

bool Box2DFixture::_edit_is_selected_on_click(const Point2 &p_point, double p_tolerance) const {
	if (!shape.is_valid())
		return false;

	return shape->_edit_is_selected_on_click(p_point, p_tolerance);
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

//Box2DFixture::FixtureType Box2DFixture::get_type() const {
//	if (fixture) {
//		return static_cast<Box2DFixture::FixtureType>(fixture->GetType());
//	} else {
//		return static_cast<Box2DFixture::FixtureType>(fixtureDef.shape->m_type);
//	}
//}

void Box2DFixture::set_shape(const Ref<Box2DShape> &p_shape) {
	if (shape.is_valid()) {
		shape->disconnect("changed", this, "_shape_changed");
	}
	shape = p_shape;

	if (shape.is_valid()) {
		shape->connect("changed", this, "_shape_changed");
	}

	update_shape();
}

Ref<Box2DShape> Box2DFixture::get_shape() {
	return shape;
}

void Box2DFixture::set_density(real_t p_density) {
	for (int i = 0; i < fixtures.size(); i++) {
		fixtures[i]->SetDensity(p_density);
		body_node->body->ResetMassData();
	}
	fixtureDef.density = p_density;
}

real_t Box2DFixture::get_density() const {
	return fixtureDef.density;
}

void Box2DFixture::set_friction(real_t p_friction) {
	for (int i = 0; i < fixtures.size(); i++) {
		fixtures[i]->SetFriction(p_friction);
	}
	fixtureDef.friction = p_friction;
}

real_t Box2DFixture::get_friction() const {
	return fixtureDef.friction;
}

void Box2DFixture::set_restitution(real_t p_restitution) {
	for (int i = 0; i < fixtures.size(); i++) {
		fixtures[i]->SetRestitution(p_restitution);
	}
	fixtureDef.restitution = p_restitution;
}

real_t Box2DFixture::get_restitution() const {
	return fixtureDef.restitution;
}

Box2DFixture::~Box2DFixture() {
	if (body_node) {
		destroy_b2();
	}
}
