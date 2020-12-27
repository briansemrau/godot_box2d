#include "box2d_area.h"

#include "servers/audio_server.h"

#include "box2d_fixtures.h"
#include "box2d_physics_body.h"

/**
* @author Brian Semrau
*/

void Box2DArea::_on_object_entered(Box2DCollisionObject *p_object) {
	Box2DPhysicsBody *body = dynamic_cast<Box2DPhysicsBody *>(p_object);
	const Box2DArea *area = dynamic_cast<const Box2DArea *>(p_object);

	if (body) {
		if (get_space_override_mode() != Box2DArea::SpaceOverride::SPACE_OVERRIDE_DISABLED) {
			body->_add_area(this);
		}
	}

	if (monitoring) {
		if (body) {
			emit_signal("body_entered", body);
		} else if (area && area->is_monitorable()) {
			emit_signal("area_entered", area);
		}
	}
}

void Box2DArea::_on_object_exited(Box2DCollisionObject *p_object) {
	Box2DPhysicsBody *body = dynamic_cast<Box2DPhysicsBody *>(p_object);
	const Box2DArea *area = dynamic_cast<const Box2DArea *>(p_object);

	if (body) {
		body->_remove_area(this);
	}

	if (monitoring) {
		if (body) {
			emit_signal("body_exited", body);
		} else if (area && area->is_monitorable()) {
			emit_signal("area_exited", area);
		}
	}
}

void Box2DArea::_on_fixture_entered(Box2DFixture *p_fixture) {
	const Box2DPhysicsBody *body = dynamic_cast<const Box2DPhysicsBody *>(p_fixture->_get_owner_node());
	const Box2DArea *area = dynamic_cast<const Box2DArea *>(p_fixture->_get_owner_node());
	if (body) {
		emit_signal("body_fixture_entered", p_fixture);
	} else if (area && area->is_monitorable()) {
		emit_signal("area_fixture_entered", p_fixture);
	}
}

void Box2DArea::_on_fixture_exited(Box2DFixture *p_fixture) {
	const Box2DPhysicsBody *body = dynamic_cast<const Box2DPhysicsBody *>(p_fixture->_get_owner_node());
	const Box2DArea *area = dynamic_cast<const Box2DArea *>(p_fixture->_get_owner_node());
	if (body) {
		emit_signal("body_fixture_exited", p_fixture);
	} else if (area && area->is_monitorable()) {
		emit_signal("area_fixture_exited", p_fixture);
	}
}

void Box2DArea::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_space_override_mode", "space_override_mode"), &Box2DArea::set_space_override_mode);
	ClassDB::bind_method(D_METHOD("get_space_override_mode"), &Box2DArea::get_space_override_mode);

	ClassDB::bind_method(D_METHOD("set_gravity_is_point", "enable"), &Box2DArea::set_gravity_is_point);
	ClassDB::bind_method(D_METHOD("is_gravity_a_point"), &Box2DArea::is_gravity_a_point);

	ClassDB::bind_method(D_METHOD("set_gravity_distance_scale", "distance_scale"), &Box2DArea::set_gravity_distance_scale);
	ClassDB::bind_method(D_METHOD("get_gravity_distance_scale"), &Box2DArea::get_gravity_distance_scale);

	ClassDB::bind_method(D_METHOD("set_gravity_vector", "vector"), &Box2DArea::set_gravity_vector);
	ClassDB::bind_method(D_METHOD("get_gravity_vector"), &Box2DArea::get_gravity_vector);

	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &Box2DArea::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &Box2DArea::get_gravity);

	ClassDB::bind_method(D_METHOD("set_linear_damp", "linear_damp"), &Box2DArea::set_linear_damp);
	ClassDB::bind_method(D_METHOD("get_linear_damp"), &Box2DArea::get_linear_damp);

	ClassDB::bind_method(D_METHOD("set_angular_damp", "angular_damp"), &Box2DArea::set_angular_damp);
	ClassDB::bind_method(D_METHOD("get_angular_damp"), &Box2DArea::get_angular_damp);

	ClassDB::bind_method(D_METHOD("set_priority", "priority"), &Box2DArea::set_priority);
	ClassDB::bind_method(D_METHOD("get_priority"), &Box2DArea::get_priority);

	//ClassDB::bind_method(D_METHOD("set_collision_mask_bit", "bit", "value"), &Box2DArea::set_collision_mask_bit);
	//ClassDB::bind_method(D_METHOD("get_collision_mask_bit", "bit"), &Box2DArea::get_collision_mask_bit);

	//ClassDB::bind_method(D_METHOD("set_collision_layer_bit", "bit", "value"), &Box2DArea::set_collision_layer_bit);
	//ClassDB::bind_method(D_METHOD("get_collision_layer_bit", "bit"), &Box2DArea::get_collision_layer_bit);

	ClassDB::bind_method(D_METHOD("set_monitoring", "enable"), &Box2DArea::set_monitoring);
	ClassDB::bind_method(D_METHOD("is_monitoring"), &Box2DArea::is_monitoring);

	ClassDB::bind_method(D_METHOD("set_monitorable", "enable"), &Box2DArea::set_monitorable);
	ClassDB::bind_method(D_METHOD("is_monitorable"), &Box2DArea::is_monitorable);

	// TODO
	ClassDB::bind_method(D_METHOD("get_overlapping_bodies"), &Box2DArea::get_overlapping_bodies);
	ClassDB::bind_method(D_METHOD("get_overlapping_areas"), &Box2DArea::get_overlapping_areas);

	ClassDB::bind_method(D_METHOD("overlaps_body", "body"), &Box2DArea::overlaps_body);
	ClassDB::bind_method(D_METHOD("overlaps_area", "area"), &Box2DArea::overlaps_area);

	ClassDB::bind_method(D_METHOD("set_audio_bus_name", "name"), &Box2DArea::set_audio_bus_name);
	ClassDB::bind_method(D_METHOD("get_audio_bus_name"), &Box2DArea::get_audio_bus_name);

	ClassDB::bind_method(D_METHOD("set_audio_bus_override", "enable"), &Box2DArea::set_audio_bus_override);
	ClassDB::bind_method(D_METHOD("is_overriding_audio_bus"), &Box2DArea::is_overriding_audio_bus);

	ADD_SIGNAL(MethodInfo("body_fixture_entered", PropertyInfo(Variant::OBJECT, "fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture"), PropertyInfo(Variant::OBJECT, "local_fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture")));
	ADD_SIGNAL(MethodInfo("body_fixture_exited", PropertyInfo(Variant::OBJECT, "fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture"), PropertyInfo(Variant::OBJECT, "local_fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture")));
	ADD_SIGNAL(MethodInfo("body_entered", PropertyInfo(Variant::OBJECT, "body", PROPERTY_HINT_RESOURCE_TYPE, "Box2DPhysicsBody")));
	ADD_SIGNAL(MethodInfo("body_exited", PropertyInfo(Variant::OBJECT, "body", PROPERTY_HINT_RESOURCE_TYPE, "Box2DPhysicsBody")));

	ADD_SIGNAL(MethodInfo("area_fixture_entered", PropertyInfo(Variant::OBJECT, "fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture"), PropertyInfo(Variant::OBJECT, "local_fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture")));
	ADD_SIGNAL(MethodInfo("area_fixture_exited", PropertyInfo(Variant::OBJECT, "fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture"), PropertyInfo(Variant::OBJECT, "local_fixture", PROPERTY_HINT_RESOURCE_TYPE, "Box2DFixture")));
	ADD_SIGNAL(MethodInfo("area_entered", PropertyInfo(Variant::OBJECT, "area", PROPERTY_HINT_RESOURCE_TYPE, "Box2DArea")));
	ADD_SIGNAL(MethodInfo("area_exited", PropertyInfo(Variant::OBJECT, "area", PROPERTY_HINT_RESOURCE_TYPE, "Box2DArea")));

	ADD_PROPERTY(PropertyInfo(Variant::INT, "space_override", PROPERTY_HINT_ENUM, "Disabled,Combine,Combine-Replace,Replace,Replace-Combine"), "set_space_override_mode", "get_space_override_mode");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "gravity_point"), "set_gravity_is_point", "is_gravity_a_point");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity_distance_scale", PROPERTY_HINT_EXP_RANGE, "0,1024,0.001,or_greater"), "set_gravity_distance_scale", "get_gravity_distance_scale");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "gravity_vec"), "set_gravity_vector", "get_gravity_vector");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity", PROPERTY_HINT_RANGE, "-1024,1024,0.001"), "set_gravity", "get_gravity");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "linear_damp", PROPERTY_HINT_RANGE, "0,100,0.001,or_greater"), "set_linear_damp", "get_linear_damp");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_damp", PROPERTY_HINT_RANGE, "0,100,0.001,or_greater"), "set_angular_damp", "get_angular_damp");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "priority", PROPERTY_HINT_RANGE, "0,128,1"), "set_priority", "get_priority");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "monitoring"), "set_monitoring", "is_monitoring");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "monitorable"), "set_monitorable", "is_monitorable");

	ADD_GROUP("Audio Bus", "audio_bus_");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "audio_bus_override"), "set_audio_bus_override", "is_overriding_audio_bus");
	ADD_PROPERTY(PropertyInfo(Variant::STRING_NAME, "audio_bus_name", PROPERTY_HINT_ENUM, ""), "set_audio_bus_name", "get_audio_bus_name");

	BIND_ENUM_CONSTANT(SPACE_OVERRIDE_DISABLED);
	BIND_ENUM_CONSTANT(SPACE_OVERRIDE_COMBINE);
	BIND_ENUM_CONSTANT(SPACE_OVERRIDE_COMBINE_REPLACE);
	BIND_ENUM_CONSTANT(SPACE_OVERRIDE_REPLACE);
	BIND_ENUM_CONSTANT(SPACE_OVERRIDE_REPLACE_COMBINE);
}

void Box2DArea::_validate_property(PropertyInfo &property) const {
	if (property.name == "audio_bus_name") {
		String options;
		for (int i = 0; i < AudioServer::get_singleton()->get_bus_count(); i++) {
			if (i > 0) {
				options += ",";
			}
			String name = AudioServer::get_singleton()->get_bus_name(i);
			options += name;
		}

		property.hint_string = options;
	}
}

void Box2DArea::set_space_override_mode(SpaceOverride p_mode) {
	if (p_mode != space_override) {
		if (p_mode != Box2DArea::SpaceOverride::SPACE_OVERRIDE_DISABLED)
			_set_contact_monitor(true);
		else if (!monitoring)
			_set_contact_monitor(false);
	}
	space_override = p_mode;
}

Box2DArea::SpaceOverride Box2DArea::get_space_override_mode() const {
	return space_override;
}

void Box2DArea::set_gravity_is_point(bool p_enabled) {
	gravity_is_point = p_enabled;
}

bool Box2DArea::is_gravity_a_point() const {
	return gravity_is_point;
}

void Box2DArea::set_gravity_distance_scale(real_t p_scale) {
	gravity_distance_scale = p_scale;
}

real_t Box2DArea::get_gravity_distance_scale() const {
	return gravity_distance_scale;
}

void Box2DArea::set_gravity_vector(const Vector2 &p_vec) {
	gravity_vec = p_vec;
}

Vector2 Box2DArea::get_gravity_vector() const {
	return gravity_vec;
}

void Box2DArea::set_gravity(real_t p_gravity) {
	gravity = p_gravity;
}

real_t Box2DArea::get_gravity() const {
	return gravity;
}

void Box2DArea::set_linear_damp(real_t p_linear_damp) {
	linear_damp = p_linear_damp;
}

real_t Box2DArea::get_linear_damp() const {
	return linear_damp;
}

void Box2DArea::set_angular_damp(real_t p_angular_damp) {
	angular_damp = p_angular_damp;
}

real_t Box2DArea::get_angular_damp() const {
	return angular_damp;
}

void Box2DArea::set_priority(int p_priority) {
	priority = p_priority;
}

int Box2DArea::get_priority() const {
	return priority;
}

void Box2DArea::set_monitoring(bool p_enable) {
	if (monitoring != p_enable) {
		if (space_override != Box2DArea::SpaceOverride::SPACE_OVERRIDE_DISABLED)
			_set_contact_monitor(true);
		else if (!p_enable)
			_set_contact_monitor(false);
	}
	monitoring = p_enable;
}

bool Box2DArea::is_monitoring() const {
	return monitoring;
}

void Box2DArea::set_monitorable(bool p_enable) {
	monitorable = p_enable;
}

bool Box2DArea::is_monitorable() const {
	return monitorable;
}

TypedArray<Node2D> Box2DArea::get_overlapping_bodies() const {
	ERR_FAIL_COND_V_MSG(!monitoring, Array(), "Can't find overlapping bodies when monitoring is off.");
	TypedArray<Node2D> ret;
	ret.resize(contact_monitor->entered_objects.size()); // this is too big but better to allocate than resize N times

	int idx = 0;
	for (const ObjectID *key = contact_monitor->entered_objects.next(NULL); key; key = contact_monitor->entered_objects.next(key)) {
		const Object *obj = ObjectDB::get_instance(*key);
		const Box2DPhysicsBody *body = dynamic_cast<const Box2DPhysicsBody *>(obj);
		if (body)
			ret[idx++] = body;
	}
	ret.resize(idx);

	return ret;
}

TypedArray<Box2DArea> Box2DArea::get_overlapping_areas() const {
	ERR_FAIL_COND_V_MSG(!monitoring, Array(), "Can't find overlapping areas when monitoring is off.");
	TypedArray<Box2DArea> ret;
	ret.resize(contact_monitor->entered_objects.size()); // this is too big but better to allocate than resize N times

	int idx = 0;
	for (const ObjectID *key = contact_monitor->entered_objects.next(NULL); key; key = contact_monitor->entered_objects.next(key)) {
		const Object *obj = ObjectDB::get_instance(*key);
		const Box2DArea *area = dynamic_cast<const Box2DArea *>(obj);
		if (area)
			ret[idx++] = area;
	}
	ret.resize(idx);

	return ret;
}

bool Box2DArea::overlaps_area(Node *p_area) const {
	return contact_monitor->entered_objects.has(p_area->get_instance_id());
}

bool Box2DArea::overlaps_body(Node *p_body) const {
	return contact_monitor->entered_objects.has(p_body->get_instance_id());
}

void Box2DArea::set_audio_bus_override(bool p_override) {
	audio_bus_override = p_override;
}

bool Box2DArea::is_overriding_audio_bus() const {
	return audio_bus_override;
}

void Box2DArea::set_audio_bus_name(const StringName &p_audio_bus) {
	audio_bus = p_audio_bus;
}

StringName Box2DArea::get_audio_bus_name() const {
	return audio_bus;
}

Box2DArea::Box2DArea() {
	bodyDef.type = b2BodyType::b2_kinematicBody;
}

Box2DArea::~Box2DArea() {
}
