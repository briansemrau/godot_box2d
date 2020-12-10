#include "box2d_polygon_editor_plugin.h"

#include "../scene/2d/box2d_fixtures.h"

/**
* @author Brian Semrau
*/

void Box2DPolygonEditor::_shape_type_changed() {
	editor->edit_current();
}

void Box2DPolygonEditor::_bind_methods() {
	ClassDB::bind_method(D_METHOD("_shape_type_changed"), &Box2DPolygonEditor::_shape_type_changed);
}

Node2D *Box2DPolygonEditor::_get_node() const {
	return node;
}

void Box2DPolygonEditor::_set_node(Node *p_polygon) {
	if (node) {
		node->disconnect("_shape_type_changed", this, "_shape_type_changed");
	}

	node = Object::cast_to<Box2DFixture>(p_polygon);

	if (node) {
		node->connect("_shape_type_changed", this, "_shape_type_changed", Vector<Variant>(), CONNECT_DEFERRED);
	}
}

bool Box2DPolygonEditor::_is_line() const {
	if (node->get_shape().is_valid()) {
		Box2DPolygonShape *poly = dynamic_cast<Box2DPolygonShape *>(*node->get_shape());
		if (poly) {
			return poly->get_build_mode() == Box2DPolygonShape::BUILD_OPEN_SEGMENTS;
		}
	}
	return false;
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
		AbstractPolygon2DEditor(p_editor) {
	editor = p_editor;
}

bool Box2DPolygonEditorPlugin::handles(Object *p_object) const {
	Box2DFixture *node = Object::cast_to<Box2DFixture>(p_object);
	return node && node->get_shape().is_valid() && node->get_shape()->is_class("Box2DPolygonShape");
}

Box2DPolygonEditorPlugin::Box2DPolygonEditorPlugin(EditorNode *p_node) :
		AbstractPolygon2DEditorPlugin(p_node, memnew(Box2DPolygonEditor(p_node)), "Box2DPolygonShape") {}
