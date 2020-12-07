#include "box2d_joint_editor_plugin.h"

#include <editor/plugins/canvas_item_editor_plugin.h>

#include "../scene/2d/box2d_joints.h"

/**
* @author Brian Semrau
*
* Referenced collision_shape_2d_editor_plugin.cpp
*/

void Box2DJointEditor::_node_removed(Node *p_node) {
	if (p_node == node) {
		node = NULL;
	}
}

Variant Box2DJointEditor::get_handle_value(int idx) const {
	switch (joint_type) {
			//case Box2DJointEditor::CIRCLE_SHAPE: {
			//	Ref<Box2DCircleShape> circle = node->get_shape();

			//	if (idx == 0) {
			//		return circle->get_radius();
			//	}

			//} break;

			//case Box2DJointEditor::RECTANGLE_SHAPE: {
			//	Ref<Box2DRectShape> rect = node->get_shape();

			//	if (idx < 3) {
			//		return rect->get_size().abs();
			//	}

			//} break;

			//case Box2DJointEditor::SEGMENT_SHAPE: {
			//	Ref<Box2DSegmentShape> segment = node->get_shape();

			//	if (idx == 0) {
			//		return segment->get_a();
			//	} else if (idx == 1) {
			//		return segment->get_b();
			//	} else if (idx == 2) {
			//		return segment->get_a_adjacent();
			//	} else if (idx == 3) {
			//		return segment->get_b_adjacent();
			//	}

			//} break;

		case REVOLUTE_JOINT: {
			// TODO
		} break;

		case PRISMATIC_JOINT: {
			// TODO
		} break;

		case DISTANCE_JOINT: {
			// TODO
		} break;

		case PULLEY_JOINT:
		case MOUSE_JOINT:
		case GEAR_JOINT:
		case WHEEL_JOINT: {
			ERR_PRINT("Not yet implemented");
		} break;

		case WELD_JOINT: {
			// TODO
		} break;

		case FRICTION_JOINT:
		case ROPE_JOINT:
		case MOTOR_JOINT:
		case INVALID_JOINT:
		default: {
			ERR_PRINT("Invalid shape type");
		} break;
	}

	return Variant();
}

void Box2DJointEditor::set_handle(int idx, Point2 &p_point) {
	switch (joint_type) {
			//case Box2DJointEditor::CIRCLE_SHAPE: {
			//	Ref<Box2DCircleShape> circle = node->get_shape();
			//	circle->set_radius(p_point.length());

			//	canvas_item_editor->update_viewport();
			//} break;

			//case Box2DJointEditor::RECTANGLE_SHAPE: {
			//	if (idx < 3) {
			//		Ref<Box2DRectShape> rect = node->get_shape();

			//		Vector2 extents = rect->get_size();
			//		if (idx == 2) {
			//			extents = p_point * 2.0f;
			//		} else {
			//			extents[idx] = p_point[idx] * 2.0f;
			//		}
			//		rect->set_size(extents.abs());

			//		canvas_item_editor->update_viewport();
			//	}
			//} break;

			//case Box2DJointEditor::SEGMENT_SHAPE: {
			//	// if (edit_handle < 2) {
			//	Ref<Box2DSegmentShape> segment = node->get_shape();

			//	if (idx == 0) {
			//		segment->set_a(p_point);
			//	} else if (idx == 1) {
			//		segment->set_b(p_point);
			//	} else if (idx == 2) {
			//		segment->set_a_adjacent(p_point);
			//	} else if (idx == 3) {
			//		segment->set_b_adjacent(p_point);
			//	}

			//	canvas_item_editor->update_viewport();
			//	// }
			//} break;

		case REVOLUTE_JOINT: {
			// TODO
		} break;

		case PRISMATIC_JOINT: {
			// TODO
		} break;

		case DISTANCE_JOINT: {
			// TODO
		} break;

		case PULLEY_JOINT:
		case MOUSE_JOINT:
		case GEAR_JOINT:
		case WHEEL_JOINT: {
			ERR_PRINT("Not yet implemented");
		} break;

		case WELD_JOINT: {
			// TODO
		} break;

		case FRICTION_JOINT:
		case ROPE_JOINT:
		case MOTOR_JOINT:
		case INVALID_JOINT:
		default: {
			ERR_PRINT("Invalid shape type");
		} break;
	}

	node->_change_notify();
}

void Box2DJointEditor::commit_handle(int idx, Variant &p_org) {
	undo_redo->create_action(TTR("Set Handle"));

	switch (joint_type) {
			//case Box2DJointEditor::CIRCLE_SHAPE: {
			//	Ref<Box2DCircleShape> circle = node->get_shape();

			//	undo_redo->add_do_method(circle.ptr(), "set_radius", circle->get_radius());
			//	undo_redo->add_do_method(canvas_item_editor, "update_viewport");
			//	undo_redo->add_undo_method(circle.ptr(), "set_radius", p_org);
			//	undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
			//} break;

			//case Box2DJointEditor::RECTANGLE_SHAPE: {
			//	Ref<Box2DRectShape> rect = node->get_shape();

			//	undo_redo->add_do_method(rect.ptr(), "set_size", rect->get_size());
			//	undo_redo->add_do_method(canvas_item_editor, "update_viewport");
			//	undo_redo->add_undo_method(rect.ptr(), "set_size", p_org);
			//	undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
			//} break;

			//case Box2DJointEditor::SEGMENT_SHAPE: {
			//	Ref<Box2DSegmentShape> segment = node->get_shape();

			//	if (idx == 0) {
			//		undo_redo->add_do_method(segment.ptr(), "set_a", segment->get_a());
			//		undo_redo->add_do_method(canvas_item_editor, "update_viewport");
			//		undo_redo->add_undo_method(segment.ptr(), "set_a", p_org);
			//		undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
			//	} else if (idx == 1) {
			//		undo_redo->add_do_method(segment.ptr(), "set_b", segment->get_b());
			//		undo_redo->add_do_method(canvas_item_editor, "update_viewport");
			//		undo_redo->add_undo_method(segment.ptr(), "set_b", p_org);
			//		undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
			//	} else if (idx == 2) {
			//		undo_redo->add_do_method(segment.ptr(), "set_a_adjacent", segment->get_a_adjacent());
			//		undo_redo->add_do_method(canvas_item_editor, "update_viewport");
			//		undo_redo->add_undo_method(segment.ptr(), "set_a_adjacent", p_org);
			//		undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
			//	} else if (idx == 3) {
			//		undo_redo->add_do_method(segment.ptr(), "set_b_adjacent", segment->get_b_adjacent());
			//		undo_redo->add_do_method(canvas_item_editor, "update_viewport");
			//		undo_redo->add_undo_method(segment.ptr(), "set_b_adjacent", p_org);
			//		undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
			//	}
			//} break;

		case REVOLUTE_JOINT: {
			// TODO
		} break;

		case PRISMATIC_JOINT: {
			// TODO
		} break;

		case DISTANCE_JOINT: {
			// TODO
		} break;

		case PULLEY_JOINT:
		case MOUSE_JOINT:
		case GEAR_JOINT:
		case WHEEL_JOINT: {
			ERR_PRINT("Not yet implemented");
		} break;

		case WELD_JOINT: {
			// TODO
		} break;

		case FRICTION_JOINT:
		case ROPE_JOINT:
		case MOTOR_JOINT:
		case INVALID_JOINT:
		default: {
			ERR_PRINT("Invalid shape type");
		} break;
	}

	undo_redo->commit_action();
}

bool Box2DJointEditor::forward_canvas_gui_input(const Ref<InputEvent> &p_event) {
	if (!node) {
		return false;
	}

	if (joint_type == INVALID_JOINT) {
		return false;
	}

	Ref<InputEventMouseButton> mb = p_event;
	Transform2D xform = canvas_item_editor->get_canvas_transform() * node->get_global_transform();

	if (mb.is_valid()) {

		Vector2 gpoint = mb->get_position();

		if (mb->get_button_index() == BUTTON_LEFT) {
			if (mb->is_pressed()) {
				for (int i = 0; i < handles.size(); i++) {
					if (xform.xform(handles[i]).distance_to(gpoint) < 8) {
						edit_handle = i;

						break;
					}
				}

				if (edit_handle == -1) {
					pressed = false;

					return false;
				}

				original = get_handle_value(edit_handle);
				pressed = true;

				return true;

			} else {
				if (pressed) {
					commit_handle(edit_handle, original);

					edit_handle = -1;
					pressed = false;

					return true;
				}
			}
		}

		return false;
	}

	Ref<InputEventMouseMotion> mm = p_event;

	if (mm.is_valid()) {
		if (edit_handle == -1 || !pressed) {
			return false;
		}

		Vector2 cpoint = canvas_item_editor->snap_point(canvas_item_editor->get_canvas_transform().affine_inverse().xform(mm->get_position()));
		cpoint = node->get_global_transform().affine_inverse().xform(cpoint);

		set_handle(edit_handle, cpoint);

		return true;
	}

	return false;
}

void Box2DJointEditor::_get_current_joint_type() {
	if (!node) {
		return;
	}

	Box2DJoint *j = node;

	// TODO maybe add an is_valid function to Box2DJoint to check if referenced bodies
	//    exist, in case those are necessary to edit joint features
	if (!j) {
		return;
	}

	if (Object::cast_to<Box2DRevoluteJoint>(j)) {
		joint_type = REVOLUTE_JOINT;
	} else if (Object::cast_to<Box2DPrismaticJoint>(j)) {
		joint_type = PRISMATIC_JOINT;
	} else if (Object::cast_to<Box2DDistanceJoint>(j)) {
		joint_type = DISTANCE_JOINT;
		// TODO more joints
	} else if (Object::cast_to<Box2DWeldJoint>(j)) {
		joint_type = WELD_JOINT;
		// TODO rest of joints
	} else {
		joint_type = INVALID_JOINT;
	}

	canvas_item_editor->update_viewport();
}

void Box2DJointEditor::forward_canvas_draw_over_viewport(Control *p_overlay) {
	if (!node) {
		return;
	}

	_get_current_joint_type();

	if (joint_type == -1) {
		return;
	}

	Transform2D gt = canvas_item_editor->get_canvas_transform() * node->get_global_transform();

	Ref<Texture> h = get_icon("EditorHandle", "EditorIcons");
	Vector2 size = h->get_size() * 0.5;

	handles.clear();

	switch (joint_type) {
			//case Box2DJointEditor::CIRCLE_SHAPE: {
			//	Ref<Box2DCircleShape> shape = node->get_shape();

			//	handles.resize(1);
			//	handles.write[0] = Point2(shape->get_radius(), 0);

			//	p_overlay->draw_texture(h, gt.xform(handles[0]) - size);
			//} break;

			//case Box2DJointEditor::RECTANGLE_SHAPE: {
			//	Ref<Box2DRectShape> shape = node->get_shape();

			//	handles.resize(3);
			//	Vector2 ext = shape->get_size() * 0.5f;
			//	handles.write[0] = Point2(ext.x, 0);
			//	handles.write[1] = Point2(0, -ext.y);
			//	handles.write[2] = Point2(ext.x, -ext.y);

			//	p_overlay->draw_texture(h, gt.xform(handles[0]) - size);
			//	p_overlay->draw_texture(h, gt.xform(handles[1]) - size);
			//	p_overlay->draw_texture(h, gt.xform(handles[2]) - size);
			//} break;

			//case Box2DJointEditor::SEGMENT_SHAPE: {
			//	Ref<Box2DSegmentShape> shape = node->get_shape();

			//	if (shape->is_one_sided()) {
			//		handles.resize(4);

			//		handles.write[0] = shape->get_a();
			//		handles.write[1] = shape->get_b();
			//		handles.write[2] = shape->get_a_adjacent();
			//		handles.write[3] = shape->get_b_adjacent();

			//		p_overlay->draw_texture(h, gt.xform(handles[0]) - size);
			//		p_overlay->draw_texture(h, gt.xform(handles[1]) - size);
			//		p_overlay->draw_texture(h, gt.xform(handles[2]) - size);
			//		p_overlay->draw_texture(h, gt.xform(handles[3]) - size);
			//	} else {
			//		handles.resize(2);

			//		handles.write[0] = shape->get_a();
			//		handles.write[1] = shape->get_b();

			//		p_overlay->draw_texture(h, gt.xform(handles[0]) - size);
			//		p_overlay->draw_texture(h, gt.xform(handles[1]) - size);
			//	}
			//} break;

		case REVOLUTE_JOINT: {
			// TODO
		} break;

		case PRISMATIC_JOINT: {
			// TODO
		} break;

		case DISTANCE_JOINT: {
			// TODO
		} break;

		case PULLEY_JOINT:
		case MOUSE_JOINT:
		case GEAR_JOINT:
		case WHEEL_JOINT: {
			ERR_PRINT("Not yet implemented");
		} break;

		case WELD_JOINT: {
			// TODO
		} break;

		case FRICTION_JOINT:
		case ROPE_JOINT:
		case MOTOR_JOINT:
		case INVALID_JOINT:
		default: {
			ERR_PRINT("Invalid shape type");
		} break;
	}
}

void Box2DJointEditor::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			get_tree()->connect("node_removed", this, "_node_removed");
		} break;

		case NOTIFICATION_EXIT_TREE: {
			get_tree()->disconnect("node_removed", this, "_node_removed");
		} break;
	}
}

void Box2DJointEditor::edit(Node *p_node) {
	if (!canvas_item_editor) {
		canvas_item_editor = CanvasItemEditor::get_singleton();
	}

	if (p_node) {
		node = Object::cast_to<Box2DJoint>(p_node);

		_get_current_joint_type();

	} else {
		edit_handle = -1;
		joint_type = INVALID_JOINT;

		node = NULL;
	}

	canvas_item_editor->update_viewport();
}

void Box2DJointEditor::_bind_methods() {

	ClassDB::bind_method("_get_current_joint_type", &Box2DJointEditor::_get_current_joint_type);
	ClassDB::bind_method(D_METHOD("_node_removed"), &Box2DJointEditor::_node_removed);
}

Box2DJointEditor::Box2DJointEditor(EditorNode *p_editor) :
		node(NULL),
		canvas_item_editor(NULL),
		editor(p_editor),
		undo_redo(p_editor->get_undo_redo()),
		joint_type(INVALID_JOINT),
		edit_handle(-1),
		pressed(false) {}

void Box2DJointEditorPlugin::edit(Object *p_obj) {
	box2d_joint_editor->edit(Object::cast_to<Node>(p_obj));
}

bool Box2DJointEditorPlugin::handles(Object *p_obj) const {
	Box2DJoint *node = Object::cast_to<Box2DJoint>(p_obj);
	return node != NULL;
}

void Box2DJointEditorPlugin::make_visible(bool visible) {
	if (!visible) {
		edit(NULL);
	}
}

Box2DJointEditorPlugin::Box2DJointEditorPlugin(EditorNode *p_editor) {
	editor = p_editor;

	box2d_joint_editor = memnew(Box2DJointEditor(p_editor));
	p_editor->get_gui_base()->add_child(box2d_joint_editor);
}

Box2DJointEditorPlugin::~Box2DJointEditorPlugin() {
}
