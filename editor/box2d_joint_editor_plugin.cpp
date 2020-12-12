#include "box2d_joint_editor_plugin.h"

#include <editor/plugins/canvas_item_editor_plugin.h>

#include "../scene/2d/box2d_joints.h"

#include <string.h>

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

void Box2DJointEditor::_menu_option(int p_option) {
	switch (static_cast<AnchorMode>(p_option)) {
		case AnchorMode::MODE_ANCHORS_LOCAL: {
			anchor_mode = AnchorMode::MODE_ANCHORS_LOCAL;
			button_anchor_local->set_pressed(true);
			button_anchor_global->set_pressed(false);
		} break;

		case AnchorMode::MODE_ANCHORS_STICKY: {
			anchor_mode = AnchorMode::MODE_ANCHORS_STICKY;
			button_anchor_local->set_pressed(false);
			button_anchor_global->set_pressed(true);
		} break;
	}

	if (node)
		node->editor_anchor_mode = static_cast<Box2DJointEditor::AnchorMode>(p_option);
}

void Box2DJointEditor::disable_anchor_modes(bool p_disable, String p_reason) {
	button_anchor_local->set_disabled(p_disable);
	button_anchor_global->set_disabled(p_disable);

	if (p_disable) {
		button_anchor_local->set_tooltip(p_reason);
		button_anchor_global->set_tooltip(p_reason);
	} else {
		button_anchor_local->set_tooltip(TTR("Move anchors with joint")); // TODO translation file?
		button_anchor_global->set_tooltip(TTR("Anchors stick to their body"));
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

		case JointType::REVOLUTE_JOINT: {
			Box2DRevoluteJoint *j = Object::cast_to<Box2DRevoluteJoint>(node);

			switch (idx) {
				case 0:
					return j->get_anchor_a();
				case 1:
					return j->get_anchor_b();
				case 2:
					return j->get_lower_limit();
				case 3:
					return j->get_upper_limit();
			}
		} break;

		case JointType::PRISMATIC_JOINT: {
			Box2DPrismaticJoint *j = Object::cast_to<Box2DPrismaticJoint>(node);

			switch (idx) {
				case 0:
					return j->get_anchor_a();
				case 1:
					return j->get_anchor_b();
				case 2:
					return j->get_local_axis();
				case 3:
					return j->get_lower_limit();
				case 4:
					return j->get_upper_limit();
			}
		} break;

		case JointType::DISTANCE_JOINT: {
			// TODO
		} break;

		case JointType::PULLEY_JOINT:
		case JointType::MOUSE_JOINT:
		case JointType::GEAR_JOINT:
		case JointType::WHEEL_JOINT: {
			ERR_PRINT("Not yet implemented");
		} break;

		case JointType::WELD_JOINT: {
			// TODO
		} break;

		case JointType::FRICTION_JOINT:
		case JointType::ROPE_JOINT:
		case JointType::MOTOR_JOINT:
		case JointType::INVALID_JOINT:
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

		case JointType::REVOLUTE_JOINT: {
			Box2DRevoluteJoint *j = Object::cast_to<Box2DRevoluteJoint>(node);

			Point2 offset_point = p_point - j->get_anchor_a(); // offset to match debug draw

			switch (idx) {
				case 0: {
					j->set_anchor_a(p_point);
				} break;
				case 1: {
					j->set_anchor_b(p_point);
				} break;
				case 2: {
					// This allows you to wrap handle around to set limit angles >180 or <-180
					float angle_prev = j->get_lower_limit();
					float angle_vector = offset_point.angle();
					int n = roundf((angle_prev - angle_vector) / static_cast<float>(Math_PI));

					j->set_lower_limit(angle_vector + n * Math_PI);
				} break;
				case 3: {
					float angle_prev = j->get_upper_limit();
					float angle_vector = offset_point.angle();
					int n = roundf((angle_prev - angle_vector) / static_cast<float>(Math_PI));

					j->set_upper_limit(angle_vector + n * Math_PI);
				} break;
			}

			canvas_item_editor->update_viewport();
		} break;

		case JointType::PRISMATIC_JOINT: {
			Box2DPrismaticJoint *j = Object::cast_to<Box2DPrismaticJoint>(node);

			Point2 offset_point = p_point - j->get_anchor_a(); // offset to match debug draw

			switch (idx) {
				case 0: {
					j->set_anchor_a(p_point);
				} break;
				case 1: {
					j->set_anchor_b(p_point);
				} break;
				case 2: {
					j->set_local_axis(offset_point);
				} break;
				case 3: {
					j->set_lower_limit(offset_point.dot(j->get_local_axis().normalized()));
				} break;
				case 4: {
					j->set_upper_limit(offset_point.dot(j->get_local_axis().normalized()));
				} break;
			}

			canvas_item_editor->update_viewport();
		} break;

		case JointType::DISTANCE_JOINT: {
			// TODO
		} break;

		case JointType::PULLEY_JOINT:
		case JointType::MOUSE_JOINT:
		case JointType::GEAR_JOINT:
		case JointType::WHEEL_JOINT: {
			ERR_PRINT("Not yet implemented");
		} break;

		case JointType::WELD_JOINT: {
			// TODO
		} break;

		case JointType::FRICTION_JOINT:
		case JointType::ROPE_JOINT:
		case JointType::MOTOR_JOINT:
		case JointType::INVALID_JOINT:
		default: {
			ERR_PRINT("Invalid shape type");
		} break;
	}
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

		case JointType::REVOLUTE_JOINT: {
			Box2DRevoluteJoint *j = Object::cast_to<Box2DRevoluteJoint>(node);

			switch (idx) {
				case 0: {
					undo_redo->add_do_method(j, "set_anchor_a", j->get_anchor_a());
					undo_redo->add_do_method(canvas_item_editor, "update_viewport");
					undo_redo->add_undo_method(j, "set_anchor_a", p_org);
					undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
				} break;
				case 1: {
					undo_redo->add_do_method(j, "set_anchor_b", j->get_anchor_b());
					undo_redo->add_do_method(canvas_item_editor, "update_viewport");
					undo_redo->add_undo_method(j, "set_anchor_b", p_org);
					undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
				} break;
				case 2: {
					undo_redo->add_do_method(j, "set_lower_limit", j->get_lower_limit());
					undo_redo->add_do_method(canvas_item_editor, "update_viewport");
					undo_redo->add_undo_method(j, "set_lower_limit", p_org);
					undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
				} break;
				case 3: {
					undo_redo->add_do_method(j, "set_upper_limit", j->get_upper_limit());
					undo_redo->add_do_method(canvas_item_editor, "update_viewport");
					undo_redo->add_undo_method(j, "set_upper_limit", p_org);
					undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
				} break;
			}
		} break;

		case JointType::PRISMATIC_JOINT: {
			Box2DPrismaticJoint *j = Object::cast_to<Box2DPrismaticJoint>(node);

			switch (idx) {
				case 0: {
					undo_redo->add_do_method(j, "set_anchor_a", j->get_anchor_a());
					undo_redo->add_do_method(canvas_item_editor, "update_viewport");
					undo_redo->add_undo_method(j, "set_anchor_a", p_org);
					undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
				} break;
				case 1: {
					undo_redo->add_do_method(j, "set_anchor_b", j->get_anchor_b());
					undo_redo->add_do_method(canvas_item_editor, "update_viewport");
					undo_redo->add_undo_method(j, "set_anchor_b", p_org);
					undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
				} break;
				case 2: {
					undo_redo->add_do_method(j, "set_local_axis", j->get_local_axis());
					undo_redo->add_do_method(canvas_item_editor, "update_viewport");
					undo_redo->add_undo_method(j, "set_local_axis", p_org);
					undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
				} break;
				case 3: {
					undo_redo->add_do_method(j, "set_lower_limit", j->get_lower_limit());
					undo_redo->add_do_method(canvas_item_editor, "update_viewport");
					undo_redo->add_undo_method(j, "set_lower_limit", p_org);
					undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
				} break;
				case 4: {
					undo_redo->add_do_method(j, "set_upper_limit", j->get_upper_limit());
					undo_redo->add_do_method(canvas_item_editor, "update_viewport");
					undo_redo->add_undo_method(j, "set_upper_limit", p_org);
					undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
				} break;
			}
		} break;

		case JointType::DISTANCE_JOINT: {
			//undo_redo->add_do_method(j, "set_", j->get_());
			//undo_redo->add_do_method(canvas_item_editor, "update_viewport");
			//undo_redo->add_undo_method(j, "set_", p_org);
			//undo_redo->add_undo_method(canvas_item_editor, "update_viewport");
			// TODO
		} break;

		case JointType::PULLEY_JOINT:
		case JointType::MOUSE_JOINT:
		case JointType::GEAR_JOINT:
		case JointType::WHEEL_JOINT: {
			ERR_PRINT("Not yet implemented");
		} break;

		case JointType::WELD_JOINT: {
			// TODO
		} break;

		case JointType::FRICTION_JOINT:
		case JointType::ROPE_JOINT:
		case JointType::MOTOR_JOINT:
		case JointType::INVALID_JOINT:
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

	if (joint_type == JointType::INVALID_JOINT) {
		return false;
	}

	Ref<InputEventMouseButton> mb = p_event;
	Transform2D xform = canvas_item_editor->get_canvas_transform() * node->get_global_transform();

	if (mb.is_valid()) {
		Vector2 gpoint = mb->get_position();

		if (mb->get_button_index() == BUTTON_LEFT) {
			if (mb->is_pressed()) {
				for (int i = 0; i < handles.size(); i++) {
					if (xform.xform(handles[i]).distance_to(gpoint - handle_offsets[i]) < 8) {
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

		Vector2 cpoint = canvas_item_editor->snap_point(canvas_item_editor->get_canvas_transform().affine_inverse().xform(mm->get_position() - handle_offsets[edit_handle]));
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
		joint_type = JointType::REVOLUTE_JOINT;
	} else if (Object::cast_to<Box2DPrismaticJoint>(j)) {
		joint_type = JointType::PRISMATIC_JOINT;
	} else if (Object::cast_to<Box2DDistanceJoint>(j)) {
		joint_type = JointType::DISTANCE_JOINT;
		// TODO more joints
	} else if (Object::cast_to<Box2DWeldJoint>(j)) {
		joint_type = JointType::WELD_JOINT;
		// TODO rest of joints
	} else {
		joint_type = JointType::INVALID_JOINT;
	}

	canvas_item_editor->update_viewport();
}

void Box2DJointEditor::forward_canvas_draw_over_viewport(Control *p_overlay) {
	if (!node) {
		return;
	}

	_get_current_joint_type();

	if (joint_type == JointType::INVALID_JOINT) {
		return;
	}

	Transform2D gt = canvas_item_editor->get_canvas_transform() * node->get_global_transform();

	Ref<Theme> theme = EditorNode::get_singleton()->get_editor_theme();
	Ref<Texture2D> handle_icon = theme->get_icon("EditorHandle", "EditorIcons");
	Ref<Texture2D> anchor_icon = theme->get_icon("EditorControlAnchor", "EditorIcons");
	Vector2 anchor_size = anchor_icon->get_size();
	Vector2 handle_hsize = handle_icon->get_size() * 0.5;

	handles.clear();
	handle_offsets.clear();

	switch (joint_type) {
			//case Box2DJointEditor::CIRCLE_SHAPE: {
			//	Ref<Box2DCircleShape> shape = node->get_shape();

			//	handles.resize(1);
			//	handles.write[0] = Point2(shape->get_radius(), 0);

			//	p_overlay->draw_texture(handle_icon, gt.xform(handles[0]) - hsize);
			//} break;

			//case Box2DJointEditor::RECTANGLE_SHAPE: {
			//	Ref<Box2DRectShape> shape = node->get_shape();

			//	handles.resize(3);
			//	Vector2 ext = shape->get_size() * 0.5f;
			//	handles.write[0] = Point2(ext.x, 0);
			//	handles.write[1] = Point2(0, -ext.y);
			//	handles.write[2] = Point2(ext.x, -ext.y);

			//	p_overlay->draw_texture(handle_icon, gt.xform(handles[0]) - hsize);
			//	p_overlay->draw_texture(handle_icon, gt.xform(handles[1]) - hsize);
			//	p_overlay->draw_texture(handle_icon, gt.xform(handles[2]) - hsize);
			//} break;

			//case Box2DJointEditor::SEGMENT_SHAPE: {
			//	Ref<Box2DSegmentShape> shape = node->get_shape();

			//	if (shape->is_one_sided()) {
			//		handles.resize(4);

			//		handles.write[0] = shape->get_a();
			//		handles.write[1] = shape->get_b();
			//		handles.write[2] = shape->get_a_adjacent();
			//		handles.write[3] = shape->get_b_adjacent();

			//		p_overlay->draw_texture(handle_icon, gt.xform(handles[0]) - hsize);
			//		p_overlay->draw_texture(handle_icon, gt.xform(handles[1]) - hsize);
			//		p_overlay->draw_texture(handle_icon, gt.xform(handles[2]) - hsize);
			//		p_overlay->draw_texture(handle_icon, gt.xform(handles[3]) - hsize);
			//	} else {
			//		handles.resize(2);

			//		handles.write[0] = shape->get_a();
			//		handles.write[1] = shape->get_b();

			//		p_overlay->draw_texture(handle_icon, gt.xform(handles[0]) - hsize);
			//		p_overlay->draw_texture(handle_icon, gt.xform(handles[1]) - hsize);
			//	}
			//} break;

		case JointType::REVOLUTE_JOINT: {
			Box2DRevoluteJoint *j = Object::cast_to<Box2DRevoluteJoint>(node);

			handles.resize(2);
			handles.write[0] = j->get_anchor_a();
			handles.write[1] = j->get_anchor_b();

			handle_offsets.resize(2);
			handle_offsets.write[0] = (-anchor_size + handle_hsize); // approximately correct
			handle_offsets.write[1] = (-anchor_size + handle_hsize) * Size2(-1, 1);

			if (j->is_limit_enabled()) {
				handles.resize(4);
				handles.write[2] = j->get_anchor_a() + Point2(10, 0).rotated(j->get_lower_limit());
				handles.write[3] = j->get_anchor_a() + Point2(10, 0).rotated(j->get_upper_limit());

				handle_offsets.resize(4);
				handle_offsets.write[2] = Point2();
				handle_offsets.write[3] = Point2();

				p_overlay->draw_texture(handle_icon, gt.xform(handles[3]) - handle_hsize);
				p_overlay->draw_texture(handle_icon, gt.xform(handles[2]) - handle_hsize);
			}

			// what the F i just want a rotated texture
			Transform2D rot_xform = Transform2D(Math_PI * 0.5f, Vector2());
			Transform2D xform = (rot_xform.inverse() * gt);
			p_overlay->draw_set_transform_matrix(rot_xform);
			p_overlay->draw_texture(anchor_icon, xform.xform(handles[1]) - anchor_size);
			p_overlay->draw_set_transform_matrix(Transform2D());

			p_overlay->draw_texture(anchor_icon, gt.xform(handles[0]) - anchor_size);
		} break;

		case JointType::PRISMATIC_JOINT: {
			Box2DPrismaticJoint *j = Object::cast_to<Box2DPrismaticJoint>(node);

			handles.resize(3);
			handles.write[0] = j->get_anchor_a();
			handles.write[1] = j->get_anchor_b();
			handles.write[2] = j->get_anchor_a() + j->get_local_axis() * 25;

			handle_offsets.resize(3);
			handle_offsets.write[0] = (-anchor_size + handle_hsize);
			handle_offsets.write[1] = (-anchor_size + handle_hsize) * Size2(-1, 1);
			handle_offsets.write[2] = Point2();

			if (j->is_limit_enabled()) {
				handles.resize(5);
				handles.write[3] = j->get_anchor_a() + j->get_local_axis() * j->get_lower_limit();
				handles.write[4] = j->get_anchor_a() + j->get_local_axis() * j->get_upper_limit();

				p_overlay->draw_texture(handle_icon, gt.xform(handles[4]) - handle_hsize);
				p_overlay->draw_texture(handle_icon, gt.xform(handles[3]) - handle_hsize);

				handle_offsets.resize(5);
				handle_offsets.write[3] = Point2();
				handle_offsets.write[4] = Point2();
			}

			p_overlay->draw_texture(handle_icon, gt.xform(handles[2]) - handle_hsize);

			Transform2D rot_xform = Transform2D(Math_PI * 0.5f, Vector2());
			Transform2D xform = (rot_xform.inverse() * gt);
			p_overlay->draw_set_transform_matrix(rot_xform);
			p_overlay->draw_texture(anchor_icon, xform.xform(handles[1]) - anchor_size);
			p_overlay->draw_set_transform_matrix(Transform2D());

			p_overlay->draw_texture(anchor_icon, gt.xform(handles[0]) - anchor_size);
		} break;

		case JointType::DISTANCE_JOINT: {
			// TODO
		} break;

		case JointType::PULLEY_JOINT:
		case JointType::MOUSE_JOINT:
		case JointType::GEAR_JOINT:
		case JointType::WHEEL_JOINT: {
			ERR_PRINT("Not yet implemented");
		} break;

		case JointType::WELD_JOINT: {
			// TODO
		} break;

		case JointType::FRICTION_JOINT:
		case JointType::ROPE_JOINT:
		case JointType::MOTOR_JOINT:
		case JointType::INVALID_JOINT:
		default: {
			ERR_PRINT("Invalid shape type");
		} break;
	}
}

void Box2DJointEditor::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			disable_anchor_modes(false, String());

			button_anchor_local->set_icon(EditorNode::get_singleton()->get_gui_base()->get_theme_icon("AnchorsLocal", "EditorIcons"));
			button_anchor_global->set_icon(EditorNode::get_singleton()->get_gui_base()->get_theme_icon("StickyAnchors", "EditorIcons"));
			button_anchor_local->set_pressed(true);
		} break;

		case NOTIFICATION_ENTER_TREE: {
			get_tree()->connect("node_removed", callable_mp(this, &Box2DJointEditor::_node_removed));
		} break;

		case NOTIFICATION_EXIT_TREE: {
			get_tree()->disconnect("node_removed", callable_mp(this, &Box2DJointEditor::_node_removed));
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

		//node->add_change_receptor(this);

		_menu_option(static_cast<int>(node->editor_anchor_mode));

		prev_joint_xform = node->get_global_transform(); // TODO should this be box2d_global_transform?

		switch (joint_type) {
			// TODO I think certain joints don't use body anchors like the others (gear?)
			case Box2DJointEditor::JointType::REVOLUTE_JOINT:
			case Box2DJointEditor::JointType::PRISMATIC_JOINT:
			case Box2DJointEditor::JointType::DISTANCE_JOINT:
			case Box2DJointEditor::JointType::PULLEY_JOINT:
			case Box2DJointEditor::JointType::MOUSE_JOINT:
			case Box2DJointEditor::JointType::GEAR_JOINT:
			case Box2DJointEditor::JointType::WHEEL_JOINT:
			case Box2DJointEditor::JointType::WELD_JOINT:
			case Box2DJointEditor::JointType::FRICTION_JOINT:
			case Box2DJointEditor::JointType::ROPE_JOINT:
			case Box2DJointEditor::JointType::MOTOR_JOINT:
			case Box2DJointEditor::JointType::INVALID_JOINT: {
				disable_anchor_modes(false, String());
			} break;
		}
		// TODO if more buttons added, enable/disable them here as they apply
	} else {
		edit_handle = -1;
		joint_type = JointType::INVALID_JOINT;

		//if (node)
		//	node->remove_change_receptor(this);
		node = NULL;
	}

	canvas_item_editor->update_viewport();
}

void Box2DJointEditor::_bind_methods() {
}

Box2DJointEditor::Box2DJointEditor(EditorNode *p_editor) :
		editor(p_editor),
		undo_redo(EditorNode::get_undo_redo()) {
	add_child(memnew(VSeparator));
	button_anchor_local = memnew(Button);
	button_anchor_local->set_flat(true);
	add_child(button_anchor_local);
	button_anchor_local->connect("pressed", callable_mp(this, &Box2DJointEditor::_menu_option), varray(static_cast<int>(AnchorMode::MODE_ANCHORS_LOCAL)));
	button_anchor_local->set_toggle_mode(true);

	button_anchor_global = memnew(Button);
	button_anchor_global->set_flat(true);
	add_child(button_anchor_global);
	button_anchor_global->connect("pressed", callable_mp(this, &Box2DJointEditor::_menu_option), varray(static_cast<int>(AnchorMode::MODE_ANCHORS_STICKY)));
	button_anchor_global->set_toggle_mode(true);

	// [VSeparator]
	// TODO add any additional buttons for other joints
}

void Box2DJointEditorPlugin::edit(Object *p_obj) {
	box2d_joint_editor->edit(Object::cast_to<Node>(p_obj));
}

bool Box2DJointEditorPlugin::handles(Object *p_obj) const {
	Box2DJoint *node = Object::cast_to<Box2DJoint>(p_obj);
	return node != nullptr;
}

void Box2DJointEditorPlugin::make_visible(bool p_visible) {
	if (p_visible) {
		box2d_joint_editor->show();
	} else {
		box2d_joint_editor->hide();
		edit(nullptr);
	}
}

Box2DJointEditorPlugin::Box2DJointEditorPlugin(EditorNode *p_editor) {
	editor = p_editor;

	box2d_joint_editor = memnew(Box2DJointEditor(p_editor));
	//p_editor->get_gui_base()->add_child(box2d_joint_editor);
	CanvasItemEditor::get_singleton()->add_control_to_menu_panel(box2d_joint_editor);
	box2d_joint_editor->hide();
}

Box2DJointEditorPlugin::~Box2DJointEditorPlugin() {
}
