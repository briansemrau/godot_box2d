#ifndef BOX2D_JOINT_EDITOR_PLUGIN_H
#define BOX2D_JOINT_EDITOR_PLUGIN_H

#include <editor/editor_node.h>
#include <editor/editor_plugin.h>

#include "../scene/resources/box2d_shapes.h"

/**
* @author Brian Semrau
*
* Referenced collision_shape_2d_editor_plugin.h
*/

class CanvasItemEditor;

class Box2DJointEditor : public Control {
	GDCLASS(Box2DJointEditor, Control);

	enum JointType {
		REVOLUTE_JOINT,
		PRISMATIC_JOINT,
		DISTANCE_JOINT,
		PULLEY_JOINT,
		MOUSE_JOINT,
		GEAR_JOINT,
		WHEEL_JOINT,
		WELD_JOINT,
		FRICTION_JOINT,
		ROPE_JOINT,
		MOTOR_JOINT,
		INVALID_JOINT
	};

	EditorNode *editor;
	UndoRedo *undo_redo;
	CanvasItemEditor *canvas_item_editor;
	Box2DJoint *node;

	Vector<Point2> handles;

	JointType joint_type;
	int edit_handle;
	bool pressed;
	Variant original;

	Variant get_handle_value(int idx) const;
	void set_handle(int idx, Point2 &p_point);
	void commit_handle(int idx, Variant &p_org);

	void _get_current_joint_type();

protected:
	void _notification(int p_what);
	void _node_removed(Node *p_node);
	static void _bind_methods();

public:
	bool forward_canvas_gui_input(const Ref<InputEvent> &p_event);
	void forward_canvas_draw_over_viewport(Control *p_overlay);
	void edit(Node *p_node);

	Box2DJointEditor(EditorNode *p_editor);
};

class Box2DJointEditorPlugin : public EditorPlugin {
	GDCLASS(Box2DJointEditorPlugin, EditorPlugin);

	Box2DJointEditor *box2d_joint_editor;
	EditorNode *editor;

public:
	virtual bool forward_canvas_gui_input(const Ref<InputEvent> &p_event) { return box2d_joint_editor->forward_canvas_gui_input(p_event); }
	virtual void forward_canvas_draw_over_viewport(Control *p_overlay) { box2d_joint_editor->forward_canvas_draw_over_viewport(p_overlay); }

	virtual String get_name() const { return "Box2DJoint"; }
	bool has_main_screen() const { return false; }
	virtual void edit(Object *p_obj);
	virtual bool handles(Object *p_obj) const;
	virtual void make_visible(bool visible);

	Box2DJointEditorPlugin(EditorNode *p_editor);
	~Box2DJointEditorPlugin();
};

#endif // BOX2D_JOINT_EDITOR_PLUGIN_H
