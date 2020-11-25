#ifndef BOX2D_SHAPE_EDITOR_PLUGIN_H
#define BOX2D_SHAPE_EDITOR_PLUGIN_H

#include "editor/editor_node.h"
#include "editor/editor_plugin.h"

#include "box2d_shapes.h"

class CanvasItemEditor;

class Box2DShapeEditor : public Control {
	GDCLASS(Box2DShapeEditor, Control);

	enum ShapeType {
		CIRCLE_SHAPE,
		RECTANGLE_SHAPE,
		SEGMENT_SHAPE,
		POLYGON_SHAPE,
		CAPSULE_SHAPE,
		UNEDITABLE_SHAPE,
	};

	EditorNode *editor;
	UndoRedo *undo_redo;
	CanvasItemEditor *canvas_item_editor;
	Box2DFixture *node;

	Vector<Point2> handles;

	ShapeType shape_type;
	int edit_handle;
	bool pressed;
	Variant original;

	Variant get_handle_value(int idx) const;
	void set_handle(int idx, Point2 &p_point);
	void commit_handle(int idx, Variant &p_org);

	void _get_current_shape_type();

protected:
	void _notification(int p_what);
	void _node_removed(Node *p_node);
	static void _bind_methods();

public:
	bool forward_canvas_gui_input(const Ref<InputEvent> &p_event);
	void forward_canvas_draw_over_viewport(Control *p_overlay);
	void edit(Node *p_node);

	Box2DShapeEditor(EditorNode *p_editor);
};

class Box2DShapeEditorPlugin : public EditorPlugin {
	GDCLASS(Box2DShapeEditorPlugin, EditorPlugin);

	Box2DShapeEditor *box2d_shape_editor;
	EditorNode *editor;

public:
	virtual bool forward_canvas_gui_input(const Ref<InputEvent> &p_event) { return box2d_shape_editor->forward_canvas_gui_input(p_event); }
	virtual void forward_canvas_draw_over_viewport(Control *p_overlay) { box2d_shape_editor->forward_canvas_draw_over_viewport(p_overlay); }

	virtual String get_name() const { return "Box2DShape"; }
	bool has_main_screen() const { return false; }
	virtual void edit(Object *p_obj);
	virtual bool handles(Object *p_obj) const;
	virtual void make_visible(bool visible);

	Box2DShapeEditorPlugin(EditorNode *p_editor);
	~Box2DShapeEditorPlugin();
};

#endif // BOX2D_SHAPE_EDITOR_PLUGIN_H
