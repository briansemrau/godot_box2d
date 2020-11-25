#ifndef BOX2D_POLYGON_EDITOR_PLUGIN_H
#define BOX2D_POLYGON_EDITOR_PLUGIN_H

#include <editor/plugins/abstract_polygon_2d_editor.h>

#include "../scene/resources/box2d_shapes.h"

/**
* @author Brian Semrau
*/

class Box2DPolygonEditor : public AbstractPolygon2DEditor {
	GDCLASS(Box2DPolygonEditor, AbstractPolygon2DEditor);

	Box2DFixture *node;
	Box2DPolygonShape *shape;

protected:
	virtual Node2D *_get_node() const override;
	virtual void _set_node(Node *p_polygon) override;

	virtual bool _is_line() const override;
	virtual Variant _get_polygon(int p_idx) const override;
	virtual void _set_polygon(int p_idx, const Variant &p_polygon) const override;

	virtual void _action_set_polygon(int p_idx, const Variant &p_previous, const Variant &p_polygon) override;

public:
	Box2DPolygonEditor(EditorNode *p_editor);
};

class Box2DPolygonEditorPlugin : public AbstractPolygon2DEditorPlugin {
	GDCLASS(Box2DPolygonEditorPlugin, AbstractPolygon2DEditorPlugin);

public:
	virtual bool handles(Object *p_object) const override;

	Box2DPolygonEditorPlugin(EditorNode *p_node);
};

#endif // BOX2D_POLYGON_EDITOR_PLUGIN_H
