#include "register_types.h"

#include "editor/box2d_joint_editor_plugin.h"
#include "editor/box2d_polygon_editor_plugin.h"
#include "editor/box2d_shape_editor_plugin.h"
#include "scene/2d/box2d_fixtures.h"
#include "scene/2d/box2d_joints.h"
#include "scene/2d/box2d_physics_body.h"
#include "scene/2d/box2d_world.h"
#include "scene/resources/box2d_shapes.h"

/**
* @author Brian Semrau
*/

void register_godot_box2d_types() {
	GLOBAL_DEF("physics/2d/box2d_conversion_factor", 50.0f);
	ProjectSettings::get_singleton()->set_custom_property_info("physics/2d/box2d_conversion_factor", PropertyInfo(Variant::FLOAT, "physics/2d/box2d_conversion_factor"));

	ClassDB::register_class<Box2DShapeQueryParameters>();
	ClassDB::register_class<Box2DWorld>();
	ClassDB::register_class<Box2DPhysicsBody>();
	ClassDB::register_class<Box2DFixture>();
	ClassDB::register_virtual_class<Box2DShape>();
	ClassDB::register_class<Box2DCircleShape>();
	ClassDB::register_class<Box2DRectShape>();
	ClassDB::register_class<Box2DSegmentShape>();
	ClassDB::register_class<Box2DPolygonShape>();
	ClassDB::register_class<Box2DCapsuleShape>();

	ClassDB::register_virtual_class<Box2DJoint>();
	ClassDB::register_class<Box2DRevoluteJoint>();
	ClassDB::register_class<Box2DPrismaticJoint>();
	ClassDB::register_class<Box2DDistanceJoint>();
	ClassDB::register_class<Box2DWeldJoint>();
	// TODO more joints

#ifdef TOOLS_ENABLED
	EditorPlugins::add_by_type<Box2DPolygonEditorPlugin>();
	EditorPlugins::add_by_type<Box2DShapeEditorPlugin>();
	EditorPlugins::add_by_type<Box2DJointEditorPlugin>();
#endif
}

void unregister_godot_box2d_types() {
}
