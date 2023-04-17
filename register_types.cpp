#include "register_types.h"

#include "scene/2d/box2d_area.h"
#include "scene/2d/box2d_collision_object.h"
#include "scene/2d/box2d_fixtures.h"
#include "scene/2d/box2d_joints.h"
#include "scene/2d/box2d_physics_body.h"
#include "scene/2d/box2d_world.h"
#include "scene/resources/box2d_shapes.h"
#ifdef TOOLS_ENABLED
#include "editor/box2d_joint_editor_plugin.h"
#include "editor/box2d_polygon_editor_plugin.h"
#include "editor/box2d_shape_editor_plugin.h"
#endif

/**
* @author Brian Semrau
*/

void initialize_godot_box2d_module(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
		GLOBAL_DEF("physics/2d/box2d_conversion_factor", 50.0f);
		ProjectSettings::get_singleton()->set_custom_property_info(PropertyInfo(Variant::FLOAT, "physics/2d/box2d_conversion_factor"));

		ClassDB::register_class<Box2DShapeQueryParameters>();
		ClassDB::register_class<Box2DWorld>();

		ClassDB::register_abstract_class<Box2DCollisionObject>();
		ClassDB::register_class<Box2DPhysicsTestMotionResult>();
		ClassDB::register_class<Box2DPhysicsBody>();
		ClassDB::register_class<Box2DKinematicCollision>();
		ClassDB::register_class<Box2DArea>();

		ClassDB::register_class<Box2DFixture>();

		ClassDB::register_abstract_class<Box2DShape>();
		ClassDB::register_class<Box2DCircleShape>();
		ClassDB::register_class<Box2DRectShape>();
		ClassDB::register_class<Box2DSegmentShape>();
		ClassDB::register_class<Box2DPolygonShape>();
		ClassDB::register_class<Box2DCapsuleShape>();

		ClassDB::register_abstract_class<Box2DJoint>();
		ClassDB::register_class<Box2DRevoluteJoint>();
		ClassDB::register_class<Box2DPrismaticJoint>();
		ClassDB::register_class<Box2DDistanceJoint>();
		ClassDB::register_class<Box2DWeldJoint>();
		// TODO more joints
	}
#ifdef TOOLS_ENABLED
	if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
		EditorPlugins::add_by_type<Box2DPolygonEditorPlugin>();
		EditorPlugins::add_by_type<Box2DShapeEditorPlugin>();
		EditorPlugins::add_by_type<Box2DJointEditorPlugin>();
	}
#endif
}

void uninitialize_godot_box2d_module(ModuleInitializationLevel p_level) {
}
