
#include "register_types.h"

#include "box2d_world.h"
#include "box2d_physics_body.h"
#include "box2d_fixtures.h"
#include "box2d_joints.h"

/**
* @author Brian Semrau
*/

void register_box2d_types() {
	GLOBAL_DEF("physics/2d/box2d_conversion_factor", 50.0f);
	ProjectSettings::get_singleton()->set_custom_property_info("physics/2d/box2d_conversion_factor", PropertyInfo(Variant::REAL, "physics/2d/box2d_conversion_factor"));

	ClassDB::register_class<Box2DWorld>();
	ClassDB::register_class<Box2DPhysicsBody>();
	ClassDB::register_virtual_class<Box2DFixture>();
	ClassDB::register_class<Box2DCircleFixture>();
	ClassDB::register_class<Box2DRectFixture>();
	// TODO more fixtures
	ClassDB::register_virtual_class<Box2DJoint>();
	ClassDB::register_class<Box2DRevoluteJoint>();
	ClassDB::register_class<Box2DWeldJoint>();
	// TODO more joints
}

void unregister_box2d_types() {
}