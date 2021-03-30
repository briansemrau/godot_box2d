#include "box2d_collision_object.h"

#include <core/engine.h>

#include "box2d_world.h"
#include "box2d_fixtures.h"
#include "box2d_joints.h"

#include <vector>

/**
* @author Brian Semrau
*/

void Box2DCollisionObject::on_parent_created(Node *) {
	//if (create_b2Body()) {
	//	print_line("body created");
	//}
	WARN_PRINT("BODY CREATED IN CALLBACK");
}

bool Box2DCollisionObject::create_b2Body() {
	if (world_node && !body) {
		ERR_FAIL_COND_V(!world_node->world, false);

		// Create body
		bodyDef.position = gd_to_b2(get_box2dworld_transform().get_origin());
		bodyDef.angle = get_box2dworld_transform().get_rotation();

		body = world_node->world->CreateBody(&bodyDef);
		body->GetUserData().owner = this;

		on_b2Body_created();

		return true;
	}
	return false;
}

bool Box2DCollisionObject::destroy_b2Body() {
	if (body) {
		ERR_FAIL_COND_V(!world_node, false);
		ERR_FAIL_COND_V(!world_node->world, false);

		// Destroy body
		body->GetUserData().owner = NULL;
		world_node->world->DestroyBody(body);
		body = NULL;

		// b2Fixture destruction is handled by Box2D
		// b2Joint destruction is handled by Box2D

		on_b2Body_destroyed();

		return true;
	}
	return false;
}

void Box2DCollisionObject::update_filterdata() {
	if (body) {
		b2Fixture *fixture = body->GetFixtureList();
		while (fixture) {
			if (!fixture->GetUserData().owner->get_override_body_collision()) {
				fixture->SetFilterData(filterDef);
			}
			fixture = fixture->GetNext();
		}
	}
}

void Box2DCollisionObject::set_box2dworld_transform(const Transform2D &p_transform) {
	std::vector<Transform2D> transforms{};
	transforms.push_back(p_transform);
	Node *parent = get_parent();
	while (parent) {
		if (parent == world_node) {
			break;
		}
		CanvasItem *cv = Object::cast_to<CanvasItem>(parent);
		if (cv) {
			transforms.push_back(cv->get_transform().affine_inverse());
		}
		parent = parent->get_parent();
	}

	Transform2D target_xform{};
	while (transforms.size() > 0) {
		target_xform = target_xform * transforms.back();
		transforms.pop_back();
	}
	set_transform(target_xform);
}

Transform2D Box2DCollisionObject::get_box2dworld_transform() const {
	std::vector<Transform2D> transforms{};
	transforms.push_back(get_transform());
	Node *parent = get_parent();
	while (parent) {
		if (parent == world_node) {
			break;
		}
		CanvasItem *cv = Object::cast_to<CanvasItem>(parent);
		if (cv) {
			transforms.push_back(cv->get_transform());
		}
		parent = parent->get_parent();
	}

	Transform2D returned{};
	while (transforms.size() > 0) {
		returned = returned * transforms.back();
		transforms.pop_back();
	}

	return returned;
}

void Box2DCollisionObject::_set_contact_monitor(bool p_enabled) {
	if (p_enabled == _is_contact_monitor_enabled()) {
		return;
	}

	if (!p_enabled) {
		memdelete(contact_monitor);
		contact_monitor = NULL;
	} else {
		contact_monitor = memnew(ContactMonitor);
		//contact_monitor->locked = false;

		if (body) {
			world_node->flag_rescan_contacts_monitored = true;
		}
	}
}

bool Box2DCollisionObject::_is_contact_monitor_enabled() const {
	return contact_monitor != NULL;
}

void Box2DCollisionObject::_notification(int p_what) {
	// TODO finalize implementation to imitate behavior from RigidBody2D and Kinematic (static too?)
	switch (p_what) {
		case NOTIFICATION_PREDELETE: {
			if (world_node) {
				if (world_node->world) {
					destroy_b2Body();
				}
				world_node->body_owners.erase(this);
			}
		} break;

		case NOTIFICATION_ENTER_TREE: {
			// Find the Box2DWorld
			Node *_ancestor = get_parent();
			Box2DWorld *new_world = NULL;
			while (_ancestor && !new_world) {
				new_world = Object::cast_to<Box2DWorld>(_ancestor);
				_ancestor = _ancestor->get_parent();
			}

			// If new parent, recreate body
			if (new_world != world_node) {
				// Destroy b2Body
				if (world_node) {
					if (world_node->world) {
						destroy_b2Body();
					}
					world_node->body_owners.erase(this);
				}
				world_node = new_world;
				// Create b2Body
				if (world_node) {
					world_node->body_owners.insert(this);

					if (world_node->world) {
						create_b2Body();
					}
				}
			}

			if (Engine::get_singleton()->is_editor_hint() || get_tree()->is_debugging_collisions_hint()) {
				set_process_internal(true);
			}
		} break;

		case NOTIFICATION_EXIT_TREE: {
			// Don't destroy body. It could be exiting/entering.
			// Body should be destroyed in destructor if node is being freed.

			// TODO What do we do if it exits the tree, the ref is kept (in a script), and it's never destroyed?
			//      Exiting w/o reentering should destroy body.
			//      This applies to Box2DFixture and Box2DJoint as well.
			//
			// Perhaps body creation/destruction should be queued
			// - body created: immediate CreateBody request
			// - body destroyed: queue DestroyBody
			//     - recreated before pumping -> remove from queue
			// - world pre-step: pump queue

			set_process_internal(false);
		} break;
	}
}

void Box2DCollisionObject::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_enabled", "enabled"), &Box2DCollisionObject::set_enabled);
	ClassDB::bind_method(D_METHOD("is_enabled"), &Box2DCollisionObject::is_enabled);
	ClassDB::bind_method(D_METHOD("set_collision_layer", "collision_layer"), &Box2DCollisionObject::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &Box2DCollisionObject::get_collision_layer);
	ClassDB::bind_method(D_METHOD("set_collision_mask", "collision_mask"), &Box2DCollisionObject::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &Box2DCollisionObject::get_collision_mask);
	ClassDB::bind_method(D_METHOD("set_group_index", "group_index"), &Box2DCollisionObject::set_group_index);
	ClassDB::bind_method(D_METHOD("get_group_index"), &Box2DCollisionObject::get_group_index);

	ClassDB::bind_method(D_METHOD("set_filter_data", "collision_layer", "collision_mask", "group_index"), &Box2DCollisionObject::set_filter_data);
	
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enabled"), "set_enabled", "is_enabled");
	ADD_GROUP("Collision", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_2D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_2D_PHYSICS), "set_collision_mask", "get_collision_mask");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "group_index"), "set_group_index", "get_group_index");

	ADD_SIGNAL(MethodInfo("enabled_state_changed"));
}

String Box2DCollisionObject::get_configuration_warning() const {
	String warning = Node2D::get_configuration_warning();

	Node *_ancestor = get_parent();
	Box2DWorld *new_world = NULL;
	while (_ancestor && !new_world) {
		new_world = Object::cast_to<Box2DWorld>(_ancestor);
		_ancestor = _ancestor->get_parent();
	}

	if (!new_world) {
		if (warning != String()) {
			warning += "\n\n";
		}
		warning += TTR("Box2DCollisionObject only serves to provide bodies to a Box2DWorld node. Please only use it under the hierarchy of Box2DWorld.");
	}

	bool has_fixture_child = false;
	for (int i = 0; i < get_child_count(); i++) {
		if (Object::cast_to<Box2DFixture>(get_child(i))) {
			has_fixture_child = true;
			break;
		}
	}
	if (!has_fixture_child) {
		if (warning != String()) {
			warning += "\n\n";
		}
		warning += TTR("This node has no fixture, so it can't collide or interact with other objects.\nConsider adding a Box2DFixture subtype as a child to define its shape.");
	}

	return warning;
}

void Box2DCollisionObject::set_enabled(bool p_enabled) {
	if (body)
		body->SetEnabled(p_enabled);
	bodyDef.enabled = p_enabled;
}

bool Box2DCollisionObject::is_enabled() const {
	return bodyDef.enabled;
}

void Box2DCollisionObject::set_collision_layer(uint16_t p_layer) {
	if (filterDef.categoryBits != p_layer) {
		filterDef.categoryBits = p_layer;
		update_filterdata();
	}
}

uint16_t Box2DCollisionObject::get_collision_layer() const {
	return filterDef.categoryBits;
}

void Box2DCollisionObject::set_collision_mask(uint16_t p_mask) {
	if (filterDef.maskBits != p_mask) {
		filterDef.maskBits = p_mask;
		update_filterdata();
	}
}

uint16_t Box2DCollisionObject::get_collision_mask() const {
	return filterDef.maskBits;
}

void Box2DCollisionObject::set_group_index(int16_t p_group_index) {
	if (filterDef.groupIndex != p_group_index) {
		filterDef.groupIndex = p_group_index;
		update_filterdata();
	}
}

int16_t Box2DCollisionObject::get_group_index() const {
	return filterDef.groupIndex;
}

void Box2DCollisionObject::set_filter_data(uint16_t p_layer, uint16_t p_mask, int16 p_group_index) {
	if (filterDef.categoryBits != p_layer || filterDef.maskBits != p_mask || filterDef.groupIndex != p_group_index) {
		filterDef.categoryBits = p_layer;
		filterDef.maskBits = p_mask;
		filterDef.groupIndex = p_group_index;
		update_filterdata();
	}
}

Array Box2DCollisionObject::get_colliding_bodies() const {
	ERR_FAIL_COND_V(!contact_monitor, Array());
	Array ret;

	List<ObjectID> keys;
	contact_monitor->entered_objects.get_key_list(&keys);
	for (int i = 0; i < keys.size(); i++) {
		Object *node = ObjectDB::get_instance(keys[i]);
		if (node && Object::cast_to<Box2DCollisionObject>(node)) {
			ret.append(node);
		}
	}

	return ret;
}

Box2DCollisionObject::Box2DCollisionObject() {
	filterDef.maskBits = 0x0001;

	set_physics_process_internal(true);
	set_notify_local_transform(true);
}

Box2DCollisionObject::~Box2DCollisionObject() {
	if (body && world_node) {
		WARN_PRINT("b2Body is being deleted in destructor, not NOTIFICATION_PREDELETE.");
		destroy_b2Body();
	} // else Box2D has/will clean up body
}
