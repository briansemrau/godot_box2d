#ifndef BOX2D_COLLISION_OBJECT_H
#define BOX2D_COLLISION_OBJECT_H

#include <core/resource.h>
#include <core/object.h>
#include <core/reference.h>
#include <core/vset.h>
#include <scene/2d/node_2d.h>

#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_world.h>

#include "../../util/box2d_types_converter.h"

/**
* @author Brian Semrau
*/

class Box2DWorld;
struct Box2DContactPoint;

class Box2DCollisionObject : public Node2D {
	GDCLASS(Box2DCollisionObject, Node2D);

	friend class Box2DWorld;
	friend class Box2DFixture;
	friend class Box2DJoint;

	b2Filter filterDef;

	b2Body *body = NULL;

	Box2DWorld *world_node = NULL;

	void on_parent_created(Node *);

	bool create_b2Body();
	bool destroy_b2Body();

	void update_filterdata();

protected:
	b2BodyDef bodyDef;

	struct ContactMonitor {
		// bool locked; // TODO when physics moved to separate thread
		VSet<Box2DContactPoint> contacts;

		// All the bodies/fixtures currently in contact with this body.
		// The int value stores the number of b2Fixtures currently in contact.
		// When the counter transitions from 0->1 or 1->0, body_entered/exited is emitted.
		HashMap<ObjectID, int> entered_objects;
	};

	ContactMonitor *contact_monitor = NULL;
	int max_contacts_reported = 0;

	inline Box2DWorld *_get_world_node() const { return world_node; };
	inline b2Body *_get_b2Body() const { return body; }

	virtual void on_b2Body_created(){};
	virtual void on_b2Body_destroyed(){};

	void _set_contact_monitor(bool p_enabled);
	bool _is_contact_monitor_enabled() const;

	virtual void _on_object_entered(Box2DCollisionObject *p_object) = 0;
	virtual void _on_object_exited(Box2DCollisionObject *p_object) = 0;
	virtual void _on_fixture_entered(Box2DFixture *p_fixture) = 0;
	virtual void _on_fixture_exited(Box2DFixture *p_fixture) = 0;

	virtual void pre_step(float p_delta){};

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	virtual String get_configuration_warning() const override;

	// Moving to and from world transform
	void set_box2dworld_transform(const Transform2D &p_transform);
	Transform2D get_box2dworld_transform() const;

	void set_enabled(bool p_enabled);
	bool is_enabled() const;

	void set_collision_layer(uint16_t p_layer);
	uint16_t get_collision_layer() const;

	void set_collision_mask(uint16_t p_mask);
	uint16_t get_collision_mask() const;

	void set_group_index(int16_t p_group_index);
	int16_t get_group_index() const;

	void set_filter_data(uint16_t p_layer, uint16_t p_mask, int16 p_group_index);

	//void set_contact_monitor(bool p_enabled);
	//bool is_contact_monitor_enabled() const;

	//void set_max_contacts_reported(int p_amount);
	//int get_max_contacts_reported() const;

	Array get_colliding_bodies() const; // Function exists for Godot feature congruency

	Box2DCollisionObject();
	~Box2DCollisionObject();
};

#endif // BOX2D_COLLISION_OBJECT_H
