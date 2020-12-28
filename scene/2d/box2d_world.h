#ifndef BOX2D_WORLD_H
#define BOX2D_WORLD_H

#include <core/io/resource.h>
#include <core/object/object.h>
#include <core/object/reference.h>
#include <core/templates/set.h>
#include <core/templates/vset.h>
#include <scene/2d/node_2d.h>

#include <box2d/b2_contact.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_joint.h>
#include <box2d/b2_world.h>
#include <box2d/b2_world_callbacks.h>

#include "../../util/box2d_types_converter.h"
#include "box2d_collision_object.h"

#include <deque>
#include <unordered_set>

/**
* @author Brian Semrau
*/

class Box2DShape;

struct Box2DContactPoint {
	// This ID is required for inserting this object into a VSet
	int32_t id = -1;

	int solves = 0; // TODO might belong inside ContactBufferManifold, but probably not
	Box2DFixture *fixture_a = NULL;
	Box2DFixture *fixture_b = NULL;
	Vector2 world_pos = Vector2();
	Vector2 impact_velocity = Vector2();
	Vector2 normal = Vector2();
	float normal_impulse = 0;
	Vector2 tangent_impulse = Vector2();

	bool operator<(const Box2DContactPoint &p_c) const {
		return id < p_c.id;
	}

	inline void reset_accum() {
		solves = 0;
	}

	inline Box2DContactPoint flipped_a_b() const {
		Box2DContactPoint ret(*this);
		ret.fixture_a = fixture_b;
		ret.fixture_b = fixture_a;
		ret.normal = -ret.normal;
		ret.tangent_impulse = -ret.tangent_impulse;
		return ret;
	}
};

struct ContactBufferManifold {
	Box2DContactPoint points[b2_maxManifoldPoints];
	int count = 0;

	// TODO Optimize? These functions may be overkill, but everything currently works this way

	inline void insert(Box2DContactPoint &p_point, int p_idx) {
		ERR_FAIL_COND(count + 1 > b2_maxManifoldPoints);
		ERR_FAIL_COND(p_idx < 0 || p_idx >= b2_maxManifoldPoints);
		ERR_FAIL_COND(p_idx > count); // Can't insert a point leaving a null at the index below

		// Shift points up
		if (p_idx < count) {
			for (int i = count; i > p_idx; --i) {
				points[i] = points[i - 1];
			}
		}
		points[p_idx] = p_point;

		++count;
	}

	inline void remove(int p_idx) {
		ERR_FAIL_COND(p_idx < 0 || p_idx >= count || p_idx >= b2_maxManifoldPoints);

		// Shift points down
		if (p_idx < count - 1) {
			for (int i = p_idx; i < count - 1; ++i) {
				// There's a buffer overflow warning for this line but I don't believe it
				points[i] = points[i + 1];
			}
			points[count - 1].id = -1;
		}

		--count;
	}
};

class Box2DWorld;
class Box2DPhysicsBody;

class Box2DShapeQueryParameters : public Reference {
	GDCLASS(Box2DShapeQueryParameters, Reference);

	friend class Box2DWorld;

	Ref<Box2DShape> shape_ref;
	Transform2D transform = Transform2D();

	Set<Box2DPhysicsBody *> exclude;
	// potential addition: exclude fixtures
	b2Filter filter; // TODO If/when we fork Box2D, filters get 32bit data
	bool collide_with_bodies = true; // TODO might be better named as "collide_with_solids"
	bool collide_with_sensors = false;

	// Properties exclusive for cast_motion
	Vector2 motion = Vector2(0, 0);
	float rotation = 0.0f;
	Vector2 local_center = Vector2(0, 0);
	//bool predict_other_body_motion = false;
	//float motion_timedelta_for_prediction = 1/60;

protected:
	static void _bind_methods();

public:
	const b2Filter &_get_filter() const;
	Set<Box2DPhysicsBody *> _get_exclude() const;

	void set_shape(const RES &p_shape_ref);
	RES get_shape() const;

	void set_transform(const Transform2D &p_transform);
	Transform2D get_transform() const;

	void set_motion(const Vector2 &p_motion);
	Vector2 get_motion() const;

	void set_motion_rotation(float p_rotation);
	float get_motion_rotation() const;

	void set_motion_transform(const Transform2D &p_transform);
	Transform2D get_motion_transform() const;

	void set_motion_local_center(const Vector2 &p_local_center);
	Vector2 get_motion_local_center() const;

	void set_collision_layer(int p_layer);
	int get_collision_layer() const;

	void set_collision_mask(int p_collision_mask);
	int get_collision_mask() const;

	void set_group_index(int p_index);
	int get_group_index() const;

	void set_collide_with_bodies(bool p_enable);
	bool is_collide_with_bodies_enabled() const;

	void set_collide_with_sensors(bool p_enable);
	bool is_collide_with_sensors_enabled() const;

	// Using ObjectIDs as int64_t so that we can bind these methods
	void set_exclude(const Vector<int64_t> &p_exclude);
	Vector<int64_t> get_exclude() const;
};

class Box2DWorld : public Node2D, public virtual b2DestructionListener, public virtual b2ContactFilter, public virtual b2ContactListener {
	GDCLASS(Box2DWorld, Node2D);

	friend class Box2DCollisionObject;
	friend class Box2DJoint;

private:
	class PointQueryCallback : public b2QueryCallback {
	public:
		Set<Box2DFixture *> results; // Use a set so composite fixtures don't double-count towards max_results

		b2Vec2 point;
		int max_results;
		Set<Box2DPhysicsBody *> exclude;
		b2Filter filter;
		bool collide_with_bodies;
		bool collide_with_sensors;

		virtual bool ReportFixture(b2Fixture *fixture) override;
	};

	class ShapeQueryCallback : public b2QueryCallback {
	public:
		Set<Box2DFixture *> results;

		Ref<Box2DShapeQueryParameters> params;
		int max_results;

		virtual bool ReportFixture(b2Fixture *fixture) override;
	};

	class RaycastQueryCallback : public b2RayCastCallback {
	public:
		struct Result {
			b2Fixture *fixture = NULL;
			b2Vec2 point;
			b2Vec2 normal;
			// float fraction; // TODO maybe we'll want this
		};

		Result result;

		Set<Box2DPhysicsBody *> exclude;
		b2Filter filter;
		bool collide_with_bodies;
		bool collide_with_sensors;

		virtual float ReportFixture(b2Fixture *fixture, const b2Vec2 &point, const b2Vec2 &normal, float fraction) override;
	};

	class UserAABBQueryCallback : public b2QueryCallback {
	public:
		std::unordered_set<const Box2DFixture *> handled_fixtures;
		const Callable *callback = NULL;

		virtual bool ReportFixture(b2Fixture *fixture) override;
	};

	class UserRaycastQueryCallback : public b2RayCastCallback {
	public:
		std::unordered_set<const Box2DFixture *> handled_fixtures;
		const Callable *callback = NULL;

		virtual float ReportFixture(b2Fixture *fixture, const b2Vec2 &point, const b2Vec2 &normal, float fraction) override;
	};

	template <class P, void (Box2DCollisionObject::*on_thing_inout)(P *)>
	class CollisionUpdateQueue {
	private:
		struct CollisionUpdatePair {
			Box2DCollisionObject *function_owner;
			P *transient;
		};
		std::deque<CollisionUpdatePair> queue{};

	public:
		inline void enqueue(Box2DCollisionObject *p_caller, P *p_transient) {
			queue.push_back({ p_caller, p_transient });
		}

		inline void call_and_clear() {
			while (!queue.empty()) {
				CollisionUpdatePair *pair = &queue.front();
				(pair->function_owner->*on_thing_inout)(pair->transient);
				queue.pop_front();
			}
		}
	};

private:
	Vector2 gravity;
	bool auto_step{true};
	b2World *world = NULL;

	CollisionUpdateQueue<Box2DCollisionObject, &Box2DCollisionObject::_on_object_entered> object_entered_queue;
	CollisionUpdateQueue<Box2DCollisionObject, &Box2DCollisionObject::_on_object_exited> object_exited_queue;
	CollisionUpdateQueue<Box2DFixture, &Box2DCollisionObject::_on_fixture_entered> fixture_entered_queue;
	CollisionUpdateQueue<Box2DFixture, &Box2DCollisionObject::_on_fixture_exited> fixture_exited_queue;

	Set<Box2DCollisionObject *> body_owners;
	Set<Box2DJoint *> joint_owners;

	virtual void SayGoodbye(b2Joint *joint) override;
	virtual void SayGoodbye(b2Fixture *fixture) override;

	virtual bool ShouldCollide(b2Fixture *fixtureA, b2Fixture *fixtureB) override;

	int32_t next_contact_id = 0;
	bool flag_rescan_contacts_monitored = false;
	HashMap<uint64_t, ContactBufferManifold> contact_buffer;

	inline void try_buffer_contact(b2Contact *contact, int i);

	virtual void BeginContact(b2Contact *contact) override;
	virtual void EndContact(b2Contact *contact) override;
	virtual void PreSolve(b2Contact *contact, const b2Manifold *oldManifold) override;

	/// This lets you inspect a contact after the solver is finished. This is useful
	/// for inspecting impulses.
	/// Note: the contact manifold does not include time of impact impulses, which can be
	/// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
	/// in a separate data structure.
	/// Note: this is only called for contacts that are touching, solid, and awake.
	virtual void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) override;

	PointQueryCallback point_callback;
	RaycastQueryCallback ray_callback;
	ShapeQueryCallback shape_callback;

	UserAABBQueryCallback user_query_callback;
	UserRaycastQueryCallback user_raycast_callback;

	void create_b2World();
	void destroy_b2World();

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	enum {
		NOTIFICATION_WORLD_STEPPED = 42300, // special int that shouldn't clobber other notifications.  See node.h
	};

	void step(float p_step);

	void set_gravity(const Vector2 &gravity);
	Vector2 get_gravity() const;

	void set_auto_step(bool p_auto_step);
	bool get_auto_step() const;

	//bool isLocked() const;

	//void shiftOrigin(const Vector2 &newOrigin);

	// Godot space query API
	// What is collide_shape? Does this return manifold points?
	//Array collide_shape(const Ref<Box2DShapeQueryParameters> &p_query, int p_max_results = 32);
	Array intersect_point(const Vector2 &p_point, int p_max_results = 32, const Vector<int64_t> &p_exclude = Vector<int64_t>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_sensors = false, uint32_t p_collision_layer = 0x0, int32_t p_group_index = 0);
	Dictionary intersect_ray(const Vector2 &p_from, const Vector2 &p_to, const Vector<int64_t> &p_exclude = Vector<int64_t>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_sensors = false, uint32_t p_collision_layer = 0x0, int32_t p_group_index = 0);
	Array intersect_shape(const Ref<Box2DShapeQueryParameters> &p_query, int p_max_results = 32);
	Array cast_motion(const Ref<Box2DShapeQueryParameters> &p_query);

	// Box2D space query API
	void query_aabb(const Rect2 &p_aabb, const Callable &p_callback);
	void raycast(const Vector2 &p_from, const Vector2 &p_to, const Callable &p_callback);

	Box2DWorld();
	~Box2DWorld();
};

#endif // BOX2D_WORLD_H
