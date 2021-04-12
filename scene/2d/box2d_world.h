#ifndef BOX2D_WORLD_H
#define BOX2D_WORLD_H

#include <core/resource.h>
#include <core/object.h>
#include <core/reference.h>
#include <core/set.h>
#include <core/vset.h>
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

	inline void set(Box2DContactPoint &p_point, int p_idx) {
		ERR_FAIL_COND(p_idx < 0 || p_idx >= b2_maxManifoldPoints);
		points[p_idx] = p_point;
	}

	inline void erase(int p_idx) {
		ERR_FAIL_COND(p_idx < 0 || p_idx >= b2_maxManifoldPoints);
		points[p_idx].id = -1;
	}

	inline bool is_empty() {
		for (int i = 0; i < b2_maxManifoldPoints; ++i) {
			if (points[i].id != -1) {
				return false;
			}
		}
		return true;
	}
};

class Box2DWorld;
class Box2DPhysicsBody;

struct MotionQueryParameters {
	Transform2D transform = Transform2D();

	Set<const Box2DPhysicsBody *> exclude;
	// potential addition: exclude fixtures
	b2Filter filter; // TODO If/when we fork Box2D, filters get 32bit data
	bool collide_with_bodies = true; // TODO might be better named as "collide_with_solids"
	bool collide_with_sensors = false;
	bool ignore_dynamic = false;

	// Properties exclusive for cast_motion
	Vector2 motion = Vector2(0, 0);
	float rotation = 0.0f; // TODO should these be combined to Transform2D?
	Vector2 local_center = Vector2(0, 0); // TODO rename shape_local_center
	//float motion_timedelta_for_prediction = 1/60;
};

class Box2DShapeQueryParameters : public Reference {
	GDCLASS(Box2DShapeQueryParameters, Reference);

	friend class Box2DWorld;

	Ref<Box2DShape> shape_ref;
	MotionQueryParameters parameters;

protected:
	static void _bind_methods();

public:
	const b2Filter &_get_filter() const;
	Set<const Box2DPhysicsBody *> _get_exclude() const;

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

	void set_ignore_rigid(bool p_enable);
	bool is_ignoring_rigid() const;

	// Using ObjectIDs (int64_t) in an Array so that we can bind these methods
	void set_exclude(const Array &p_exclude);
	Array get_exclude() const;
};

class Box2DPhysicsTestMotionResult;

class Box2DWorld : public Node2D, public virtual b2DestructionListener, public virtual b2ContactFilter, public virtual b2ContactListener {
	GDCLASS(Box2DWorld, Node2D);

	friend class Box2DCollisionObject;
	friend class Box2DJoint;

public:
	struct MotionResult {
		Vector2 motion;
		Vector2 remainder;
		float t; // TOI with respect to motion [0, 1]
		bool colliding;

		Vector2 collision_point;
		Vector2 collision_normal;
		Vector2 collider_velocity;
		Box2DFixture *collider_fixture = nullptr;
		Box2DFixture *local_fixture = nullptr;
	};

private:
	class PointQueryCallback : public b2QueryCallback {
	public:
		Set<Box2DFixture *> results; // Use a set so composite fixtures don't double-count towards max_results

		b2Vec2 point;
		int max_results;
		Set<const Box2DPhysicsBody *> exclude;
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

		Set<const Box2DPhysicsBody *> exclude;
		b2Filter filter;
		bool collide_with_bodies;
		bool collide_with_sensors;

		virtual float ReportFixture(b2Fixture *fixture, const b2Vec2 &point, const b2Vec2 &normal, float fraction) override;
	};

	class UserAABBQueryCallback : public b2QueryCallback {
	public:
		std::unordered_set<const Box2DFixture *> handled_fixtures;
		Object *callback_owner = NULL;
		String callback_func;

		virtual bool ReportFixture(b2Fixture *fixture) override;
	};

	class UserRaycastQueryCallback : public b2RayCastCallback {
	public:
		std::unordered_set<const Box2DFixture *> handled_fixtures;
		Object *callback_owner = NULL;
		String callback_func;

		virtual float ReportFixture(b2Fixture *fixture, const b2Vec2 &point, const b2Vec2 &normal, float fraction) override;
	};

	struct CastQueryWrapper { // TODO rename this. It's no longer relevant to just cast_motion
		const b2BroadPhase *broadPhase;

		MotionQueryParameters params;
		int max_results = -1;

		Vector<b2FixtureProxy *> results;

		bool QueryCallback(int32 proxyId);
	};
	
	template <void (Box2DCollisionObject::*on_object_inout)(Box2DCollisionObject *)>
	class ObjectCollisionUpdateQueue {
	private:
		struct CollisionUpdatePair {
			Box2DCollisionObject *function_owner;
			Box2DCollisionObject *transient;
		};
		std::deque<CollisionUpdatePair> queue{};

	public:
		inline void enqueue(Box2DCollisionObject *p_caller, Box2DCollisionObject *p_transient) {
			queue.push_back({ p_caller, p_transient });
		}

		inline void call_immediate(Box2DCollisionObject *p_caller, Box2DCollisionObject *p_transient) {
			(p_caller->*on_object_inout)(p_transient);
		}

		inline void call_and_clear() {
			while (!queue.empty()) {
				CollisionUpdatePair *pair = &queue.front();
				(pair->function_owner->*on_object_inout)(pair->transient);
				queue.pop_front();
			}
		}
	};

	template <void (Box2DCollisionObject::*on_fixture_inout)(Box2DFixture *, Box2DFixture *)>
	class FixtureCollisionUpdateQueue {
	private:
		struct CollisionUpdatePair {
			Box2DCollisionObject *function_owner;
			Box2DFixture *transient;
			Box2DFixture *self;
		};
		std::deque<CollisionUpdatePair> queue{};

	public:
		inline void enqueue(Box2DCollisionObject *p_caller, Box2DFixture *p_transient, Box2DFixture *p_self) {
			queue.push_back({ p_caller, p_transient, p_self });
		}

		inline void call_immediate(Box2DCollisionObject *p_caller, Box2DFixture *p_transient, Box2DFixture *p_self) {
			(p_caller->*on_fixture_inout)(p_transient, p_self);
		}

		inline void call_and_clear() {
			while (!queue.empty()) {
				CollisionUpdatePair *pair = &queue.front();
				(pair->function_owner->*on_fixture_inout)(pair->transient, pair->self);
				queue.pop_front();
			}
		}
	};

private:
	Vector2 gravity;
	bool auto_step{true};

	b2World *world = NULL;

	float last_step_delta = 0.0f;

	ObjectCollisionUpdateQueue<&Box2DCollisionObject::_on_object_entered> object_entered_queue;
	ObjectCollisionUpdateQueue<&Box2DCollisionObject::_on_object_exited> object_exited_queue;
	FixtureCollisionUpdateQueue<&Box2DCollisionObject::_on_fixture_entered> fixture_entered_queue;
	FixtureCollisionUpdateQueue<&Box2DCollisionObject::_on_fixture_exited> fixture_exited_queue;

	// TODO make sure these are using the best data structure
	Set<Box2DCollisionObject *> body_owners;
	Set<Box2DJoint *> joint_owners;

	// b2World Callbacks
	// TODO extract into classes

	virtual void SayGoodbye(b2Joint *joint) override;
	virtual void SayGoodbye(b2Fixture *fixture) override;

	virtual bool ShouldCollide(b2Fixture *fixtureA, b2Fixture *fixtureB) override;

	// TODO class ContactManager
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
	// end TODO

	PointQueryCallback point_callback;
	RaycastQueryCallback ray_callback;
	ShapeQueryCallback shape_callback;

	UserAABBQueryCallback user_query_callback;
	UserRaycastQueryCallback user_raycast_callback;

	void create_b2World();
	void destroy_b2World();

	struct TestMotionTOIResult {
		bool collision;

		b2Fixture *col_fixture;
		int col_child_index;

		int test_shape_index;
		int test_shape_child_index;

		int manifold_pt_count;
		b2WorldManifold manifold;
	};

	float _test_motion_toi(const Vector<const b2Shape *> &p_test_shapes, const MotionQueryParameters &p_params, TestMotionTOIResult *r_result);

	bool _solve_position_step(const Vector<const b2Shape *> &p_body_shapes, const MotionQueryParameters &p_params, b2Vec2 &r_correction) const;
	b2Vec2 _solve_position(const Vector<const b2Shape *> &p_body_shapes, const MotionQueryParameters &p_params, int p_solve_steps = 4) const;

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void step(float p_step, int32 velocity_iterations = 8, int32 position_iterations = 8);

	float get_last_step_delta() const;

	void set_gravity(const Vector2 &gravity);
	Vector2 get_gravity() const;

	void set_auto_step(bool p_auto_step);
	bool get_auto_step() const;

	//bool isLocked() const;

	//void shiftOrigin(const Vector2 &newOrigin);

	// Godot space query API
	// TODO What is collide_shape? Does this return manifold points? //Array collide_shape(const Ref<Box2DShapeQueryParameters> &p_query, int p_max_results = 32);
	Array intersect_point(const Vector2 &p_point, int p_max_results = 32, const Array &p_exclude = Array(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_sensors = false, uint32_t p_collision_layer = 0x0, int32_t p_group_index = 0);
	Dictionary intersect_ray(const Vector2 &p_from, const Vector2 &p_to, const Array &p_exclude = Array(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_sensors = false, uint32_t p_collision_layer = 0x0, int32_t p_group_index = 0);
	Array intersect_shape(const Ref<Box2DShapeQueryParameters> &p_query, int p_max_results = 32);
	Array cast_motion(const Ref<Box2DShapeQueryParameters> &p_query);

	// This is by-default continuous collision. Is this slow? TODO test or remove commented code
	bool body_test_motion(const Box2DPhysicsBody *p_body, const Transform2D &p_from, const Vector2 &p_motion, bool p_infinite_inertia, MotionResult *r_result = nullptr);
	bool _body_test_motion_binding(const Object *p_body, const Transform2D &p_from, const Vector2 &p_motion, bool p_infinite_inertia, const Ref<Box2DPhysicsTestMotionResult> &r_result = Ref<Box2DPhysicsTestMotionResult>());

	// Box2D space query API
	void query_aabb(const Rect2 &p_aabb, Object *p_callback_owner, const String &p_callback_func);
	void raycast(const Vector2 &p_from, const Vector2 &p_to, Object *p_callback_owner, const String &p_callback_func);

	Box2DWorld();
	~Box2DWorld();
};

class Box2DPhysicsTestMotionResult : public Reference {
	GDCLASS(Box2DPhysicsTestMotionResult, Reference);

	friend class Box2DWorld;

	Box2DWorld::MotionResult result;

protected:
	static void _bind_methods();

public:
	bool is_colliding() const;
	Vector2 get_motion() const;
	Vector2 get_motion_remainder() const;

	Vector2 get_collision_point() const;
	Vector2 get_collision_normal() const;
	Vector2 get_collider_velocity() const;
	Box2DFixture *get_collider_fixture() const;
	ObjectID get_collider_fixture_id() const;
	Box2DFixture *get_local_fixture() const;
	ObjectID get_local_fixture_id() const;
};

#endif // BOX2D_WORLD_H
