#ifndef BOX2D_WORLD_H
#define BOX2D_WORLD_H

#include <core/object.h>
#include <core/reference.h>
#include <core/resource.h>
#include <scene/2d/node_2d.h>

#include <box2d/b2_contact.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_joint.h>
#include <box2d/b2_world.h>
#include <box2d/b2_world_callbacks.h>

#include "../../util/box2d_types_converter.h"

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

class Box2DShapeQueryParameters : public Reference {
	GDCLASS(Box2DShapeQueryParameters, Reference);

	Ref<Box2DShape> shape;
	Transform2D transform;
	//Vector2 motion; // TODO does Box2D support this?
	//Set<Ref<Box2DPhysicsBody>> exclude; // TODO figure out how to use nodes as parameters in bound methods
	uint32_t collision_mask;

	// TODO a bunch of shit ugh
};

class Box2DWorld;
class Box2DPhysicsBody;

class Box2DWorld : public Node2D, public virtual b2DestructionListener, public virtual b2ContactFilter, public virtual b2ContactListener {
	GDCLASS(Box2DWorld, Node2D);

	friend class Box2DPhysicsBody;
	friend class Box2DJoint;

private:
	// TODO Refactor this callback garbage.
	//      It may make sense to do this when/if shape queries are implemented.
	//      These at least need renamed.
	class QueryCallback : public b2QueryCallback {
	public:
		Vector<b2Fixture *> results;

		Box2DShapeQueryParameters params;

		virtual bool ReportFixture(b2Fixture *fixture) override;
	};

	class IntersectPointCallback : public b2QueryCallback {
	public:
		Vector<b2Fixture *> results;

		b2Vec2 point;
		int max_results;
		//Set<Ref<Box2DPhysicsBody> > exclude;

		virtual bool ReportFixture(b2Fixture *fixture) override;
	};

private:
	Vector2 gravity;
	bool auto_step{true};
	b2World *world;

	Set<Box2DPhysicsBody *> bodies;
	Set<Box2DJoint *> joints;

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

	QueryCallback aabbCallback;
	IntersectPointCallback pointCallback;

	void create_b2World();
	void destroy_b2World();

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void step(real_t p_step);

	void set_gravity(const Vector2 &gravity);
	Vector2 get_gravity() const;

	void set_auto_step(bool p_auto_step);
	bool get_auto_step() const;

	//bool isLocked() const;

	Array intersect_point(const Vector2 &p_point, int p_max_results = 32); //, const Vector<Ref<Box2DPhysicsBody> > &p_exclude = Vector<Ref<Box2DPhysicsBody> >() /*, uint32_t p_layers = 0*/);
	//Array intersect_shape();
	//Array query_aabb(const Rect2 &p_bounds); // TODO add more parameters like Physics2DDirectSpaceState::_intersect_point

	//void shiftOrigin(const Vector2 &newOrigin);

	// debugDraw

	Box2DWorld();
	~Box2DWorld();
};

#endif // BOX2D_WORLD_H
